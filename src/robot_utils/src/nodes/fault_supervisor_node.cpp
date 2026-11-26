/**
 * @file fault_supervisor_node.cpp
 * @brief 故障监督器节点（节点入口层）：把 /system/health 转成急停动作。
 * @details 所属包：`robot_utils`；所在层次：节点入口层。
 *
 * 分工：system_monitor_node 只负责“判断是否健康”，本节点负责“健康出问题时做什么”，
 *       二者分开，避免一处既判定又执行、难以单独测试。
 *
 * 输入：/system/health（robot_interfaces_core/RobotState，state ∈ OK|WARN|ERROR）
 * 输出：/system/fault_state（同样的 RobotState，反映监督器自身状态机）
 * 动作：调用 /teleop/enable_emergency_stop / /teleop/clear_emergency_stop（data=true）
 *
 * 状态机：
 *   WAITING     还没收到 /system/health，或急停服务尚未就绪
 *   ARMING      启动宽限期内（startup_grace_ms）且健康为 ERROR：先不动作，避免启动瞬态误触发
 *   FAULT       健康为 ERROR 且已请求急停（recoverable=false）
 *   RECOVERING  健康恢复且允许清除，已请求解除急停
 *   CLEAR       健康非 ERROR，无动作
 *
 * 设计要点：
 *   * 所有服务调用都带冷却（command_cooldown_ms），避免 100 ms 周期里刷爆服务；
 *   * 单线程 spin，状态变量无需加锁；
 *   * 急停是“安全侧”动作：宁可多停一次，也不要在 ERROR 时继续运行。
 */

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces_core/msg/robot_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "robot_utils/config/topic_names.hpp"

namespace topics = robot_utils::topics;

using namespace std::chrono_literals;

class FaultSupervisorNode final : public rclcpp::Node
{
public:
  /**
   * @brief 构造节点：声明参数、建立发布/订阅与服务客户端、启动 100 ms 监督定时器。
   * @details 参数：
   *          auto_clear           健康恢复后是否自动解除急停（默认 true）
   *          command_cooldown_ms  两次急停/解除命令之间的最小间隔（毫秒，默认 500）
   *          startup_grace_ms     启动宽限期（毫秒，默认 2000），期间 ERROR 不触发急停
   */
  FaultSupervisorNode() : Node("fault_supervisor_node")
  {
    auto_clear_ = declare_parameter<bool>("auto_clear", true);
    command_cooldown_ms_ = declare_parameter<int>("command_cooldown_ms", 500);
    startup_grace_ms_ = declare_parameter<int>("startup_grace_ms", 2000);
    health_timeout_ms_ = declare_parameter<int>("health_timeout_ms", 2000);
    if (health_timeout_ms_ <= 0)
    {
      RCLCPP_WARN(get_logger(), "health_timeout_ms must be > 0, using 2000");
      health_timeout_ms_ = 2000;
    }
    started_stamp_ = now();

    state_publisher_ = create_publisher<robot_interfaces_core::msg::RobotState>(topics::kSystemFaultState, 10);
    health_subscription_ = create_subscription<robot_interfaces_core::msg::RobotState>(
        topics::kSystemHealth, 10, [this](robot_interfaces_core::msg::RobotState::SharedPtr msg)
        {
          last_health_ = *msg;
          health_seen_ = true;
          last_health_stamp_ = now(); });
    enable_estop_client_ =
        create_client<std_srvs::srv::SetBool>("/teleop/enable_emergency_stop");
    clear_estop_client_ =
        create_client<std_srvs::srv::SetBool>("/teleop/clear_emergency_stop");
    timer_ = create_wall_timer(100ms, [this]()
                               { Tick(); });
  }

private:
  /**
   * @brief 冷却是否结束（可否再发一次急停/解除命令）。
   * @return 从未发过命令，或距上次命令已超过 command_cooldown_ms_ 时返回 true
   */
  bool CooldownReady() const
  {
    if (last_command_stamp_.nanoseconds() == 0)
    {
      return true;
    }
    return ((now() - last_command_stamp_).nanoseconds() / 1000000) >= command_cooldown_ms_;
  }

  /**
   * @brief 监督主循环（每 100 ms 一次），按状态机决定是否动作。
   * @details 判定顺序很重要：未收到健康 → 启动宽限 → ERROR → 已故障且可清除 → 正常。
   *          顺序颠倒会出现“启动瞬态直接急停”或“刚恢复又被判故障”的抖动。
   */
  void Tick()
  {
    if (!health_seen_)
    {
      SetState("WAITING", "waiting for /system/health", true);
      return;
    }
    if (InStartupGrace() && (last_health_.state == "ERROR" || HealthStale()))
    {
      SetState("ARMING", "startup grace active: " + last_health_.message, true);
      return;
    }
    if (HealthStale())
    {
      // 监控节点死亡 / 断流：宁可急停，也不能把“没有结论”当成“健康”。
      fault_active_ = true;
      if (!emergency_stop_requested_ && CooldownReady())
      {
        RequestEmergencyStop();
      }
      SetState("FAULT",
               "system health stale: no /system/health within " +
                   std::to_string(health_timeout_ms_) + " ms",
               false);
      return;
    }
    if (last_health_.state == "ERROR")
    {
      fault_active_ = true;
      if (!emergency_stop_requested_ && CooldownReady())
      {
        RequestEmergencyStop();
      }
      SetState("FAULT", "emergency stop requested: " + last_health_.message, false);
      return;
    }
    if (fault_active_ && auto_clear_ && HealthAllowsClear())
    {
      if (emergency_stop_requested_ && CooldownReady())
      {
        RequestEmergencyClear();
      }
      SetState("RECOVERING", "system health allows emergency stop clear", true);
      return;
    }
    SetState("CLEAR", "system health " + last_health_.state, true);
  }

  /**
   * @brief 请求置位急停（异步调用安全闸门的 enable 服务）。
   * @details 服务未就绪时只发 WAITING 状态，不置位 emergency_stop_requested_，
   *          下个周期会重试；服务调用成功后记录时间戳用于冷却。
   * @note 这里不等待响应结果（fire and forget）：急停是安全动作，
   *       重复下发比“等确认再发”更安全。
   */
  void RequestEmergencyStop()
  {
    if (!enable_estop_client_->service_is_ready())
    {
      SetState("WAITING", "waiting for /teleop/enable_emergency_stop", true);
      return;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    enable_estop_client_->async_send_request(request);
    emergency_stop_requested_ = true;
    last_command_stamp_ = now();
  }

  /**
   * @brief 请求解除急停（异步调用安全闸门的 clear 服务）。
   * @details clear 服务只接受 data=true（安全闸门做严格校验），这里固定传 true。
   *          成功后清空 fault_active_，允许回到 CLEAR。
   */
  void RequestEmergencyClear()
  {
    if (!clear_estop_client_->service_is_ready())
    {
      SetState("WAITING", "waiting for /teleop/clear_emergency_stop", true);
      return;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    clear_estop_client_->async_send_request(request);
    emergency_stop_requested_ = false;
    fault_active_ = false;
    last_command_stamp_ = now();
  }

  /**
   * @brief 是否仍处于启动宽限期。
   * @return 距节点启动不足 startup_grace_ms_ 时返回 true
   * @details 启动瞬间各节点尚未发布数据，健康结论可能是 ERROR；
   *          宽限期内只报 ARMING，不真的拉急停。
   */
  bool InStartupGrace() const
  {
    return ((now() - started_stamp_).nanoseconds() / 1000000) < startup_grace_ms_;
  }

  /**
   * @brief /system/health 是否已断流。
   * @return 已收到过健康消息且超过 health_timeout_ms_ 未更新时返回 true
   * @details 从未收到过返回 false：启动阶段由 WAITING 负责，不把“尚未开始”当成“断流”。
   */
  bool HealthStale() const
  {
    if (!health_seen_)
    {
      return false;
    }
    const auto age_ms = (now() - last_health_stamp_).nanoseconds() / 1000000;
    return age_ms > health_timeout_ms_;
  }

  /**
   * @brief 当前健康状态是否允许解除急停。
   * @return 健康为 OK 时 true；健康为 WARN 且原因**只是**“急停激活”时也 true
   * @details 后者是必须的特例：急停激活会让 health 变成 WARN，
   *          若不排除这一点，解除急停后又被判“可清除”的条件永远无法满足，
   *          会卡在 FAULT 里出不来。
   */
  bool HealthAllowsClear() const
  {
    if (last_health_.state == "OK")
    {
      return true;
    }
    return last_health_.state == "WARN" &&
           last_health_.message.find("emergency stop active") != std::string::npos;
  }

  /**
   * @brief 更新内部状态并立即发布。
   * @param state 状态机状态（WAITING/ARMING/FAULT/RECOVERING/CLEAR）
   * @param message 人类可读原因（含触发该项的健康摘要）
   * @param recoverable 是否可自动恢复
   */
  void SetState(const std::string &state, const std::string &message, const bool recoverable)
  {
    current_state_ = state;
    current_message_ = message;
    recoverable_ = recoverable;
    PublishState();
  }

  /**
   * @brief 发布 /system/fault_state。
   * @details active_goal_id 恒为空（监督器不承载任务）；previous_state 记录上一拍状态，
   *          便于用 topic echo 观察状态跳变。
   */
  void PublishState()
  {
    robot_interfaces_core::msg::RobotState state;
    state.header.stamp = now();
    state.state = current_state_;
    state.previous_state = previous_state_;
    state.active_goal_id = "";
    state.message = current_message_;
    state.recoverable = recoverable_;
    state_publisher_->publish(state);
    previous_state_ = current_state_;
  }

  // ---- 参数 ----
  bool auto_clear_ = true;
  int command_cooldown_ms_ = 500;
  int startup_grace_ms_ = 2000;
  int health_timeout_ms_ = 2000;
  // ---- 运行状态 ----
  bool health_seen_ = false;
  bool fault_active_ = false;
  bool emergency_stop_requested_ = false;
  bool recoverable_ = true;
  std::string current_state_ = "WAITING";
  std::string previous_state_ = "UNKNOWN";
  std::string current_message_ = "waiting for /system/health";
  // ---- 输入快照与时间戳 ----
  robot_interfaces_core::msg::RobotState last_health_;
  rclcpp::Time started_stamp_;
  rclcpp::Time last_health_stamp_;
  rclcpp::Time last_command_stamp_;
  rclcpp::Publisher<robot_interfaces_core::msg::RobotState>::SharedPtr state_publisher_;
  rclcpp::Subscription<robot_interfaces_core::msg::RobotState>::SharedPtr health_subscription_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_estop_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr clear_estop_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FaultSupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
