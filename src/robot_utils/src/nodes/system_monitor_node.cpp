/**
 * @file system_monitor_node.cpp
 * @brief 系统监控节点（节点入口层）：把多路状态汇成单一健康结论 + 诊断项。
 * @details 所属包：`robot_utils`；所在层次：节点入口层。
 *
 * 输入（订阅，只记录“是否收到”与“收到时刻”）：
 *   /chassis/state            底盘连接/故障码/电池电压
 *   /system/emergency_stop    急停状态（std_msgs/Bool）
 *   /scan、/imu/data、/odom   数据链路的“是否还在更新”
 *
 * 输出：
 *   /system/health            robot_interfaces_core/RobotState，state ∈ OK|WARN|ERROR
 *                             —— fault_supervisor 据此决定是否触发急停
 *   /diagnostics              diagnostic_updater 发布的 6 个诊断项
 *                             （system/chassis、system/battery、system/emergency_stop、
 *                               system/scan、system/imu、system/odom）
 *                             —— diagnostic_aggregator 据此归类成 /Robot 诊断树
 *
 * 设计要点：
 *   * 新鲜度优先：任何一路超时都先按“不可用”处理，而不是沿用最后一次的值；
 *   * 单线程 spin：订阅回调与定时器不会并发，成员变量无需加锁；
 *   * 结论只在这里合成，其他节点不要各自判断“整车是否健康”。
 */

#include <algorithm>
#include <chrono>
#include <sstream>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces_core/msg/chassis_state.hpp"
#include "robot_interfaces_core/msg/robot_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

#include "robot_utils/config/topic_names.hpp"

namespace topics = robot_utils::topics;

using namespace std::chrono_literals;

namespace {

/**
 * @brief 构造一个诊断键值对（DiagnosticStatus.values 的元素）。
 * @param key 键名（诊断面板显示）
 * @param value 值（已转成字符串）
 * @return KeyValue 消息
 */
diagnostic_msgs::msg::KeyValue MakeKeyValue(const std::string& key, const std::string& value) {
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

/**
 * @brief 布尔值转诊断面板可读字符串。
 * @param value 布尔值
 * @return "true" 或 "false"
 */
std::string BoolString(const bool value) { return value ? "true" : "false"; }

}  // namespace

class SystemMonitorNode final : public rclcpp::Node {
 public:
  /**
   * @brief 构造节点：声明参数、注册 6 个诊断项、建立订阅与发布、启动周期定时器。
   * @details 参数：
   *          stale_timeout_ms          超过该时长未收到数据即判为过期（毫秒，默认 1000）
   *          low_battery_voltage       低于该电压报 WARN（默认 22.0 V，与任务中枢低电阈值一致）
   *          critical_battery_voltage  低于该电压报 ERROR（默认 20.0 V）
   *          publish_period_ms         /system/health 与诊断项的发布周期（毫秒，默认 200，
   *                                    下限 50，避免把 diagnostics 打爆）
   */
  SystemMonitorNode() : Node("system_monitor_node"), diagnostics_(this) {
    stale_timeout_ms_ = declare_parameter<int>("stale_timeout_ms", 1000);
    low_battery_voltage_ = declare_parameter<double>("low_battery_voltage", 22.0);
    critical_battery_voltage_ = declare_parameter<double>("critical_battery_voltage", 20.0);
    publish_period_ms_ = declare_parameter<int>("publish_period_ms", 200);

    diagnostics_.setHardwareID("robot");
    diagnostics_.setPeriod(static_cast<double>(std::max(50, publish_period_ms_)) / 1000.0);
    diagnostics_.add("system/chassis", [this](diagnostic_updater::DiagnosticStatusWrapper& stat) {
      FillDiagnosticStatus(CheckChassis(), stat);
    });
    diagnostics_.add("system/battery", [this](diagnostic_updater::DiagnosticStatusWrapper& stat) {
      FillDiagnosticStatus(CheckBattery(), stat);
    });
    diagnostics_.add(
        "system/emergency_stop", [this](diagnostic_updater::DiagnosticStatusWrapper& stat) {
          FillDiagnosticStatus(CheckEmergencyStop(), stat);
        });
    diagnostics_.add("system/scan", [this](diagnostic_updater::DiagnosticStatusWrapper& stat) {
      FillDiagnosticStatus(CheckTopicFreshness("system/scan", scan_seen_, scan_stamp_), stat);
    });
    diagnostics_.add("system/imu", [this](diagnostic_updater::DiagnosticStatusWrapper& stat) {
      FillDiagnosticStatus(CheckTopicFreshness("system/imu", imu_seen_, imu_stamp_), stat);
    });
    diagnostics_.add("system/odom", [this](diagnostic_updater::DiagnosticStatusWrapper& stat) {
      FillDiagnosticStatus(CheckTopicFreshness("system/odom", odom_seen_, odom_stamp_), stat);
    });
    health_publisher_ = create_publisher<robot_interfaces_core::msg::RobotState>(topics::kSystemHealth, 10);

    chassis_subscription_ = create_subscription<robot_interfaces_core::msg::ChassisState>(
        topics::kChassisState, 10, [this](robot_interfaces_core::msg::ChassisState::SharedPtr msg) {
          chassis_state_ = *msg;
          chassis_seen_ = true;
          chassis_stamp_ = now();
        });
    emergency_subscription_ = create_subscription<std_msgs::msg::Bool>(
        topics::kSystemEmergencyStop, 10, [this](std_msgs::msg::Bool::SharedPtr msg) {
          emergency_stop_ = msg->data;
          emergency_seen_ = true;
          emergency_stamp_ = now();
        });
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        topics::kScan, 10, [this](sensor_msgs::msg::LaserScan::SharedPtr) {
          scan_seen_ = true;
          scan_stamp_ = now();
        });
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        topics::kImu, 10, [this](sensor_msgs::msg::Imu::SharedPtr) {
          imu_seen_ = true;
          imu_stamp_ = now();
        });
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        topics::kOdom, 10, [this](nav_msgs::msg::Odometry::SharedPtr) {
          odom_seen_ = true;
          odom_stamp_ = now();
        });

    const auto period = std::chrono::milliseconds(std::max(50, publish_period_ms_));
    timer_ = create_wall_timer(period, [this]() { PublishHealth(); });
  }

 private:
  /**
   * @brief 判断某路数据是否“新鲜”。
   * @param seen 是否至少收到过一次
   * @param stamp 最后一次收到的时刻
   * @return 收到过且距今未超过 stale_timeout_ms_ 时返回 true
   * @note 从未收到也返回 false（启动阶段会把相关项标为 STALE/ERROR，属预期行为）。
   */
  bool IsFresh(const bool seen, const rclcpp::Time& stamp) const {
    if (!seen) {
      return false;
    }
    return ((now() - stamp).nanoseconds() / 1000000) <= stale_timeout_ms_;
  }

  /**
   * @brief 组装一条诊断状态。
   * @param name 诊断项名称（必须与 diagnostic_aggregator.yaml 的 startswith 匹配）
   * @param level OK / WARN / ERROR / STALE
   * @param message 一句话结论（显示在诊断面板）
   * @param values 附加键值对（电压、backend 等，便于排障）
   * @return DiagnosticStatus 消息，hardware_id 固定为 "robot"
   */
  diagnostic_msgs::msg::DiagnosticStatus MakeStatus(
      const std::string& name, const uint8_t level, const std::string& message,
      const std::vector<diagnostic_msgs::msg::KeyValue>& values = {}) const {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = name;
    status.hardware_id = "robot";
    status.level = level;
    status.message = message;
    status.values = values;
    return status;
  }

  /**
   * @brief 检查底盘链路。
   * @return 超时 → ERROR（chassis state timeout）；connected=false → ERROR（disconnected）；
   *         否则 OK，并附 backend / connected / status 三个键值
   * @details 超时与“连上但报错”是两类问题，message 里可区分，便于运维定位。
   */
  diagnostic_msgs::msg::DiagnosticStatus CheckChassis() const {
    if (!IsFresh(chassis_seen_, chassis_stamp_)) {
      return MakeStatus(
          "system/chassis", diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          "chassis state timeout");
    }
    const auto level = chassis_state_.connected ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                                : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    return MakeStatus(
        "system/chassis", level, chassis_state_.connected ? "connected" : "disconnected",
        {
            MakeKeyValue("backend", chassis_state_.backend),
            MakeKeyValue("connected", BoolString(chassis_state_.connected)),
            MakeKeyValue("status", chassis_state_.status),
        });
  }

  /**
   * @brief 检查电池电压。
   * @return 底盘数据超时 → STALE（battery unavailable）；
   *         ≤ critical_battery_voltage_ → ERROR；≤ low_battery_voltage_ → WARN；否则 OK
   * @note 阈值语义：WARN 表示“该回桩/找机会充电”，ERROR 表示“必须立即停”。
   *       若底盘不上报电压（值恒为 0），会一直判 ERROR，接入新底盘时需同步确认该项。
   */
  diagnostic_msgs::msg::DiagnosticStatus CheckBattery() const {
    if (!IsFresh(chassis_seen_, chassis_stamp_)) {
      return MakeStatus(
          "system/battery", diagnostic_msgs::msg::DiagnosticStatus::STALE,
          "battery unavailable");
    }
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string message = "battery normal";
    if (chassis_state_.battery_voltage <= critical_battery_voltage_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      message = "battery critical";
    } else if (chassis_state_.battery_voltage <= low_battery_voltage_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      message = "battery low";
    }
    return MakeStatus(
        "system/battery", level, message,
        {MakeKeyValue("voltage", std::to_string(chassis_state_.battery_voltage))});
  }

  /**
   * @brief 检查急停状态。
   * @return 急停状态超时 → STALE；急停激活 → WARN（可用但受限）；否则 OK
   * @note 急停激活本身不是故障，所以用 WARN；STALE 才说明“谁在管急停”这件事失联了。
   */
  diagnostic_msgs::msg::DiagnosticStatus CheckEmergencyStop() const {
    if (!IsFresh(emergency_seen_, emergency_stamp_)) {
      return MakeStatus(
          "system/emergency_stop", diagnostic_msgs::msg::DiagnosticStatus::STALE,
          "emergency stop state timeout");
    }
    const auto level = emergency_stop_ ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                                       : diagnostic_msgs::msg::DiagnosticStatus::OK;
    return MakeStatus(
        "system/emergency_stop", level, emergency_stop_ ? "emergency stop active" : "clear",
        {MakeKeyValue("active", BoolString(emergency_stop_))});
  }

  /**
   * @brief 通用话题新鲜度检查。
   * @param name 诊断项名称（同时作为诊断名与判据标识）
   * @param seen 是否收到过
   * @param stamp 最后一次收到时刻
   * @return 新鲜 → OK（fresh）；超时/未收到 → STALE（stale）
   * @details 用 STALE 而不是 ERROR：数据断流通常由上游节点未启动或重启引起，
   *          与“硬件报错”区分开，诊断面板上一眼能看出来。
   */
  diagnostic_msgs::msg::DiagnosticStatus CheckTopicFreshness(
      const std::string& name, const bool seen, const rclcpp::Time& stamp) const {
    const bool fresh = IsFresh(seen, stamp);
    return MakeStatus(
        name, fresh ? diagnostic_msgs::msg::DiagnosticStatus::OK
                    : diagnostic_msgs::msg::DiagnosticStatus::STALE,
        fresh ? "fresh" : "stale");
  }

  /**
   * @brief 一次性算出全部 6 项诊断状态。
   * @return 顺序固定为 chassis、battery、emergency_stop、scan、imu、odom
   * @note 顺序只影响诊断面板显示，不影响结论合成（结论取最坏项）。
   */
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> BuildStatuses() const {
    return {
        CheckChassis(),
        CheckBattery(),
        CheckEmergencyStop(),
        CheckTopicFreshness("system/scan", scan_seen_, scan_stamp_),
        CheckTopicFreshness("system/imu", imu_seen_, imu_stamp_),
        CheckTopicFreshness("system/odom", odom_seen_, odom_stamp_),
    };
  }

  /**
   * @brief 把已算好的状态写进 diagnostic_updater 的包装器。
   * @param status 上面 Check* 系列函数的结果
   * @param wrapper diagnostic_updater 回调提供的输出对象
   * @details 只做“拷贝 + summary”，不在回调里做任何计算，保证诊断发布周期稳定。
   */
  void FillDiagnosticStatus(
      const diagnostic_msgs::msg::DiagnosticStatus& status,
      diagnostic_updater::DiagnosticStatusWrapper& wrapper) const {
    wrapper.summary(status.level, status.message);
    for (const auto& value : status.values) {
      wrapper.add(value.key, value.value);
    }
  }

  /**
   * @brief 合成并发布 /system/health，同时触发一次诊断更新。
   * @details 结论规则（取最坏级别）：
   *            OK                 → state=OK，message="all monitored systems nominal"
   *            WARN               → state=WARN，message=各异常项摘要
   *            ERROR / STALE 更高 → state=ERROR，message=各异常项摘要
   *          recoverable 的含义是“能否自动恢复”：不等于 ERROR 时为 true。
   *
   * @note 已知语义细节：DiagnosticStatus 的级别枚举中 STALE(3) 大于 ERROR(2)，
   *       因此“只有数据断流（STALE）”时 state 也会是 ERROR，但 recoverable=true
   *       （上游节点恢复发布即自动恢复）。这是刻意保留的行为，改动前请同步
   *       fault_supervisor 的判定逻辑。
   */
  void PublishHealth() {
    const auto statuses = BuildStatuses();

    uint8_t worst_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> issues;
    for (const auto& status : statuses) {
      worst_level = std::max(worst_level, status.level);
      if (status.level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
        issues.push_back(status.name + "=" + status.message);
      }
    }

    robot_interfaces_core::msg::RobotState health;
    health.header.stamp = now();
    health.previous_state = previous_state_;
    health.active_goal_id = "";
    health.recoverable = worst_level != diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    if (worst_level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
      health.state = "OK";
      health.message = "all monitored systems nominal";
    } else if (worst_level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
      health.state = "WARN";
      health.message = JoinIssues(issues);
    } else {
      health.state = "ERROR";
      health.message = JoinIssues(issues);
    }
    previous_state_ = health.state;

    health_publisher_->publish(health);
    diagnostics_.force_update();
  }

  /**
   * @brief 把多个异常项拼成一行可读文本。
   * @param issues 形如 "system/scan=stale" 的字符串列表
   * @return 以 "; " 分隔的文本
   */
  std::string JoinIssues(const std::vector<std::string>& issues) const {
    std::ostringstream out;
    for (std::size_t i = 0; i < issues.size(); ++i) {
      if (i > 0) {
        out << "; ";
      }
      out << issues[i];
    }
    return out.str();
  }

  // ---- 参数 ----
  int stale_timeout_ms_ = 1000;
  double low_battery_voltage_ = 22.0;
  double critical_battery_voltage_ = 20.0;
  int publish_period_ms_ = 200;
  // ---- 各路数据的“是否收到 / 最后收到时刻” ----
  bool chassis_seen_ = false;
  bool emergency_seen_ = false;
  bool scan_seen_ = false;
  bool imu_seen_ = false;
  bool odom_seen_ = false;
  bool emergency_stop_ = false;
  std::string previous_state_ = "UNKNOWN";
  robot_interfaces_core::msg::ChassisState chassis_state_;
  rclcpp::Time chassis_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time emergency_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time scan_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time imu_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};
  // ---- 诊断与发布 ----
  diagnostic_updater::Updater diagnostics_;
  rclcpp::Publisher<robot_interfaces_core::msg::RobotState>::SharedPtr health_publisher_;
  rclcpp::Subscription<robot_interfaces_core::msg::ChassisState>::SharedPtr chassis_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
