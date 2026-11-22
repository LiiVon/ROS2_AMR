# @file SafetyState.msg
# @brief 导航安全状态：障碍阻塞、急停、动态/区域限速及其合成结果，发布在 /navigation/safety_state。
# @details 所属包：robot_interfaces_navigation
#
std_msgs/Header header  # 状态时间戳
bool obstacle_blocked  # 当前是否被障碍阻塞（阻塞时暂停或绕行按实现策略）
bool safety_stop  # 是否处于安全停车状态（当前实现与 obstacle_blocked 同步）
string state  # 安全状态：OK / SPEED_LIMITED / OBSTACLE_BLOCKED
float64 runtime_speed_limit_mps  # 运行时动态限速，单位 m/s；0 表示未设置
float64 zone_speed_limit_mps  # 区域限速，单位 m/s；0 表示无区域限速
float64 effective_speed_limit_mps  # 生效限速（运行时与区域限速取较小值），单位 m/s；0 表示不限速
string stop_reason  # 停车原因文本（如障碍描述）；无停车时为空
string blockage_id  # 当前阻塞事件 ID，用于后续清除阻塞
string mission_id  # 受该安全状态影响的任务 ID；无则为空
string message  # 状态说明文本
