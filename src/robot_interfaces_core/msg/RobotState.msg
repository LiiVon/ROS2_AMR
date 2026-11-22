# @file RobotState.msg
# @brief 机器人运行状态：任务运行器状态机、当前目标与恢复标志，发布在 /mission/state。
# @details 所属包：robot_interfaces_core
#
std_msgs/Header header  # 状态时间戳
string state  # 运行器状态字符串：任务中枢使用 IDLE / RUNNING / PAUSED / LOCALIZATION_LOST 等；导航序列服务端使用 idle / navigating / paused
string previous_state  # 上一个状态，用于观察状态迁移
string active_goal_id  # 当前正在执行的导航目标/任务标识；空闲时为空
string message  # 状态说明或最近一次失败原因
bool recoverable  # 当前失败是否可恢复（true 表示可重试/可恢复）
