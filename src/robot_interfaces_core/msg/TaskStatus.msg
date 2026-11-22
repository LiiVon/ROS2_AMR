# @file TaskStatus.msg
# @brief 多目标导航任务状态：当前第几个目标、成功/失败计数与暂停标志，发布在 /navigation/task_status。
# @details 所属包：robot_interfaces_core
#
std_msgs/Header header  # 状态时间戳
string state  # 任务状态：idle / navigating / paused
int32 current_goal_index  # 当前执行到第几个目标（从 0 开始）
int32 total_goals  # 本次任务的目标总数
int32 succeeded_goals  # 已成功到达的目标数
int32 failed_goals  # 已失败/跳过的目标数
string message  # 状态说明；失败时为失败原因
bool paused  # 任务是否处于暂停状态
