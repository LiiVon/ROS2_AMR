# TaskStatus.msg  任务状态

# 消息时间戳和参考坐标系等元信息
std_msgs/Header header

# 当前任务的执行状态
string state

# 当前正在处理的目标  在任务列表中 的索引
int32 current_goal_index

# 任务包含的导航目标总数
int32 total_goals

# 已成功完成的目标数量
int32 succeeded_goals

# 已失败的目标数量
int32 failed_goals

# 当前任务的补充说明或提示
string message

# 当前任务是否处于暂停状态
bool paused