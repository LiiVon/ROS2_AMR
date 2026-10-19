# TaskStatus.msg
# 当前任务的执行进度和状态


std_msgs/Header header

# 当前任务状态
string state

# 当前正在处理的目标索引
int32 current_goal_index

# 任务包含的目标总数
int32 total_goals

# 已成功完成的目标数量
int32 succeeded_goals

# 已失败的目标数量
int32 failed_goals

# 任务状态的补充说明
string message

# 任务当前是否处于暂停状态
bool paused
