# RobotState.msg
# 描述机器人当前的状态信息。

std_msgs/Header header

# 机器人当前的状态，例如“空闲”、“执行任务中”、“故障”等
string state

# 机器人 上一步的状态，例如“空闲”、“执行任务中”、“故障”等
string previous_state

# 当前正在执行的目标或任务 ID
string active_goal_id

# 对当前状态的补充说明
string message

# 当前异常或状态是否允许通过恢复流程处理
bool recoverable
