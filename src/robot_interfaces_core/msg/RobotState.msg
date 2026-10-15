# RobotState.msg 机器人状态情况

# 消息时间戳及参考坐标系等元信息
std_msgs/Header header

# 机器人当前状态
string state

# 机器人上一个状态
string previous_state

# 当前正在处理的导航目标 ID
string active_goal_id

# 对当前状态的补充说明或提示信息
string message

# 当前异常或状态是否允许尝试恢复
bool recoverable