# TrackingError.msg  机器人沿规划路径运动时，实际运动与期望路径之间的偏差

# 消息时间戳及参考坐标系等元信息
std_msgs/Header header

# 横向误差：机器人相对目标路径横向偏离了多少，单位为米（m）
float64 lateral_error

# 纵向误差：机器人相对目标路径纵向偏离了多少，单位为米（m）
float64 heading_error

# 期望速度，即控制器希望机器人达到的速度
float64 target_speed

# 实际速度，即机器人当前的实际速度
float64 actual_speed

# 与机器人当前位置 最近的路径点 在路径数组中的索引
int32 nearest_path_index

# 当前使用的路径跟踪控制器名称
string controller_name