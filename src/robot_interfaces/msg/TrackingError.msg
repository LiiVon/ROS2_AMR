# TrackingError.msg
# 描述机器人在导航过程中与预期路径的偏差信息。

std_msgs/Header header

# 横向误差，即机器人相对目标路径的横向偏离量
float64 lateral_error

# 航向误差，即机器人朝向与目标路径方向的角度差
float64 heading_error

# 控制器期望机器人达到的目标速度
float64 target_speed

# 机器人当前的实际速度
float64 actual_speed

# 当前机器人对应的最近路径点索引
int32 nearest_path_index

# 当前使用的路径跟踪控制器名称
string controller_name
