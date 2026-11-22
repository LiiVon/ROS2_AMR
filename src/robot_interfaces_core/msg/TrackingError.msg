# @file TrackingError.msg
# @brief 路径跟踪误差：横向/航向偏差与速度跟踪情况，由跟踪控制器发布在 /tracking/error。
# @details 所属包：robot_interfaces_core
#
std_msgs/Header header  # 采样时间戳
float64 lateral_error  # 横向误差，单位 m（机器人到参考路径的有符号距离，左正右负）
float64 heading_error  # 航向误差，单位 rad（车头方向与路径切向之差）
float64 target_speed  # 期望线速度，单位 m/s
float64 actual_speed  # 实际线速度，单位 m/s
int32 nearest_path_index  # 参考路径上最近点的序号（从 0 开始）
string controller_name  # 产生该误差的控制器：pure_pursuit / stanley
