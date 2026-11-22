# @file NavigationStatistics.msg
# @brief 导航统计：累计目标数、成功/失败数、成功率、里程与耗时，用于运营看板与评测。
# @details 所属包：robot_interfaces_core
#
std_msgs/Header header  # 统计快照时间戳
int32 total_goals  # 累计下发的导航目标总数
int32 succeeded_goals  # 累计成功到达的目标数
int32 failed_goals  # 累计失败的目标数（被取消也计入失败）
float64 success_rate  # 成功率 = succeeded_goals / (succeeded_goals + failed_goals)，范围 0.0~1.0；无已完成目标时为 0
float64 total_distance_m  # 累计导航里程，单位 m
float64 last_navigation_time_s  # 最近一次导航耗时，单位 s
float64 mean_navigation_time_s  # 累计平均耗时 = last_navigation_time_s / succeeded_goals，单位 s
int32 recovery_count  # 触发恢复行为（重规划/清除代价地图等）的累计次数
