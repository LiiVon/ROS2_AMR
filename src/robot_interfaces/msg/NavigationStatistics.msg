# NavigationStatistics.msg

std_msgs/Header header
int32 total_goals
int32 succeeded_goals
int32 failed_goals
float64 success_rate
float64 total_distance_m

# 最近一次导航任务的耗时，单位秒
float64 last_navigation_time_s

# 平均每次导航任务的耗时
float64 mean_navigation_time_s

# 导航过程中触发恢复行为的次数
int32 recovery_count
