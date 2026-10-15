# NavigationStatistics.msg 导航任务统计数据

# 消息时间戳及参考坐标系等元信息
std_msgs/Header header

# 累计导航任务的总数
int32 total_goals 

# 成功完成的导航任务数量
int32 succeeded_goals

# 失败的导航任务数量
int32 failed_goals

# 导航成功率
float64 success_rate

# 累计导航距离,单位为米（m）
float64 total_distance_m

# 最近一次导航任务的耗时，单位为秒（s）
float64 last_navigation_time_s

# 平均导航任务耗时，单位为秒（s）
float64 mean_navigation_time_s

# 导航过程中触发恢复行为的累计次数
int32 recovery_count