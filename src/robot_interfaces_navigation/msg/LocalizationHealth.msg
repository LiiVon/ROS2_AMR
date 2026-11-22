# @file LocalizationHealth.msg
# @brief 定位健康度：定位是否可信、协方差与是否已按站点重置，发布在 /navigation/localization_health。
# @details 所属包：robot_interfaces_navigation
#
std_msgs/Header header  # 采样时间戳
bool localized  # 当前是否定位可信
string state  # 定位状态：OK / LOST / RELOCALIZING / RECOVERED
float64 covariance_xy  # 位姿 xy 协方差（m²），越大越不可信
float64 covariance_threshold  # 判定定位失效的协方差阈值（m²）
string station_id  # 最近一次用于重置初始位姿的站点 ID；无则为空
string message  # 状态说明或失败原因
