# LocalizationHealth.msg
# 描述机器人定位系统的健康状态。

# 消息头：包含时间戳和参考坐标系等元信息
std_msgs/Header header

# 定位系统是否处于已定位状态
bool localized

# 定位系统的健康状态描述信息，例如“正常运行”、“故障”等
string state

# XY 平面位置估计的不确定性指标
float64 covariance_xy

# 判断位置不确定性是否可接受的阈值
float64 covariance_threshold

# 当前关联的站点或定位参考点 ID
string station_id

# 对当前定位状态的补充说明
string message
