# SafetyState.msg
# 描述机器人当前的安全状态。

std_msgs/Header header

# 是否因障碍物而无法正常通行
bool obstacle_blocked

# 是否触发安全停止
bool safety_stop

# 当前安全状态
string state

# 运行时动态设置的速度上限，单位 m/s
float64 runtime_speed_limit_mps

# 当前所在地图区域规定的速度上限，单位 m/s
float64 zone_speed_limit_mps

# 安全逻辑计算出的最终有效速度上限，单位 m/s
float64 effective_speed_limit_mps

# 停止原因
string stop_reason

# 当前障碍阻塞事件的标识
string blockage_id

# 当前关联的任务 ID
string mission_id

# 安全状态的补充说明
string message
