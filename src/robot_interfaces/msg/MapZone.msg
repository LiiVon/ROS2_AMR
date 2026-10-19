# MapZone.msg
# 描述机器人在地图上的区域信息。

# 区域唯一标识
string zone_id

# 所属地图的名称
string map_name

# 区域类型
string type

# 区域坐标所使用的参考坐标系
string frame_id

# 该区域规则是否启用
bool enabled

# 区域内允许的最高速度，单位 m/s
float64 speed_limit_mps

# 多边形各顶点的 X 坐标数组
float64[] polygon_x

# 多边形各顶点的 Y 坐标数组
float64[] polygon_y

# 区域对应的规则或策略描述
string rule
