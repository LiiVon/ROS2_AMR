# @file MapZone.msg
# @brief 地图区域定义：多边形 + 语义类型（禁行/限速/推荐通道）与速度上限。
# @details 所属包：robot_interfaces_navigation
#
string zone_id  # 区域唯一标识（同一地图内唯一）
string map_name  # 所属地图名称
string type  # 区域类型：keepout（禁行）/ speed_limit（限速）/ preferred_lane（推荐通道）
string frame_id  # 多边形所在坐标系，一般为 map；当前实现仅校验非空，不做坐标变换
bool enabled  # 是否启用；false 时导航忽略该区域
float64 speed_limit_mps  # 区域限速，单位 m/s；仅 speed_limit 类型且必须 > 0 才生效
float64[] polygon_x  # 多边形顶点 x 坐标数组，单位 m；与 polygon_y 一一对应，至少 3 个点
float64[] polygon_y  # 多边形顶点 y 坐标数组，单位 m；与 polygon_x 等长
string rule  # 规则说明文本，仅用于人工阅读/审计，不参与计算
