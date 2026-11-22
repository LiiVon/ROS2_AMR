# @file ChassisState.msg
# @brief 底盘状态：链路连接、运动学模型、速度与电池等实时快照，由底盘驱动节点发布。
# @details 所属包：robot_interfaces_core
#
std_msgs/Header header  # 时间戳与坐标系（frame_id 一般为 base_footprint 或 odom）
string backend  # 当前通信后端：mock / serial / udp
string kinematics_model  # 当前运动学模型：diff_drive / mecanum
bool connected  # 底盘链路是否在线（含看门狗判定）
float64 battery_voltage  # 电池电压，单位 V；无电池数据时为 0
float64 linear_velocity  # 机体前进线速度，单位 m/s（取 x 分量）
float64 angular_velocity  # 机体角速度，单位 rad/s（绕 z 轴）
string status  # 状态摘要字符串，格式：<硬件状态>:<控制模式>:wheels_rpm=<四轮转速>；链路断开时为 backend_unavailable:<控制模式>
