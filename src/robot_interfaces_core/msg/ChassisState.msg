# ChassisState.msg   底盘状态

# 消息头：消息的时间戳和参考坐标系等元信息   
# header 含有两个字段：
#    1. stamp：时间戳，表示消息的生成时间
#    2. frame_id：参考坐标系的名称，通常用于表示消息数据所对应的坐标系
std_msgs/Header header

# 底盘使用的后端实现或通信驱动名称
string backend


# 底盘使用的运动学模型名称
# 常见两种运动学模型：
#    1. 差速驱动（Differential Drive）：两个驱动轮通过不同的转速组合，让机器人直行、转弯或原地旋转。
#    2. 全向驱动（Omni-directional Drive）：利用四个特殊滚轮，可以实现横移、斜移等全向运动
string kinematics_mmodel

# 底盘驱动或底层设备是否处于连接状态
bool connected

# 电池电压，单位为伏特（V）
float64 battery_voltage

# 底盘的线速度，单位为米每秒（m/s）
float64 linear_velocity 

# 底盘的角速度，单位为弧度每秒（rad/s）
float64 angular_velocity

# 底盘的状态描述信息，例如“正常运行”、“故障”等
string status