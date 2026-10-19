# ChassisState.msg
# 描述机器人底盘当前的运行状态。

# 消息时间戳和坐标系等元信息
std_msgs/Header header

# 当前使用的底盘通信后端
string backend

# 当前使用的底盘运动学模型
string kinematics_model

# 底盘是否连接
bool connected

# 电池电压，单位为伏特（V）
float64 battery_voltage

# 底盘线速度，单位为米每秒（m/s）
float64 linear_velocity

# 底盘角速度，单位为弧度每秒（rad/s）
float64 angular_velocity

# 底盘状态描述信息，例如“正常运行”、“故障”等
string status