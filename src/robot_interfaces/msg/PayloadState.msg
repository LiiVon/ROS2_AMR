# PayloadState.msg
# 载荷状态，
# 也就是机器人有没有携带货物、货物有多重、载荷能力是否足够，以及顶部搭载模块的状态

std_msgs/Header header

# 当前是否装载了货物
bool loaded

# 当前货物或载荷的唯一标识
string payload_id

# 当前载荷重量，单位 kg
float64 weight_kg

# 允许承载的最大重量，单位 kg
float64 capacity_kg

# 机器人顶部搭载的模块类型
string top_module_type

# 当前载荷系统的状态
string state

# 最近一次载荷相关操作
string last_action

# 状态补充说明或错误信息
string message
