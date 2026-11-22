# @file PayloadState.msg
# @brief 载荷状态：是否载货、重量、顶部模块类型与最近动作，发布在 /facility/payload_state。
# @details 所属包：robot_interfaces_facility
#
std_msgs/Header header  # 状态时间戳
bool loaded  # 当前是否载有货物
string payload_id  # 载荷（货物/容器）标识；空载时为空
float64 weight_kg  # 当前载荷重量，单位 kg
float64 capacity_kg  # 允许的最大载荷重量，单位 kg
string top_module_type  # 顶部模块类型：manual_load（人工上料）/ lift（顶升）
string state  # 载荷状态：EMPTY / LOADED / TOP_MODULE_ACTION
string last_action  # 最近一次载荷动作描述（如 load / unload / lift）
string message  # 状态说明或最近一次动作结果
