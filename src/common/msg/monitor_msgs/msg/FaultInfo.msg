# 注意: ROS 2 推荐使用 builtin_interfaces/Time 而不是 float64
builtin_interfaces/Time timestamp_sec   # 故障发生时间
string module_name                       # 故障模块或节点名称
int32 sequence_num                       # 消息数
int32 version                            # 版本号
int32 error_code                         # 故障码
int32 fault_level                        # 故障等级
string msg                               # 故障描述