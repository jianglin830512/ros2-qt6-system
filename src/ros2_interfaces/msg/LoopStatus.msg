# LoopStatus.msg
# 描述单个回路（试验/参考）的当前状态
bool enabled # 该回路是否已经启动
HardwareLoopStatus hardware_loop_status  # 硬件状态
builtin_interfaces/Duration elapsed_heating_time    # 已经加热时间
builtin_interfaces/Duration remaining_heating_time  # 剩余加热时间
uint16 completed_cycle_count   # 已经循环次数
uint16 remaining_cycle_count   # 剩余循环次数