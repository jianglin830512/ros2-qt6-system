# 单个回路（试验或参考）的参数
HardwareLoopSettings hardware_loop_settings  # 硬件参数
builtin_interfaces/Time start_date # 试验开启日期，其时间部分为0
builtin_interfaces/Duration heating_time # 加热时刻，从00:00:00开始计算
builtin_interfaces/Duration heating_duration # 加热时长
int32 cycle_count # 循环次数
bool enabled # 是否启动该回路