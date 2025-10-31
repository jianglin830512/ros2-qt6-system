# 单个回路（试验或参考）的参数
int32 start_current_a
int32 max_current_a
int32 current_change_range_percent # 电流变化范围
int32 ct_ratio # 互感器变比
builtin_interfaces/Time start_datetime # 试验开启日期和时间
builtin_interfaces/Duration heating_duration # 加热时长
int32 cycle_count # 循环次数