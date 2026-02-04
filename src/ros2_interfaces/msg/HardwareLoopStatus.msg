float64 current   # 电流值
float64[16] temperature_array  # 温度值 (注意：两个回路的温度数组大小不同)
bool breaker_closed_switch_ack # 合闸确认
bool breaker_opened_switch_ack # 分闸确认
bool over_current_on  # 过流报警