float64 current   # 电流值
bool breaker_closed_switch_ack # 合闸确认
bool breaker_opened_switch_ack # 分闸确认
bool over_current_on  # 过流报警

# 每个支路独立的控制模式反馈
uint8 PLC_MODE_MANUAL = 1
uint8 PLC_MODE_AUTO_CURRENT = 2

uint8 plc_control_mode