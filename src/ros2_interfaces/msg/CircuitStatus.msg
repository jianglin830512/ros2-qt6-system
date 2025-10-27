# 包含时间戳和坐标系信息，这是良好实践
std_msgs/Header header

# --- 回路编号 ---
uint8 circuit_id  # 编号

# --- 回路读数 (连续值) ---
float64 test_current   # 试验回路电流值
float64 ref_current    # 参考回路电流值
float64[16] test_temperature_array  # 试验回路温度值
float64[8] ref_temperature_array    # 参考回路温度值
builtin_interfaces/Duration elapsed_heating_time    # 已经加热时间，单位：分钟
builtin_interfaces/Duration remaining_heating_time  # 剩余加热时间，单位：分钟
uint16 completed_cycle_count   # 已经循环次数
uint16 remaining_cycle_count   # 剩余循环次数


# --- 回路状态（常量定义） ---
# 控制模式
uint8 MAN_MODE = 0   # 手动
uint8 CURR_MODE = 1  # 恒流
uint8 TEMP_MODE = 2  # 温控

# --- 回路状态 (离散值) ---
uint8 control_mode # 控制模式 (使用上面的常量)
bool circuit_enabled  # 启动状态
bool breaker_closed_switch_ack # 合闸确认
bool breaker_opened_switch_ack # 分闸确认
