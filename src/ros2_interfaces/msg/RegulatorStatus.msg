# 包含时间戳和坐标系信息，这是良好实践
std_msgs/Header header

# --- 调压器编号 ---
uint8 regulator_id # 编号

# --- 调压器读数 (连续值) ---
float64 voltage_reading   # 电压值
float64 current_reading   # 电流值

# --- 调压器状态常量定义 ---
# 调压方向
int8 VOLTAGE_STATIC = 0      # 静止
int8 VOLTAGE_INCREASING = 1  # 正在升压
int8 VOLTAGE_DECREASING = -1 # 正在降压

# --- 调压器状态 (离散值) ---
int8 voltage_direction       # 调压方向 (使用上面的常量)

# 分合闸确认。保留两个独立信号以增强鲁棒性和故障检测
bool breaker_closed_switch_ack # 合闸确认
bool breaker_opened_switch_ack # 分闸确认

bool upper_limit_switch_on     # 上限位
bool lower_limit_switch_on     # 下限位

bool over_current_on  # 过流报警
bool over_voltage_on  # 过压报警