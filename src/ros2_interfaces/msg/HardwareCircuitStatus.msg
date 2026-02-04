# 包含时间戳和坐标系信息，这是良好实践
std_msgs/Header header

# --- 回路编号 ---
uint8 circuit_id  # 编号

# --- 两个回路的状态 ---
HardwareLoopStatus test_loop   # 试验回路的状态
HardwareLoopStatus ref_loop    # 参考回路的状态

# --- 手动/恒流 相关的反馈 ---

# 1. PLC 当前的工作模式反馈
uint8 PLC_MODE_MANUAL = 1
uint8 PLC_MODE_AUTO_CURRENT = 2
uint8 PLC_MODE_AUTO_TEMP = 3

uint8 plc_control_mode  # 当前 PLC 认为自己在什么模式

# 2. PLC 当前的控制源反馈 (仅在自动模式下有效)
uint8 PLC_SOURCE_TEST_LOOP = 1
uint8 PLC_SOURCE_REF_LOOP = 2

uint8 plc_control_source # 当前 PLC 正在用哪个支路的数据做恒流的参考
