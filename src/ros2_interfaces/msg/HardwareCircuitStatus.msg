# 包含时间戳和坐标系信息，这是良好实践
std_msgs/Header header

# --- 回路编号 ---
uint8 circuit_id  # 编号

# --- 两个回路的状态 ---
HardwareLoopStatus test_loop   # 试验回路的状态
HardwareLoopStatus ref_loop    # 参考回路的状态

