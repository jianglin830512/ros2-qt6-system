# 包含时间戳和坐标系信息，这是良好实践
std_msgs/Header header

# --- 回路编号 ---
uint8 circuit_id  # 编号

# --- 两个回路的状态 ---
LoopStatus test_loop   # 试验回路的状态
LoopStatus ref_loop    # 参考回路的状态

# --- 回路状态（常量定义） ---
uint8 MAN_MODE = 0   # 手动
uint8 CURR_MODE = 1  # 恒流
uint8 TEMP_MODE = 2  # 温控

# --- 回路状态 (离散值) ---
uint8 control_mode # 控制模式 (使用上面的常量)
