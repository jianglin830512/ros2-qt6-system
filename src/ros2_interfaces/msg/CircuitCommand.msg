# 回路控制命令

# 目标回路的ID
uint8 circuit_id

# 命令常量定义
uint8 CMD_TEST_BREAKER_CLOSE = 1  # 试验回路合闸
uint8 CMD_TEST_BREAKER_OPEN = 2   # 试验回路分闸
uint8 CMD_SIM_BREAKER_CLOSE = 3   # 模拟回路合闸
uint8 CMD_SIM_BREAKER_OPEN = 4    # 模拟回路分闸
uint8 CMD_SET_MANUAL_MODE = 5     # 切换为手动模式
uint8 CMD_SET_CONST_CURRENT_MODE = 6 # 切换为恒流模式
uint8 CMD_SET_TEMP_CONTROL_MODE = 7  # 切换为温控模式

# 要执行的命令
uint8 command