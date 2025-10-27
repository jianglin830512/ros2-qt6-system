# 调压器命令

# 目标调压器的ID
uint8 regulator_id

# 命令常量定义
uint8 CMD_BREAKER_CLOSE = 1 # 合闸
uint8 CMD_BREAKER_OPEN = 2  # 分闸
uint8 CMD_VOLTAGE_UP = 3    # 升压
uint8 CMD_VOLTAGE_DOWN = 4  # 降压

# 要执行的命令
uint8 command