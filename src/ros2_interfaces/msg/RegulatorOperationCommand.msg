# 调压器操作命令 (升/降/停)

# 目标调压器的ID
uint8 regulator_id

# --- 命令常量定义 ---
uint8 CMD_VOLTAGE_UP = 1    # 升压
uint8 CMD_VOLTAGE_DOWN = 2  # 降压
uint8 CMD_VOLTAGE_STOP = 3  # 停止升压或降压

# 要执行的命令
uint8 command