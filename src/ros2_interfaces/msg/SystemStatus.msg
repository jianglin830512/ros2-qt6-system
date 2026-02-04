std_msgs/Header header

# --- 连接状态监控 ---
bool database_connected
bool hardware_connected

# --- 系统生命周期状态定义 ---
uint8 STATE_INITIALIZING = 0  # 初始化中（正在连接依赖或同步数据）
uint8 STATE_NORMAL       = 1  # 正常运行（依赖就绪，数据已同步）
uint8 STATE_ERROR        = 2  # 错误（运行中丢失连接）

# --- 当前系统状态 ---
uint8 system_state