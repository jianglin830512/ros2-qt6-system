# 硬件系统级状态：包含控制权、急停及总告警
std_msgs/Header header

# --- 控制权限 (Control Authority) ---
# true:  远方 (Remote) - 由 ROS2 / 上位机程序下发指令
# false: 本地 (Local)  - 由现场物理操作台、手柄或 PLC 触摸屏直接控制
bool is_remote

# --- 安全状态 (Safety Status) ---
# true:  急停状态已触发 (Emergency Stop Active)
# false: 正常运行
bool emergency_stop_on
