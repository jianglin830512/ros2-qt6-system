# 产生记录的日期和时间
string record_time
# 当前回路 ID
uint8 circuit_id
# 是否处于自动模式，根据system settings判断
bool auto_on

# 调压器1管测试支路
bool regulator_1_breaker_closed
float64 regulator_1_voltage
float64 regulator_1_current
# 测试支路状态和读值
bool test_loop_is_heat
bool test_loop_breaker_closed
uint8 test_loop_strategy
float64 test_loop_current
float64[16] test_loop_temp

# 调压器2管参考支路
bool regulator_2_breaker_closed
float64 regulator_2_voltage
float64 regulator_2_current
# 参考支路状态和读值
bool ref_loop_is_heat
bool ref_loop_breaker_closed
uint8 ref_loop_strategy
float64 ref_loop_current
float64[16] ref_loop_temp