uint8 circuit_id  # 1 for circuit 1, 2 for circuit 2
# 完整回路的参数设置
LoopSettings test_loop
LoopSettings ref_loop
Cable sample_cable       # 这里替换了以前的 SampleSettings sample_params
uint8 test_id            # 试验ID，记录了当前试验的温度测量对应位置以及其它重要信息