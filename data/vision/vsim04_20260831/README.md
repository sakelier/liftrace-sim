# V-SIM-04视觉性能派生数据

本目录由根目录交付包`liftrace_nav_to_vision_handoff_20260831.zip`生成，原始包不在这里重复展开。

- `condition_success_rates.csv`：按`kind × class × height × speed`聚合的`p_selected`；
- `provenance.json`：源压缩包SHA-256、样本数和适用限制；
- 重新生成：`uv run python tools/import_vision_handoff.py`。

动态边界条件同时聚合`sparse30_seed11`的一次观测与三次同种子运行重复；其余动态条件通常只有一次观测。静态数据不用于飞行航段查表。所有结果均为固定`seed=11`证据，不能解释为已经收敛的总体概率，也不能替代尚为空的`P_interrupt`。

仿真默认只在类别、高度、速度均精确命中动态实测条件时采用此表；未测条件回退到`baseline.yaml`中明确标为`ASSUMED`的旧模型。
