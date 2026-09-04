# V-SIM-04 视觉交付导入（2026-09-04 KS2A543）

状态：**ACTIVE FOR CURRENT KS2A543 / SINGLE-SEED DIAGNOSTIC**。相机为 `1280×720`、`horizontal_fov=1.4459345 rad`；不得与历史D435i 表拼接。

仿真查表仅使用 `B_full100_seed11`；未混入 formal23、sparse30 或旧相机运行。

- `condition_success_rates.csv`：B100 的 5 类 × 5 高度 × 4 速度精确查表；
- `operating_surface_trials.csv`：B100 原始逐试验汇总表；
- `lateral_trials.csv`：KS2A543 C25 横向实验逐试验表；
- `motion_trials.csv`：KS2A543 D11 supported 运动实验逐试验表；
- `provenance.json`：归档哈希、源版本、批次与解释边界。

源交付包 `liftrace_vision_to_navigation_handoff_20260904_v3.zip` 不在数据目录内重复存储；默认导入路径见导入脚本。

注意：这些结果均为单 seed；每格的0/1是一次确定性测量，不是置信区间；
`p_selected` 不是导航 `P_interrupt`。
