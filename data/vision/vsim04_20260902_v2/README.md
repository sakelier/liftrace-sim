# V-SIM-04 视觉交付导入（2026-09-02 v2）

状态：**ARCHIVED / NOT ACTIVE FOR KS2A543**。该数据由D435i `640×480`、
`horizontal_fov=1.211 rad`夹具生成，不得与当前KS2A543 `1280×720`、
`horizontal_fov=1.4459345 rad`基线混用。

仿真查表仅使用 `B_full100_seed11`；未混入历史 formal23、sparse30 或重复运行。

- `condition_success_rates.csv`：B100 的 5 类 × 5 高度 × 4 速度精确查表；
- `operating_surface_trials.csv`：B100 原始逐试验汇总表；
- `lateral_trials.csv`：C25 post-fix 横向实验逐试验表；
- `motion_trials.csv`：D16 supported 运动实验逐试验表；
- `provenance.json`：归档哈希、源版本、批次与解释边界。

原始交付包保留在工作区根目录 `liftrace_vision_to_navigation_handoff_20260902_v2.zip`，不在数据目录内重复存储。

注意：这些结果均为单 seed；`p_selected` 不是导航 `P_interrupt`；当前
`config/baseline.yaml` 已停用该表。
