# liftrace-sim

R64最新：固定seed11完整PASS，随后十seed原始7/10完整PASS；5/7/8有靶板压墙，原始数据保留并标注。新增11轮姿态轨迹/相机几何诊断、严格0.80m左右门和独立树箱随机导出；随机门SITL未运行。[报告与工具](docs/R64_BASELINE_AND_RANDOMIZATION.md)。

最新：[整机问题、真实航向与轻量模型合理性复核](docs/ENGINEERING_REVIEW_20260909.md)。新增完整刚性相机姿态投影、日志航向分析和seed11姿态敏感性；57项测试通过。旧M0/M2的2.4m/随航迹转头/3.4m升高转运属于历史研究假设，不能用于当前比赛的水平绕障验收。

2026-09-09新增[R60实际布设与R61新场地的策略预筛](data/recorded_fields/SEARCH_COMPARISON.md)，使用实测16cm外参、固定机头与9.6m场地。入口`recorded_search_replay.py`独立于下文历史M0/M2默认参数；不把旧2.4m研究基线当作当前整机参数。

面向无人机搜索策略研究的纯Python任务级仿真，不是ROS或catkin工作区。

建模边界和后续阶段以[MODELING_OUTLINE.md](MODELING_OUTLINE.md)为准。当前已完成：

```text
M0：牛耕航迹几何与名义计时校核
M1a：确定性下视相机视场、航段横向偏差和可见驻留时间
M1b：受约束随机目标、航段级Cue概率与可复现Monte Carlo
M1b-V：已导入KS2A543 V-SIM-04 B100；精确格点按单seed诊断值驱动，格外显式回退
M1c：矩形障碍物、安全膨胀、航段碰撞、视线遮挡与栅格A*绕行
M1d：沿实际轨迹按帧采样的目标五点遮挡门控与有效连续可见时间
M2：Cue中断、升高转运、接近复核、投递、返回断点与超时状态机
M2-S：正式规则计分、3件载荷容量、投递权重、600秒限时与同分计时规则
M2-D：有限载荷在线决策与公共随机场景策略对比
```

## 当前文件

```text
config/baseline.yaml       当前M0/M1参数及证据标签
ASSUMPTIONS.md             临时假设、范围和替换数据
search_sim.py              航迹、可见区间、Cue事件与Monte Carlo
tests/test_search_sim.py   M0/M1人工可复算、边界和复现性测试
config/sweep.yaml          高度、速度、航带间距扫描范围
sweep_sim.py               公共随机场景网格扫描与Pareto前沿
plot_route.py              二维航线、航向和随机目标SVG图
plot_vision_heatmap.py     分类别高度—速度实测性能热力图
plot_pareto.py             搜索时间—Cue率及Pareto前沿散点图
plot_dynamic_threshold_heatmap.py  动态阈值分数与时间二维热力图
obstacles.py               障碍碰撞与相机—目标视线遮挡
occlusion.py               遮挡采样、可见比例和连续可见窗口
mission_sim.py             带中断任务链与Monte Carlo汇总
scoring.py                 正式计分常量、逐件投递计分与完整性标记
delivery_policy.py         先见先投、固定优先、红十字预留和动态阈值策略
compare_delivery_policies.py  同场景策略对比与配对差值统计
sweep_delivery_threshold.py   动态阈值二维调参与独立seed验证
vision_performance.py      V-SIM-04实测条件读取与未测策略
tools/import_vision_handoff.py  从视觉组交付包重建派生表
data/vision/vsim04_20260902_v2/  已停用的D435i B100/C25/D16归档，仅供历史对照
data/vision/vsim04_20260904_ks2a543/  当前KS2A543 A/B/C/D v3派生表与来源
```

## 运行

```bash
cd /home/xhj/liftrace-sim
uv run python search_sim.py
```

保存结构化结果和航点：

```bash
uv run python search_sim.py \
  --output-json results/m1b_baseline.json \
  --waypoints-csv results/m1b_waypoints.csv

# 临时覆盖试验次数与种子
uv run python search_sim.py --monte-carlo 5000 --seed 42

# 只看确定性的M0/M1a几何报告
uv run python search_sim.py --no-monte-carlo
```

扫描高度、速度和航带间距：

```bash
uv run python sweep_sim.py

# 快速冒烟实验，覆盖每个网格点的Monte Carlo次数
uv run python sweep_sim.py --trials 50 --seed 42
```

生成二维航线图：

```bash
uv run python plot_route.py
# 固定另一套目标布置，或只画航线
uv run python plot_route.py --seed 42
uv run python plot_route.py --no-targets
```

默认输出为`results/route_2d.svg`，可直接在浏览器或VS Code中打开。
图中红色实心矩形为障碍物，浅红虚线范围为安全膨胀区，红色航段表示原始直线牛耕路线不可行，青绿色折线为经过直视简化的A*可行路线。

生成视觉性能热力图：

```bash
uv run python plot_vision_heatmap.py
```

默认输出为`results/vision_performance_heatmap.svg`。当前默认数据是KS2A543 B100；每格只有
seed11的一次0/1结果，灰色格代表未测条件，不进行插值。

生成搜索时间—Cue率Pareto散点图：

```bash
uv run python sweep_sim.py
uv run python plot_pareto.py
```

默认输出为`results/pareto_time_cue.svg`；点的颜色表示搜索高度，黑色描边和虚线表示采样网格上的Pareto前沿。
参数扫描会先根据每个候选高度执行A*：低于障碍物安全顶高的组合计入绕行距离和新增转向，高于安全顶高的组合保留直线航段。

遮挡层对每个几何可见航段记录采样帧数、有效帧数、平均可见比例、最长连续可见时间和遮挡障碍物；Monte Carlo汇总输出遮挡门控通过率与平均可见比例。遮挡只决定视觉证据是否足够，不会未经标定地直接乘改视觉概率。

默认结果写入`results/m1b_sweep.csv`和`results/m1b_sweep.json`。所有参数组合复用相同seed派生出的逐试验目标布置，避免把场景随机差异误认为策略差异。当前只报告“总时间更小、平均Cue率更高”的Pareto前沿，不擅自设定二者的加权系数。

如果Pareto点全部落在某个扫描边界，程序会在JSON和终端中报告边界饱和。当前扫描网格与
KS2A543 B100精确对应，但每格仅有一次seed11诊断；边界结果只能用于筛选整机候选，不能解释为
物理最优或真实发现概率。

运行测试：

```bash
uv run python -m unittest discover -s tests -v
```

运行带中断任务链：

```bash
uv run python mission_sim.py --trials 200 --seed 20260831
```

结果写入`results/m2_mission.json`，包含汇总指标、一轮完整状态事件时间线和计分拆分。
当前起飞项按仿真场景输入计分；避障、穿门和自主降落尚未建模，因此明确记为
`unmodelled_items`并计0分。投递成功暂按普通目标最内圈/红十字完全入圈处理，属于
乐观假设，不应把当前`mean_official_score`当作完整比赛分数预测。
计分常量的来源登记为比赛规则`1786409924213228.pdf`第11—13页；配置中的规则值若
与计分模块内已核对常量不一致，程序会直接报错而不是静默计算。

比较有限载荷投递策略：

```bash
uv run python compare_delivery_policies.py --trials 1000 --seed 20260831
```

结果写入`results/delivery_policy_comparison.json`和同名CSV。四种策略共享目标布置、
Cue抽样以及按`target_id`绑定的复核/投递随机数；JSON同时给出逐场景配对分数差、
95%置信区间和胜/平概率。动态阈值使用“当前目标期望投递分是否超过剩余区域中第
`k`高的未来机会价值”进行决策，其中`k`为剩余物资数。未来Cue概率仍是假设参数，
因此当前比较用于验证决策机制，不代表策略已经完成实测标定。

扫描动态阈值参数并在独立seed上验证前5名：

```bash
uv run python sweep_delivery_threshold.py
```

默认扫描`6×6=36`个“统一未来Cue先验×进度衰减指数”组合，每点使用500个调参
场景；前5名再使用2000个不同seed的验证场景，与`first_seen`和
`reserve_red_cross`配对比较。结果写入`results/dynamic_threshold_sweep.json`和CSV。
KS2A543基线下的当前候选是`p=0.70, gamma=0.50`：500次/格调参后，使用独立
seed的2000次验证，相对`reserve_red_cross`平均高`1.595`分（配对95% CI
`[1.425, 1.765]`）。它仍是在含假设的任务模型中得到的策略参数，不是对真实视觉
Cue概率的测量值，也未设为`active_policy`。

生成动态阈值分数—时间热力图：

```bash
uv run python plot_dynamic_threshold_heatmap.py
```

默认输出`results/dynamic_threshold_heatmap.svg`。黑框为进入独立验证的前5名，金框
为最终候选；左右面板的颜色分别归一化，精确比较应读取格内数字。

## 当前输出含义

- `lane_distance_m`：所有长航带长度之和；
- `connector_distance_m`：相邻航带之间的换带距离；
- `route_distance_m`：上述两项之和，不含起飞区接入；
- `flight_time_s`：按恒定搜索速度计算；
- `turn_time_s`：所有非共线转向的固定惩罚；
- `total_time_s`：飞行时间和转向惩罚之和。
- `footprint`：给定相对地面高度下的横向和沿航段理论视场；
- `cross_track_distance_m`：目标中心到航段的最小横向距离；
- `entry/exit_distance_m`：目标进入和离开有效视场时在航段上的位置；
- `visible_path_length_m`：目标中心或完整目标满足视场约束的航程；
- `dwell_time_s`：上述可见航程除以名义飞行速度。

命令行默认加载两个明确标为`TEST_FIXTURE`的目标，只用于展示M1a结果。全部完整入画观测会写入`--output-json`指定的文件。

M1a只描述理论几何可见，不等于YOLO检出、圆环关联、`map_valid`或稳定Cue，不能用于宣称搜索策略最优。

M1b把目标生成与Cue抽样提升到任务级。当前相机基线和V-SIM-04表均为KS2A543；B100的
100个精确高度×速度×类别格点使用视觉`p_selected`，其他条件回退到明示的`ASSUMED`公式。
C25/D11只归档供策略设计参考，尚未硬接进Cue接口。Cue仍是每次有效可见航段上的Bernoulli
事件，不是逐帧YOLO输出；单seed的0/1格点也不是导航实际`P_interrupt`或概率置信区间。
本轮完整扫描的Pareto点为`3.6 m/2.0 m/s`、`2.4 m/1.5 m/s`和
`2.4 m/1.0 m/s`；前者发现率较低且贴近4 m限高，因此基线保持较保守的
`2.4 m/1.0 m/s`。动态阈值新候选虽通过独立seed验证，输入仍包含目标分布、复核/投递
成功率和格外回退等假设；当前只作后续整机对照，不直接替代简单策略。
