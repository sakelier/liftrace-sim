# Liftrace无人机搜索策略建模与仿真大纲

- 文档状态：第一版建模依据
- 建立日期：2026-08-29
- 适用项目：`liftrace-sim`
- 结果接收方：`liftrace-controlwork/uav_mission`
- 视觉数据来源：`liftrace-visionwork/uav_vision_eval`

## 1. 文档目的

本文件规定`liftrace-sim`后续数学建模、代码实现、参数标定、Monte Carlo实验和Gazebo复核的统一口径。

本项目不复现PX4姿态控制、电机动力学、FAST-LIO内部估计或Fast-Planner的B样条优化，而是在任务层回答：

> 在有限任务时间和安全约束下，如何联合选择搜索高度、速度、航带间距、航带方向及事件中断策略，使随机未知目标环境中的期望比赛得分和低分鲁棒性最大。

`liftrace-sim`的核心因果链为：

```text
搜索参数与策略
→ 目标进入有效观察区域
→ Cue生成
→ 主动复核
→ 有效投递
→ 任务时间与比赛得分
```

## 2. 权威依据与事实边界

### 2.1 比赛规则

比赛规则以19页正式PDF为准。首版模型至少落实以下约束：

- 场地外尺寸为`10 m × 10 m × 4 m`；
- 单次任务最长`600 s`；
- 无人机携带3件货物；
- 正式投放目标为4个标准靶和1个红十字随机靶；
- 标准类别为`tent、pillbox、bridge、panzer`；
- 红十字类别为`red_cross`；
- 正式收益权重依次为`1.0、1.5、2.0、2.5、10.0`；
- `tank`属于当前视觉模型兼容或仿真遗留类，不进入正式比赛收益；
- 搜索策略必须为过门、返航和自主降落预留时间；
- 高空飞越障碍可能影响在线避障得分，不能视为无代价行为。

如补充规则与本文件冲突，应先修订规则配置和本文，再运行正式实验。

### 2.2 参数证据等级

所有参数必须标注来源：

```text
RULE       正式比赛规则
MEASURED   视觉、飞行或投递实验测得
SIM        Gazebo或现有仿真配置
ASSUMED    尚无数据的临时假设
TUNED      通过仿真优化得到的策略参数
```

正式结论不得把`ASSUMED`或当前仿真随机分布描述为真实比赛事实。

## 3. 问题形式化

### 3.1 随机场景

将单次比赛场景记为：

\[
\omega=\{
\text{目标位置},
\text{障碍布局},
\text{视觉随机事件},
\text{复核结果},
\text{投递结果}
\}.
\]

搜索策略为`π`，待优化参数为：

\[
\theta=(h,v,s,\theta_{\rm lane},\eta),
\]

其中：

- `h`：搜索高度；
- `v`：搜索速度；
- `s`：航带间距；
- `θ_lane`：航带方向；
- `η`：机会式中断阈值或继续搜索的单位时间机会收益。

### 3.2 优化目标

比赛采用“得分优先、同分时间优先”，因此仿真采用字典序评价：

```text
1. 满足安全、时间和规则约束；
2. 最大化期望总得分 E[S]；
3. 最大化最差10%场景平均得分；
4. 得分近似相同时最小化期望任务时间 E[T]；
5. 再比较航程、能耗代理和重复覆盖率。
```

形式化主问题为：

\[
\max_{\pi,\theta}\;\mathbb E_{\omega}[S(\pi,\theta;\omega)]
\]

满足：

\[
T(\pi,\theta;\omega)\le 600,
\]

\[
\Gamma(\pi,\theta;\omega)\subseteq\Omega_{\rm free},
\]

\[
P_{\omega}(\text{collision})\le\epsilon.
\]

低分鲁棒性定义为得分分布最低10%样本的平均值：

\[
S_{\rm worst10\%}=E[S\mid S\le Q_{0.1}(S)].
\]

## 4. 仿真层级与基本假设

### 4.1 2.5D任务级模型

无人机状态至少包含`(x,y,h)`，障碍物至少包含：

```text
(x, y, width, length, height)
```

高度必须保留，因为它会影响：

- 相机地面视场；
- 目标像素尺寸和视觉概率；
- 遮挡关系；
- 下降复核时间；
- 航迹可达性和比赛避障收益。

### 4.2 离散事件推进

第一版不采用高频`dt`动力学积分。仿真只推进任务事件：

```text
ENTER_SEGMENT
TARGET_ENTERS_VIEW
CUE_SUCCESS / CUE_MISSED
INTERRUPT_SEARCH
ARRIVE_VERIFY_POINT
VERIFY_SUCCESS / VERIFY_FAILED
DROP_SUCCESS / DROP_FAILED
RESUME_SEARCH
ARRIVE_SEGMENT_END
STOP_SEARCH
MISSION_TIMEOUT
MISSION_FINISHED
```

该设计用于支持大规模Monte Carlo，同时保持中断和恢复过程可审计。

## 5. 场地与障碍模型

### 5.1 场地对象

`Field`至少包含：

```text
boundary
search_region
takeoff_region
landing_region
door_regions
obstacles
target_allowed_region
target_forbidden_regions
```

### 5.2 安全包络

无人机第一版视为任务级质点，障碍物按无人机碰撞半径与安全裕度膨胀：

\[
O_{\rm safe}=O\oplus B(r_{\rm uav}+r_{\rm margin}).
\]

碰撞包络参数必须来自机体尺寸、桨叶保护和规划安全裕度，不得随意用零半径替代。

### 5.3 绕障距离

任意任务级移动时间不得长期使用纯欧氏距离。首版可在二维膨胀障碍图上用A*近似：

\[
D_{\rm free}(p_a,p_b).
\]

移动时间为：

\[
T_{\rm move}=\frac{D_{\rm free}}{v}+T_{\rm turn}+T_{\rm accel}+T_{\rm settle}.
\]

若某阶段暂未实现A*，必须在结果中标明使用欧氏距离的偏差，并禁止据此得出障碍相关结论。

### 5.4 遮挡

碰撞障碍和视觉遮挡必须分别处理。目标进入理论FOV后，还需根据相机高度、目标位置和障碍高度判断视线是否被阻挡。

首版遮挡函数输出：

```text
VISIBLE
OCCLUDED
UNKNOWN
```

只有独立几何真值明确时才把遮挡样本计入相应条件下的视觉成功率。

## 6. 目标与随机场景模型

### 6.1 目标对象

`Target`至少包含：

```text
id
class_name
position
size
score_weight
cue_state
confirmed
serviced
drop_result
```

目标的真实位置只属于仿真环境真值，不得直接暴露给搜索策略。

### 6.2 场景族

正式规则没有规定目标和障碍物的概率分布，因此必须至少维护：

1. 当前仿真拒绝采样分布；
2. 合法区域内约束均匀分布；
3. 靠近边界、障碍和视野盲区的困难分布；
4. 人工构造的最坏或对抗场景。

每个场景必须保存：

```text
seed
生成器版本
全部真实参数
约束检查结果
```

策略不得只在单一世界或单一随机生成器上报告性能。

## 7. 航迹与覆盖模型

### 7.1 任务级航迹

搜索策略输出分段航迹：

\[
e_k=(p_k,p_{k+1},h_k,v_k).
\]

Fast-Planner负责在真实系统中将任务级端点转换为连续安全轨迹；`liftrace-sim`只近似其路径长度、转弯、加减速、规划失败和稳定时间。

### 7.2 相机地面视场

垂直下视且地面近似平坦时，理论视场尺寸为：

\[
W(h)=2h\tan\frac{\alpha_x}{2},
\]

\[
H(h)=2h\tan\frac{\alpha_y}{2}.
\]

正式版本应优先使用CameraInfo、相机外参和地面射线求交得到视场多边形，以上公式只作为早期校核。

### 7.3 航段驻留时间

对目标`i`和航段`k`，计算：

- 最小横向距离`d_ik`；
- 进入和离开有效视野的位置；
- 有效可见区间长度；
- 可见驻留时间。

\[
\tau_{ik}=\frac{l^{\rm out}_{ik}-l^{\rm in}_{ik}}{v_k}.
\]

驻留时间是连续多帧确认能力的主要解释变量。

### 7.4 可靠航带宽度

航带间距不能仅依据理论FOV确定。定义：

\[
W_{\rm reliable}(h,v,c)
=2\max_d\{d:P_{\rm cue}(h,v,d,c)\ge P_{\min}\}.
\]

基础安全策略要求：

\[
s\le\min_c W_{\rm reliable}(h,v,c).
\]

后续可根据类别收益和剩余时间研究加权可靠宽度，但必须与无权重保底策略比较。

## 8. 视觉概率模型

### 8.1 三阶段分解

视觉与任务结果拆为三个条件随机过程：

```text
Cue：一次搜索航段内形成可信线索；
Verify：由Cue主动接近或下降后形成可靠地图目标；
Drop：确认后完成有效对准和投递。
```

\[
P_{\rm service}
=P_{\rm cue}
P_{\rm verify\mid cue}
P_{\rm drop\mid verify}.
\]

不得默认三个阶段相互独立。

### 8.2 Cue事件定义

`P_cue`表示一次飞越中产生稳定任务级线索或候选的概率，而不是单帧YOLO检出率：

\[
P_{\rm cue}=f(h,d,\tau,\psi,c,o,q).
\]

其中：

- `h`：高度；
- `d`：横向偏差；
- `τ`：可见驻留时间；
- `ψ`：相对视角或偏航；
- `c`：类别；
- `o`：遮挡；
- `q`：光照、角速度、系统负载等实验条件。

速度可通过驻留时间进入主模型，也可作为运动模糊修正项，但拟合时必须检查变量共线性。

### 8.3 数据来源

视觉组应从`uav_vision_eval`输出按以下维度分组的数据：

```text
height
actual_speed
lateral_offset
yaw
class
fully_in_frame
occlusion
runtime_profile
```

需要统计：

```text
raw检测率
圆环关联率
map_valid率
CONFIRMED率
selected_target率
确认时间
地图误差
端到端墙钟延迟
```

没有实测数据时可使用参数化临时模型或概率查表，但必须标为`ASSUMED`，并进行敏感性分析。

## 9. 任务状态、时间预算与中断决策

### 9.1 任务状态

`MissionState`至少包含：

\[
X_t=(
p_t,t,n_{\rm cargo},
\mathcal T_{\rm cue},
\mathcal T_{\rm confirmed},
\mathcal T_{\rm serviced},
\mathcal E_{\rm remaining},
S_t,
\text{phase}
).
\]

### 9.2 剩余时间

\[
T_{\rm search,remain}
=600-t-T_{\rm door}-T_{\rm land}-T_{\rm safety}.
\]

当`T_search,remain <= 0`时，策略必须停止搜索并进入保分流程。

### 9.3 机会式中断

目标`i`的期望服务价值为：

\[
V_i=P_i^{\rm true}P_i^{\rm service}R_i.
\]

中断价值定义为：

\[
V_{\rm interrupt}
=P_i^{\rm true}P_i^{\rm service}R_i
-\eta_{\rm search}\Delta T_i.
\]

当`V_interrupt > 0`且时间、安全、货物、可达性约束满足时允许中断。红十字应因规则收益自然获得高优先级，而不是仅依赖硬编码类别分支。

### 9.4 Resume策略

至少比较：

```text
ResumeOriginal：返回原中断位置并继续原航段；
ResumeNearest：从当前位置接入最近未完成航段；
```

后续可研究：

\[
e^*=\arg\max_{e\in\mathcal E_{\rm rem}}
[G_{\rm remaining}(e)-\lambda D_{\rm free}(p,e)].
\]

## 10. 比赛计分模型

### 10.1 投递收益

目标权重来自正式规则配置。单个目标的期望投递收益为：

\[
E[S_i]
=P_{\rm service,i}
\cdot E[S_{\rm landing\ ring},i]
\cdot w_i.
\]

实现时必须避免重复投递、超过3件货物或给遗留`tank`类别计入正式收益。

### 10.2 完整任务收益

总分模型后续应包含：

```text
携带分
自主起飞分
在线避障分
投递分
两次过门分
自主降落分
碰撞或任务终止后的保留规则
```

首版若只实现搜索与投递子分，必须将结果命名为`delivery_score`，不得冒充完整比赛总分。

## 11. 核心软件对象

第一版只保留六个核心对象：

```text
Field
Target
VisionModel
SearchPolicy
MissionState
Simulator
```

职责边界：

### `Field`

```text
边界、障碍、安全膨胀、搜索域、门、起降区
shortest_path()
visibility()
```

### `Target`

```text
类别、真值位置、尺寸、权重和服务状态
```

### `VisionModel`

```text
cue_probability()
verify_probability()
verify_time()
drop_probability()
drop_time()
```

### `SearchPolicy`

```text
根据MissionState选择下一航段、复核、投递、Resume或结束动作
```

### `MissionState`

```text
保存时间、位置、货物、得分、覆盖进度、目标集合和任务阶段
```

### `Simulator`

```text
生成和推进事件、采样随机结果、记录审计日志并输出单次任务结果
```

## 12. 首轮策略与参数空间

首轮基准策略：

```text
LowBoustrophedon + ResumeOriginal
LowBoustrophedon + ResumeNearest
```

随后加入：

```text
HighScan
HighCueThenDescend
HybridSearch
AdaptiveCoverage
```

首轮只优化：

\[
\theta=(h,v,s,\theta_{\rm lane},\eta).
\]

航带方向先取：

```text
0°、90°
```

必要时再加入：

```text
45°、135°
```

参数维度较低时优先使用Grid Search或随机搜索，不引入遗传算法、强化学习或复杂贝叶斯优化。

## 13. 仿真阶段

### M0：几何与计时校核

- 矩形场地；
- 无视觉随机性；
- 生成牛耕航迹；
- 校验覆盖面积、航程、转弯数和理论时间。

完成标准：简单案例能人工复算，误差来源有说明。

### M1：航段级视觉事件

- 加入随机目标；
- 计算FOV相交、横向偏差和驻留时间；
- 加入`P_cue`；
- 统计发现率和首次发现时间。

完成标准：固定seed完全可复现，概率极限测试正确。

当前实施状态：M1a确定性几何和M1b第一版随机实验框架均已完成。M1b包括固定种子的受约束目标生成、完整入画航段、可替换的航段级`P_cue`接口、Bernoulli抽样以及分类别Monte Carlo统计。当前目标分布与`P_cue`参数均为`ASSUMED`，必须由视觉组实验替换后才能用于策略结论。

参数研究状态：已加入高度、速度、航带间距的全因子网格扫描。各组合使用公共随机场景，并按“总时间最小、平均Cue率最大”提取Pareto前沿；在真实视觉参数和正式任务收益接入前，不定义武断的标量最优解。

### M2：Cue、Verify、Drop与中断恢复

- 加入三阶段条件概率；
- 加入货物、目标状态和投递收益；
- 比较两种Resume策略；
- 加入600 s与安全结束时间预算。

完成标准：不存在重复服务、负时间、超过货物数量或恢复死循环。

当前实施状态：已加入正式规则的600 s限时、3件载荷容量和逐项计分器。计分器实现
载货、自主起飞、水平避障、分类别加权投递、两道门和自主降落，并按“总分优先、
同分用时较短优先”生成排序键。尚未由任务状态机观测到的避障、穿门和降落项目
明确计0且写入`unmodelled_items`，不会用假设分数填满总分。现有投递成功事件尚无
落点散布，暂按最内圈/完全入圈计分并标为乐观假设。

在线投递决策已加入四个可替换基线：先见先投、固定高权重类别、为红十字预留一件
物资、随搜索进度衰减的机会成本阈值。决策发生在Cue之后、接近复核之前。比较实验
对所有策略复用同一目标布置和Cue随机数，并把复核/投递随机结果绑定到`target_id`，
从而能够进行逐场景配对差值分析。动态阈值仍是研究策略，其未来Cue先验尚未标定，
不能直接称为最优策略。

动态阈值已完成第一轮二维调参和独立seed验证：在`p=0.25–1.00`、
`gamma=0.50–2.00`的36点网格中用500场景筛选，再用2000个新场景验证前5名。
当前候选为`p=0.40, gamma=0.50`。这里的`p`是策略侧机会价值超参数；由于场景分布、
复核/投递概率和落点仍含假设，调参结果不能解释为真实物理Cue概率。

### M3：障碍、遮挡和近似绕障

- 加入2.5D障碍；
- 加入安全膨胀；
- 加入A*绕障距离；
- 加入视线遮挡；
- 统计规划不可达和碰撞风险。

完成标准：障碍影响能够通过人工构造案例验证。

### M4：高空与混合搜索

- 加入高空Cue模式；
- 加入下降复核时间和失败；
- 加入高空避障得分代价；
- 比较低空主搜、高空主搜和混合策略。

完成标准：策略比较使用同一场景集和同一随机数控制。

### M5：Gazebo与真实数据校准

- 用视觉实验替换临时概率模型；
- 用Gazebo测得的路径时间校准运动时间；
- 将最优候选参数送入`uav_mission`闭环复核；
- 对比任务级预测与Gazebo实际结果。

完成标准：记录仿真偏差，并明确模型的可用范围。

## 14. Monte Carlo实验设计

每组参数至少保存：

```text
experiment_id
config_version
policy_name
parameter_set
master_seed
scene_seeds
vision_model_version
result_schema_version
```

比较策略时使用共同随机数：不同策略运行同一批场景seed和可复现随机流，减少场景差异带来的比较噪声。

第一轮每组参数建议运行`N=1000`个世界；开发期间可用较小样本，正式报告必须给出置信区间或自举区间。

## 15. 输出指标

### 15.1 主要指标

```text
mean_score
worst_10_percent_mean_score
score_std
mission_completion_rate
collision_rate
timeout_rate
```

### 15.2 搜索与视觉指标

```text
target_cue_rate_by_class
red_cross_cue_rate
first_cue_time
all_required_targets_time
verify_success_rate
false_cue_cost
mean_visible_dwell_time
coverage_ratio
repeat_coverage_ratio
```

### 15.3 投递与任务指标

```text
service_rate_by_class
three_drop_completion_rate
delivery_score
door_score
landing_score
total_time
search_time
verify_time
drop_time
resume_distance
total_distance
```

所有aggregate必须同时提供逐类别、逐场景族和逐参数条件结果。

## 16. 校准、验证与防止自证

### 16.1 单元级验证

必须为以下功能建立可人工复算的小场景：

- 点到航段距离；
- FOV进入和离开区间；
- 驻留时间；
- 障碍膨胀；
- A*距离；
- 视线遮挡；
- 计分；
- 时间预算；
- 中断与Resume；
- 固定seed复现。

### 16.2 概率模型验证

至少测试：

```text
P=0时绝不成功
P=1时总是成功
条件概率链结果正确
样本频率随N增加收敛
类别和高度参数不会串用
```

### 16.3 外部验证

数学仿真输出只能生成候选策略。任何正式工程结论必须经过：

```text
liftrace-sim Monte Carlo
→ liftrace-controlwork Gazebo闭环
→ 必要的实机验证
```

不得用数学仿真结果替代视觉、规划或飞行安全验收。

## 17. 假设与变更管理

维护独立假设表，每条至少包含：

```text
assumption_id
description
evidence_level
current_value
sensitivity_range
owner
replacement_data
status
```

修改规则、概率模型、场景生成器、计分或核心时间参数时，必须更新模型版本。不同模型版本的结果不得直接混合汇总。

## 18. 第一轮实施清单

第一轮只完成以下事项：

1. 建立参数与假设配置；
2. 实现`Field、Target、VisionModel、MissionState`最小数据结构；
3. 实现低空牛耕航段生成；
4. 实现航段与目标视场几何关系；
5. 实现航段级Cue采样；
6. 实现Cue→Verify→Drop事件链；
7. 实现中断、ResumeOriginal和ResumeNearest；
8. 实现600 s、3件货物和正式目标收益；
9. 运行固定seed单次任务并输出事件日志；
10. 运行首轮Monte Carlo，比较少量`h、v、s`组合。

第一轮明确不做：

```text
ROS节点
Gazebo插件
PX4或电机动力学
Fast-Planner源码复现
强化学习
遗传算法
任意连续曲线优化
复杂GUI
```

## 19. 首轮成功判据

满足以下条件后，才进入高空与混合策略研究：

- 固定seed结果完全可复现；
- 基础几何、时间和计分可以人工复算；
- 事件日志能够解释每次Cue、中断、复核、投递和恢复；
- 概率模型的0、1和统计收敛测试通过；
- 三件货物、600 s和正式五类目标约束正确；
- 牛耕参数变化能产生符合物理直觉的覆盖、时间和发现率变化；
- ResumeOriginal与ResumeNearest能在同一场景集公平比较；
- 所有临时参数均有`ASSUMED`标记和敏感性范围；
- 输出结果可追溯到配置、模型版本和seed。

## 20. 当前统一结论

`liftrace-sim`第一版采用：

```text
2.5D场地
+ 离散事件推进
+ 航段级视觉概率
+ Cue/Verify/Drop条件链
+ 搜索中断与可比较Resume
+ 正式规则计分
+ Monte Carlo与低分鲁棒性评价
```

牛耕搜索是第一基线和覆盖保底，不预设为最终最优策略。高空主搜、低空主搜和混合策略的优劣必须由同一场景集、同一视觉模型和同一比赛计分口径下的统计实验决定。

本项目的第一使命是建立：

\[
(h,v,s,\eta)
\longrightarrow
P_{\rm cue},
T_{\rm search},
P_{\rm service},
E[S],
S_{\rm worst10\%}
\]

这一条可标定、可复现、可优化并能被Gazebo验证的因果链。
