# R64 实录基线与随机地图

2026-09-09。固定seed11完整PASS；冻结版seed1–10原始7/10完整PASS、8/10三投。实际布设几何复核发现5/7/8靶板压墙，保留这些原始结果和非法几何标记；不能把整批当作10个合法场景，不能只选择成功样本校准轻量模型。主要剩余问题是seed3近地落地以及近墙捕获净空。

`data/recorded_fields/r64_matrix.json`包含11轮实际布局、墙几何、结果及约1Hz完整刚体轨迹；FC高度已由记录local z转换为真实AGL，保留四元数。不是原始bag，不是硬件记录。相机仍为FC下16cm、IMU下21cm的刚性下视安装，没有随航迹自动旋转的云台。

```bash
python tools/r64_visibility.py --input data/recorded_fields/r64_matrix.json \
  --output results/r64_visibility.json
```

该工具调用已安装相机投影模型，对真实姿态序列统计目标中心/完整外圈的视锥几何样本数。没有包含墙/树遮挡或YOLO误检漏检，不能称为识别召回率；这是探索搜索策略的几何基线。旧M0/M2的高空、自动朝航迹转头、理想路径假设不作为当前实机保证。

随机地图模块`r2026_scene.py`提供严格0.80m左/右门四组合和独立树箱位置/朝向；`tools/export_r2026_scene.py`将当前完整SDF模板转换为可用场景。目标seed仍由整机spawner独立控制。场内墙实体应按AABB排除，不能仅按搜索矩形或整个场地模型的包围圆处理。

scene_seed=0保留标称布局。生成器只保证其几何约束，未知门和随机树箱的实际SITL尚未验证。导出的实验中线航点不读取门左右真值；门真值只用于评测配置。不要把理想A*路径输出伪装成飞机自主感知到的通口。

[整机完整矩阵报告](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/blob/feat/r2026-competition-integrated/docs/verification/r64_matrix/REPORT.md) · [随机化使用及边界](https://github.com/Qinling-Melon-Farmers/liftrace-visionwork/blob/feat/r2026-competition-integrated/docs/verification/r64_randomization/README.md)。
