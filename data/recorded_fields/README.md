# 记录场景与离线样本

- r60_actual_seed01_11.json：实际R60十组及seed11先导的靶标中心/朝向，旧8m宽场景；树冠显示半径为示意，不是完整碰撞网格。
- r61_inner960_seed01_11.json：新的9.6m内净场景，直接调用整机生产plan_footprint_layout生成11套；seed11与原生Gazebo实际生成记录逐坐标核对（误差<0.11mm/0.00011rad），其余10套未飞行。不能把它们说成11次新的SITL实验。
- config/r61_camera_info.json只复用真实CameraInfo内参/畸变；相机安装在离线模型中用最终确认的FC下16cm，AGL1.4m时光心高1.24m。固定机头yaw0，不随航段转向。

场景真值只用于离线已知地图几何预筛；路线生成不使用靶标坐标。轻量模型不覆盖在线建图误差、飞控动力学、真实YOLO概率或完整比赛Gate。
