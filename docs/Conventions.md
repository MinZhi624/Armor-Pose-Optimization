# 项目数据约定

本文档记录 `armor_detector` 的全局硬性数据约定。单位、坐标系、角点顺序、`cv::Mat` 生命周期等基础规则统一放在这里；debug、GUI、算法参考文档只引用本文件，不重复定义。

## 基本原则

- 代码实现、算法文档、debug 数据结构必须遵守本文档约定。
- 如果某处确实需要例外，必须在对应文档和代码注释中说明原因。
- 配置参数、结构体字段、局部变量命名应尽量体现单位，特别是角度。

## 单位约定

| 数据类型 | 单位 | 命名要求 | 说明 |
|---|---|---|---|
| 三维长度 / 三维坐标 | `m` | 必要时使用 `_m` | `xyz_camera`、`xyz_gimbal`、装甲板 3D 模型点等 |
| 图像坐标 / 图像距离 / 图像宽高 | `px` | 必要时使用 `_px` | `cv::Point2f`、角点、中心点、图像距离等 |
| 图像面积 | `px2` | 必要时使用 `_px2` | 轮廓面积、灯条面积等 |
| 三维姿态角 | `rad` | 标量建议使用 `_rad` | yaw、pitch、roll、YPR、YPD 等内部存储和运算 |
| 图像平面几何角 / 判断阈值 | `deg` | 必须使用 `_deg` | `cv::RotatedRect::angle`、灯条角度、角度差、配置角度阈值 |
| 处理耗时 | `ms` | 必须使用 `_ms` | debug 显示、阶段耗时统计 |
| 分类置信度 | `0.0~1.0` | 使用 `confidence` | 显示时再转换为百分比 |

角度补充规则：

- 三维姿态相关计算统一使用 `rad`。
- 图像几何判断、人工调参配置统一使用 `deg`，因为 OpenCV 图像角和调参阈值更直观。
- 如果 `deg` 值进入三维姿态计算，必须显式转换为 `rad`。

## 坐标系约定

| 坐标系 | 含义 | 轴方向 | 当前阶段使用情况 |
|---|---|---|---|
| `image pixel` | 图像像素坐标 | 原点左上，x 向右，y 向下 | 角点、灯条、ROI、绘图 |
| `camera` | OpenCV 相机坐标系 | x 向右，y 向下，z 向前 | PnP 输出、相机系位姿 |
| `gimbal` | 项目习惯命名的观察坐标系 | x 向前，y 向左，z 向上 | 由 `camera` 固定转换得到，不依赖真实云台姿态 |
| `world` | 世界坐标系 | 待未来云台姿态定义 | 当前阶段不作为核心依赖 |

`camera` 到 `gimbal` 的固定转换：

```cpp
x_gimbal =  z_camera;
y_gimbal = -x_camera;
z_gimbal = -y_camera;
```

当前阶段不要把 `gimbal` 理解为真实云台姿态积分后的世界相关坐标。真实云台姿态参与后再引入 `world`。

## 图像几何顺序

装甲板四个角点顺序固定为：

```text
left_top
right_top
right_bottom
left_bottom
```

`ArmorGeometry::paired_lights` 顺序固定为：

```text
paired_lights[0] = left light
paired_lights[1] = right light
```

左右以图像 x 坐标排序为准。

## 数据流与 `cv::Mat` 生命周期

当前单帧数据流：

```text
接收图像 -> 检测/分类/位姿解算 -> 绘制 debug 图层 -> imshow
```

`cv::Mat` 生命周期规则：

- 默认按 OpenCV 浅拷贝语义共享数据，不主动 clone。
- Debug 数据结构中的 `cv::Mat` 默认只在当前帧、当前回调/处理流程内有效。
- `DebugGUI` 默认按短生命周期显示设计：`setFrame` 后应在同一帧流程内 `showOnce` 消费。
- 任何跨帧保存、异步线程、队列缓存、后台写文件、延迟显示的图像，必须显式 `clone()`。
- 如果未来引入 planner/tracker 回传和跨 topic 聚合，需要重新定义 GUI 所有权回收策略。

## 命名拼写规范

重构后的公开类型和字段只保留正确拼写。新增代码统一使用：

```text
ClassifiedArmor
angle_deg
image_distance_to_center
```

历史错误拼写不再保留兼容别名；如果旧代码引用这些名称，应直接迁移到上面的新名称。
