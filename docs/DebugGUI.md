# DebugGUI 注册规范

本文档记录 `armor_detector` debug GUI 的显示注册规则。默认只使用 `debug_show` 一个窗口；各模块通过图层叠加显示。单位、坐标系、`cv::Mat` 生命周期见 `Conventions.md`。

## 总体原则

- OpenCV 颜色统一使用 BGR 顺序。
- 所有 debug 显示内容都要登记，避免窗口中元素互相覆盖或颜色语义冲突。
- 文字叠加和图形绘制分开登记。
- 文字样式统一，不在各个 Observer 中随意修改字体、字号、行高。
- 图形颜色、线宽、点半径等参数应集中注册，不在代码中散落魔法值。
- GUI 只表达 debug 语义，不参与算法判断。
- 如果显示内容、位置、行号、颜色或绘制样式发生变化，必须同步更新本文档。

## 窗口策略

当前默认窗口：

```text
debug_show
```

默认不为每个模块单独开窗口。后续确实需要独立窗口时，必须先在本文档新增对应窗口注册表。

## 文字叠加规范

### 统一文本样式

当前所有 debug 文字统一使用下面样式：

```text
font_face:      cv::FONT_HERSHEY_SIMPLEX
font_scale:     0.60
font_thickness: 2
line_type:      cv::LINE_AA
line_height_px: 24
margin_x_px:    10
margin_y_px:    24
```

说明：

- `line_height_px` 是同一位置内相邻两行文字的垂直间距。
- `margin_x_px` / `margin_y_px` 是文字锚点距离窗口边缘的基础边距。
- 后续如果某个窗口确实需要不同字号，必须在该窗口小节中单独说明原因。

### 文本位置槽位

每个窗口固定提供四个文本位置：

```text
TOP_LEFT      左上
BOTTOM_LEFT   左下
TOP_RIGHT     右上
BOTTOM_RIGHT  右下
```

行号从 1 开始。同一位置内：

```text
line 1
line 2
line 3
...
```

建议用途：

```text
TOP_LEFT      帧号、处理用时、主状态
TOP_RIGHT     当前参数、模式、开关状态
BOTTOM_LEFT   检测数量、拒绝原因摘要
BOTTOM_RIGHT  坐标、分类、PnP 等结果摘要
```

### 文本注册格式

每个窗口按下面格式登记文字：

```text
## 窗口名 - Text

| 位置 | 行号 | 内容 | 字体颜色(BGR) | 说明 |
|---|---:|---|---|---|
| TOP_LEFT | 1 | frame processing time | (0, 165, 255) | 当前帧或阶段处理用时 |
```

字段说明：

- `位置`：只能使用 `TOP_LEFT`、`BOTTOM_LEFT`、`TOP_RIGHT`、`BOTTOM_RIGHT`。
- `行号`：同一位置内从 1 开始递增。
- `内容`：写语义，不写具体变量值，例如 `frame processing time`，不要写 `Process: 12.3 ms`。
- `字体颜色(BGR)`：OpenCV BGR 三元组。
- `说明`：解释该行显示什么、什么时候显示。

## 图形绘制规范

图形绘制部分参考旧项目的 GUI debug 颜色注册方式，但改成表格登记。当前先定义格式，具体颜色和元素后续按图层补充。

### 图形注册格式

`debug_show` 中每个模块按图层登记：

```text
## debug_show - Draw

| 图层 | 元素 | 内容 | 颜色(BGR) | 样式 | 显示条件 | 说明 |
|---|---|---|---|---|---|---|
| lights | rotated_rect | accepted light bar | (0, 255, 0) | thickness=2 | debug_lights enabled | 已通过筛选的灯条 |
```

字段说明：

- `图层`：同一窗口内的逻辑层，例如 `preprocess`、`lights`、`armor_match`、`classification`、`pose`。
- `元素`：绘制元素类型，例如 `point`、`line`、`rect`、`rotated_rect`、`polygon`、`text_background`。
- `内容`：元素语义，例如 `accepted light bar`、`rejected armor candidate`。
- `颜色(BGR)`：OpenCV BGR 三元组。
- `样式`：线宽、半径、填充方式等，例如 `thickness=2`、`radius=5 filled`。
- `显示条件`：什么时候显示，例如 `always`、`debug_lights enabled`、`pnp failed`。
- `说明`：补充解释。

### 图形使用规则

- `cv::Mat` 所有权和 clone 规则遵守 `Conventions.md`。
- 同一窗口内相同语义保持同色，不同语义尽量不用同色。
- 绘制点、线、框、文字时，必须能在原始图像和灰度/二值背景上看清。
- 如果颜色不足以区分语义，应增加文字标签、线型或填充差异，不要只靠相近颜色区分。
- 对同一目标的不同阶段结果，颜色应稳定，不要每帧随机变色。
- 如果 `debug_show` 图形元素过多，应通过配置/按键关闭部分图层；确实需要时再新增独立窗口。

## 当前注册表

## debug_show - Text

| 位置 | 行号 | 内容 | 字体颜色(BGR) | 说明 |
|---|---:|---|---|---|
| TOP_LEFT | 1 | frame processing time (total) | (0, 165, 255) | 当前帧总处理耗时，由 DebugTiming 绘制 |

## debug_show - Draw

| 图层 | 元素 | 内容 | 颜色(BGR) | 样式 | 显示条件 | 说明 |
|---|---|---|---|---|---|---|
| lights | rotated_rect | accepted light bar | (0, 255, 0) | thickness=2, LINE_AA | layer lights ON | 已通过筛选的灯条 |
| lights | point | light endpoints (top/bottom) | (255, 0, 255) | radius=1, filled | layer lights ON | 灯条端点标记 |
| lights | rotated_rect | rejected light bar | (0, 0, 255) | thickness=2, LINE_AA | layer lights ON | 被拒绝的灯条 |
| lights | text | reject reason detail | (0, 0, 255) | scale=0.60, thickness=2 | layer lights ON | 拒绝原因文字，偏移 +5px |
| result | line | final classified armor X mark | (255, 0, 255) | thickness=1, LINE_AA | layer result ON | 最终识别装甲板 X 标记，紫色 |

## /armor_markers (Foxglove/RViz)

Pose debug 通过 `/armor_markers` topic 发布 `visualization_msgs/msg/MarkerArray`，由 `DebugPoseMarkerPublisher` observer 负责，受 `debug.pose` 图层控制（按键 5 切换）。

| 属性 | 值 | 说明 |
|---|---|---|
| topic | /armor_markers | MarkerArray |
| frame_id | gimbal | gimbal 坐标系 |
| ns | armor_pose | 命名空间 |
| type | CUBE | 3D 立方体 |
| 位置 | xyz_gimbal | 装甲板在 gimbal 坐标系下的位置 |
| 姿态 | yaw(PnP) + pitch(15°固定) + roll(0) | 从 ypr_gimbal 构造四元数 |
| scale.x | 0.01 | 厚度 (m) |
| scale.y | 0.135 / 0.225 | small / large 装甲板宽度 (m) |
| scale.z | 0.055 | 装甲板高度 (m) |
| 颜色 | (r=1.0, g=1.0, b=0.0, a=0.7) | 黄色半透明 |
| lifetime | 0 | 永不过期，由 DELETEALL 清理 |

生命周期：每帧先发 DELETEALL 清空上一帧，再发当前帧所有 cube。debug.pose 关闭时发 DELETEALL 清残留。
