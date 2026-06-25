# Debug 扩展规范

本文档只记录 `armor_detector` 包内 debug 功能的扩展方式和代码规范。单位、坐标系、角点顺序、`cv::Mat` 生命周期等全局硬约定见 `Conventions.md`。

## 总体原则

- 主流程只负责算法顺序，不直接写窗口显示、ROI 保存、耗时统计等 debug 细节。
- Debug 功能通过 `IDebugObserver` 扩展，由 `DebugHub` 统一注册和分发事件。
- 所有 OpenCV 窗口、`imshow`、`waitKey`、`destroyAllWindows` 只能放在 `DebugGUI` 或其实现中。
- Observer 只读主流程数据，不反向修改算法结果。
- 新增 debug 功能优先新增一个 `DebugXXX` Observer，不要把功能塞回 `DetectorNode` 主流程。
- Debug 数据中的单位、坐标和 `cv::Mat` 生命周期必须遵守 `Conventions.md`。

## 文件命名

Debug 相关文件放在：

```text
include/armor_detector/debug/
src/debug/
```

命名建议：

```text
DebugData.hpp             阶段 debug 数据结构
IDebugObserver.hpp        Observer 接口
DebugHub.hpp              事件分发器
DebugGUI.hpp              GUI 统一出口
DebugKeyHandler.hpp       按键转换
DebugTiming.hpp           耗时统计
DebugPreprocess.hpp       预处理图层
DebugLight.hpp            灯条图层
DebugArmorMatch.hpp       装甲板匹配图层
DebugClassification.hpp   数字分类图层
DebugPose.hpp             位姿/坐标图层
DebugRoiRecorder.hpp      ROI 录制
```

新增功能使用 `Debug + 功能名`，例如：

```text
DebugPnpError.hpp
DebugRejectedArmor.hpp
DebugYawSearch.hpp
```

## 新增 Observer 的步骤

1. 判断这个功能关心哪个阶段事件。
2. 若现有事件数据不够，在 `DebugData.hpp` 中补充对应阶段的数据结构。
3. 新增 `DebugXXX` 类，继承 `IDebugObserver`。
4. 只重写自己关心的事件，其他事件保持默认空实现。
5. 如果需要显示图像，把图像提交给 `DebugGUI`，不要直接调用 `cv::imshow`。
6. 如果需要响应按键，实现 `onKey(const DebugKeyEvent & event)`。
7. 在初始化 debug 时，根据配置参数决定是否注册到 `DebugHub`。

示例：

```cpp
class DebugPnpErrorView : public IDebugObserver
{
public:
    explicit DebugPnpErrorView(DebugGUI & gui);

    void onPoseSolved(const DebugFrameContext & context, const PoseDebugData & data) override;

private:
    DebugGUI * gui_ = nullptr;
};
```

## 事件使用规范

当前保留这些阶段事件：

```text
onFrameStart
onPreprocess
onLights
onArmorMatch
onClassification
onPoseSolved
onFrameEnd
onKey
```

事件粒度保持“阶段级”，不要在像素循环、轮廓循环等高频内层逻辑里大量发事件。

适合用事件的内容：

```text
显示中间图
绘制检测结果
保存 ROI
统计耗时
记录 benchmark 数据
显示拒绝原因
```

不适合用事件的内容：

```text
灯条几何判断
PnP fallback
yaw 优化方法选择
会改变算法输出的逻辑
```

这些应留在算法代码或策略类中。

## 开关规范

Debug 开关控制 Observer 是否注册，而不是在主流程中堆 `if debug_xxx`。

推荐：

```cpp
if (config.debug.preprocess) {
    debug_hub.addObserver(std::make_shared<DebugPreprocessView>(debug_gui));
}
```

不推荐：

```cpp
if (debug_preprocess) {
    drawPreprocessInMainFlow();
}
```

配置结构：

```yaml
debug:
  show: true
  rosbag_control: true
  rosbag_player_node: "/rosbag2_player"
  preprocess: false
  lights: true
  armor_match: false
  classification: false
  pose: false
  result: true
  stats_interval: 50
```

## GUI 规范

- 默认只使用 `debug_show` 一个窗口，各模块作为图层叠加显示。
- 多个模块同时启用时，由配置或后续按键决定哪些图层参与绘制。
- 只有 `DebugGUI` 负责 OpenCV 窗口生命周期。
- Observer 需要显示图像时，提交窗口名、图像和缩放比例。
- `cv::Mat` 是否 clone 遵守 `Conventions.md`，不要在 Observer 中随意长期持有共享 Mat。
- GUI 显示元素、文字位置、颜色和绘制样式见 `DebugGUI.md`。
- 需要无窗口运行时，通过 `DebugGUI::setEnabled(false)` 关闭所有显示。

## 按键规范

`cv::waitKey` 的原始值由 `DebugKeyHandler` 转换为 `DebugKeyEvent`。

当前保留动作：

```text
EXIT          退出节点 / 关闭窗口
PAUSE_TOGGLE  rosbag 暂停/继续
STEP_FRAME    rosbag 单步
SAVE_ROI      保存 ROI（预留）
TOGGLE_LAYER  切换图层显示（数字键 1-6）
```

Observer 不直接读 `cv::waitKey`，只在 `onKey` 中响应语义动作。

## 图层切换

数字键 1–5 切换对应 debug 图层的显示状态：

| 按键 | 图层 | 说明 |
|---|---|---|
| 1 | preprocess | 预处理中间图（独立窗口 preprocess_debug） |
| 2 | lights | 灯条检测结果（叠加到 debug_show） |
| 3 | armor_match | 装甲板匹配（预留，当前只切状态） |
| 4 | classification | 数字分类（预留，当前只切状态） |
| 5 | pose | 位姿解算（预留，当前只切状态） |
| 6 | result | 最终识别装甲板 X 标记 |

切换后输出日志，例如：`DEBUG layer lights: ON`

## Timing 行为

- `timing` 不是图层——`debug.show=true` 时始终启用，无法通过数字键关闭。
- 每帧在 `debug_show` 左上角绘制 `Process` 总耗时和各阶段耗时。
- 每 `debug.stats_interval` 帧（默认 50）打印一次平均耗时日志。

## rosbag 控制规划

当前阶段只写规范，不实现 rosbag 服务客户端。

未来可通过配置开启 DebugGUI 调用 rosbag2 服务：

```text
TogglePaused  暂停/继续
PlayNext      播放下一条消息，仅 paused 状态下有效
```

默认仍允许使用 `ros2 bag play` 终端键盘控制。自动化测试如需稳定控制回放，优先使用 rosbag2 服务而不是模拟终端按键。

## 数据结构规范

- 阶段 debug 数据统一放在 `DebugData.hpp`。
- 数据结构只表达“该阶段产生了什么”，不要混入窗口布局细节。
- 拒绝原因使用枚举 `DebugRejectReason`，补充细节放到 `detail` 字符串。
- 如果字段会影响算法结果，它不应该放在 debug 数据里。

## 下一阶段目标

基础 debug 设施稳定后，下一阶段优先设计多线程 GUI。该阶段需要重新讨论 `cv::Mat` 所有权、跨线程队列、窗口刷新频率和退出顺序。

## 不要提前实现的内容

当前阶段不要把下面内容塞进 debug 框架：

```text
Tracker / Planner overlay
KeyFrameCache 跨 topic 聚合
世界坐标同步
ROS debug msg
通用 EventBus 模板系统
```

等单包检测流程稳定后，再按需要扩展。
