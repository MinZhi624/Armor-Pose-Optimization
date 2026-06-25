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

当前已有文件：

```text
DebugData.hpp                   阶段 debug 数据结构
IDebugObserver.hpp              Observer 接口
DebugHub.hpp                    事件分发器
DebugGUI.hpp                    GUI 统一出口
DebugKeyHandler.hpp             按键转换
DebugLayerState.hpp             图层开关状态（线程安全）
DebugLayerController.hpp        数字键切换图层
DebugTiming.hpp                 耗时统计（无头模式也打印日志）
DebugPoseMarkerPublisher.hpp    /armor_markers MarkerArray 发布
DebugPreprocess.hpp             预处理图层
DebugLight.hpp                  灯条图层
DebugArmorMatch.hpp             装甲板匹配图层
DebugClassification.hpp         数字分类图层
DebugPose.hpp                   位姿/坐标图层
DebugResult.hpp                 最终结果图层
DebugRoiRecorder.hpp            ROI 录制
```

新增功能使用 `Debug + 功能名`，例如：

```text
DebugPnpError.hpp
DebugRejectedArmor.hpp
```

## 新增 Observer 的步骤

1. 判断这个功能关心哪个阶段事件。
2. 若现有事件数据不够，在 `DebugData.hpp` 中补充对应阶段的数据结构。
3. 新增 `DebugXXX` 类，继承 `IDebugObserver`。
4. 只重写自己关心的事件，其他事件保持默认空实现。
5. 如果需要显示图像，把图像提交给 `DebugGUI`，不要直接调用 `cv::imshow`。
6. 如果需要发布 ROS topic（如 MarkerArray），在构造时创建 publisher。
7. 如果需要响应按键，实现 `onKey(const DebugKeyEvent & event)`。
8. 在 `DetectorNode::initDebug()` 中注册到 `DebugHub`。

## Observer 分类

### 非 GUI Observer（始终注册）

不受 `debug.show` 控制，即使无头模式也工作：

- **DebugTiming** — 阶段耗时统计，每 `debug.stats_interval` 帧打印平均用时日志
- **DebugPoseMarkerPublisher** — `/armor_markers` MarkerArray 发布，受 `debug.pose` 图层控制

### GUI Observer（仅 debug.show=true 时注册）

需要 OpenCV 窗口，无头模式下不注册：

- DebugPreprocessView, DebugLightView, DebugArmorMatchView
- DebugClassificationView, DebugResultView
- DebugLayerController（数字键切换）

注册逻辑：

```cpp
void DetectorNode::initDebug() {
    // 非 GUI observer：始终注册
    debug_hub_.addObserver(std::make_shared<debug::DebugTiming>(debug_config_.stats_interval));
    debug_hub_.addObserver(std::make_shared<debug::DebugPoseMarkerPublisher>(*this, layer_state_));

    // GUI observer：仅 debug.show=true 时注册
    if (!debug_config_.show) return;
    // ... 注册 GUI observers
}
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

事件粒度保持"阶段级"，不要在像素循环、轮廓循环等高频内层逻辑里发事件。

## 开关规范

Debug 开关控制 Observer 是否注册，而不是在主流程中堆 `if debug_xxx`。

配置结构：

```yaml
debug:
  show: true            # OpenCV GUI 开关（非 GUI observer 不受此控制）
  rosbag_control: true
  rosbag_player_node: "/rosbag2_player"
  preprocess: false
  lights: true
  armor_match: false
  classification: false
  pose: false           # 控制 /armor_markers 发布
  result: true
  stats_interval: 50    # 耗时统计打印间隔（帧数）

playback:
  mode: "realtime"      # "realtime" 或 "step"
  max_frames: 0         # 0=不限制，>0 时处理指定帧数后退出
  exit_on_complete: false
```

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

## 图层切换

数字键 1–6 切换对应 debug 图层的显示状态：

| 按键 | 图层 | 说明 |
|---|---|---|
| 1 | preprocess | 预处理中间图 |
| 2 | lights | 灯条检测结果 |
| 3 | armor_match | 装甲板匹配 |
| 4 | classification | 数字分类 |
| 5 | pose | /armor_markers 发布开关 |
| 6 | result | 最终识别装甲板 X 标记 |

切换后输出日志，例如：`DEBUG layer lights: ON`

## Timing 行为

- `timing` 不是图层——始终注册，无法通过数字键关闭。
- 无论 `debug.show` 是否启用，每 `debug.stats_interval` 帧都打印平均耗时日志。
- 仅当 `debug.show=true` 且 `display_bgr` 非空时，才在 GUI 上绘制 `Process: ... ms`。
- `auto_test.launch.py` 支持 `timing_interval` 参数覆盖 `debug.stats_interval`。

## 播放控制

### realtime 模式

- rosbag 按时间正常播放
- 按键 Space 暂停/继续，n 单步

### step 模式

- 处理完一帧后再请求播放下一帧
- 通过 `/rosbag2_player/play_next` service 控制
- 使用 in-flight 状态机避免请求竞争
- `auto_test.launch.py` 默认使用此模式

## 数据结构规范

- 阶段 debug 数据统一放在 `DebugData.hpp`。
- 数据结构只表达"该阶段产生了什么"，不要混入窗口布局细节。
- 拒绝原因使用枚举 `DebugRejectReason`，补充细节放到 `detail` 字符串。
- 如果字段会影响算法结果，它不应该放在 debug 数据里。
