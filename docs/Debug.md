# Debug 扩展规范

本文档按 `Core-Ball-Vision` 当前实现记录 debug 模块的扩展边界。它不是 ROS2、检测后端或算法阶段的规范；当前工程只有桌面截图、帧处理和 OpenCV 调试显示。

## 总体原则

- `src/main.cpp` 负责帧消费、状态切换和事件分发，不直接调用 `cv::imshow`、`cv::waitKey` 或窗口销毁 API。
- Debug 功能通过 `IDebugObserver` 扩展，由 `DebugHub` 统一分发帧事件和按键事件。
- 所有 OpenCV HighGUI 调用只能放在 `DebugGUI` 的 GUI 线程中。
- Observer 应把原始图像 `DebugFrameContext::source_bgr` 视为只读；需要叠加可视化时修改 `display_bgr`。
- 新增独立的统计、绘制或日志功能时，优先新增 `DebugXXX` Observer，不要把实现塞入主循环。

## 当前数据流

```text
WindowCapture 生产线程
    -> ThreadSafeQueue<type::Frame>（容量 1，仅保留最新帧）
    -> main 消费帧
    -> DebugFrameContext{source_bgr, display_bgr}
    -> DebugHub::onFrameStart()
    -> DebugHub::onFrameEnd()
    -> DebugGUI::setFrame("RESULT", display_bgr, 0.5)
    -> DebugGUI GUI 线程显示
```

`source_bgr` 与采集帧共享 OpenCV 图像数据；`display_bgr` 在主循环中由 `frame.image.clone()` 创建，供 Observer 绘制。`DebugGUI::setFrame()` 会再次 clone，因此 GUI 线程不依赖下一帧的图像生命周期。

## 代码位置

```text
include/debug/DebugData.hpp        帧上下文、图层和按键事件类型
include/debug/IDebugObserver.hpp   Observer 默认空实现接口
include/debug/DebugHub.hpp         Observer 注册和同步分发
include/debug/DebugGUI.hpp         GUI 线程与帧/按键队列接口
include/debug/DebugKeyHandler.hpp  原始按键转换
include/debug/DebugTiming.hpp      帧处理耗时统计
src/debug/                          DebugGUI、按键和 Timing 实现
```

## 事件与数据

当前 `IDebugObserver` 只定义以下阶段级事件：

```text
onFrameStart(DebugFrameContext&)   主循环取得新帧后、处理开始前
onFrameEnd(DebugFrameContext&)     当前帧处理完成后、提交 RESULT 前
onKey(DebugKeyEvent const&)        主循环从 GUI 取到并翻译按键后
```

不要在像素循环或每个候选目标的内层循环中增加事件分发。若后续算法确实需要新的阶段事件，应先在 `DebugData.hpp` 定义清楚事件输入，再为接口和 `DebugHub` 同步增加该事件。

`DebugFrameContext` 当前字段如下：

| 字段 | 含义 |
|---|---|
| `frame_index` | 已消费帧的递增编号 |
| `timestamp` | 生产线程完成截图和像素转换后的 `steady_clock` 时间戳 |
| `source_bgr` | 原始 BGR 图像，只读输入 |
| `display_bgr` | 可绘制的 BGR 显示副本 |

## 新增 Observer

1. 在 `include/debug/` 和 `src/debug/` 新增 `DebugXXX`。
2. 继承 `IDebugObserver`，只重写所需事件。
3. 绘图时修改 `context.display_bgr`；需要独立窗口时通过 `DebugGUI::setFrame()` 提交，不能自行调用 HighGUI。
4. 在 `src/main.cpp` 创建 `DebugHub` 后注册 Observer。
5. 若新增窗口名、叠加文字或图形语义，同步更新 `DebugGUI.md`。

当前唯一 Observer 是 `DebugTiming(50)`：

- `onFrameStart` 记录当前帧开始时间；若时间戳未设置则补为当前时间。
- `onFrameEnd` 计算处理耗时，在 `display_bgr` 左上绘制耗时，并每 50 帧输出平均耗时和 FPS 日志。

## 按键与主循环状态

GUI 线程只采集原始按键；`DebugKeyHandler` 将其转为 `DebugKeyEvent`，主线程再处理并调用 `DebugHub::onKey()`。

| 原始按键 | 动作 | 主循环行为 |
|---|---|---|
| `Esc`、`q`、`Q` | `EXIT` | 结束主循环 |
| Space、`p`、`P` | `PAUSE_TOGGLE` | 暂停或恢复帧消费、debug 处理和显示更新 |
| `0` | `TOGGLE_LAYER(RESULT)` | 切换 RESULT 窗口；关闭时清除窗口 |

暂停不停止 `WindowCapture` 生产线程。队列只保存一帧，恢复后处理的是最新桌面画面；暂停循环仍轮询按键，因此 `Q` 必须保持可用。

## 线程边界

- `DebugHub` 和 Observer 回调只由主线程调用；不要让 GUI 线程调用 Observer。
- `DebugGUI` 自己持有一个线程，负责窗口显示、`waitKeyEx(1)` 和 `destroyAllWindows()`。
- `WindowCapture` 的 X11 截图由独立生产线程完成。不要在 debug 代码中直接访问其 X11 资源。
- 程序退出时先 `debug_gui.stop()`，再 `capture.close()`，避免 GUI 或抓取线程遗留运行状态。
