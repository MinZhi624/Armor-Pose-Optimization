# DebugGUI 显示注册规范

本文档按 `Core-Ball-Vision` 当前 `debug::DebugGUI` 实现记录 GUI 使用规则。`DebugGUI` 是 OpenCV HighGUI 的唯一出口，并通过独立线程隔离窗口显示与主线程帧处理。

## 总体原则

- 只有 `DebugGUI::loop()` 可以调用 `cv::imshow`、`cv::waitKeyEx`、`cv::destroyWindow` 和 `cv::destroyAllWindows`。
- 主线程或 Observer 通过 `setFrame()` 提交图像，通过 `takeKeyEvents()` 获取已翻译的按键事件。
- OpenCV 图像颜色使用 BGR 顺序。
- GUI 只显示和收集输入，不参与截图、算法判断或主循环状态修改。
- 新增窗口、文字或图形显示时，必须在本文档登记名称、内容和显示条件。

## 生命周期与接口

```cpp
debug::DebugGUI debug_gui;
debug_gui.start();

debug_gui.setFrame("RESULT", image_bgr, 0.5);
for (const auto &event : debug_gui.takeKeyEvents()) {
    // main 处理事件
}

debug_gui.stop();
```

| 接口 | 行为 |
|---|---|
| `start()` | GUI 启用且尚未运行时创建 GUI 线程 |
| `stop()` | 停止并 join GUI 线程，清空待显示帧、按键和待销毁窗口 |
| `setFrame(name, image, scale)` | clone 图像，可选缩放后替换同名窗口的最新帧 |
| `clearFrame(name)` | 移除同名帧，并让 GUI 线程销毁已显示窗口 |
| `takeKeyEvents()` | 原子地取走并清空按键队列 |
| `setEnabled(false)` | 禁止后续 GUI 启动和帧提交；不替代已运行实例的 `stop()` |

`setFrame()` 拒绝空图像或空窗口名。它会 clone 输入图像，因此调用者可以在提交后继续修改或释放原始 `cv::Mat`。

## 线程与队列行为

GUI 线程每轮执行以下步骤：

1. 销毁 `clearFrame()` 登记的窗口。
2. 在锁保护下复制当前窗口帧表的快照。
3. 对快照中的每个窗口调用 `cv::imshow`。
4. 通过 `cv::waitKeyEx(1)` 读取一个按键，翻译后放入按键队列。

帧表按窗口名保存一张最新图像，不积压历史帧。按键队列最多保存 16 个事件；满时丢弃最早事件。主线程应每轮调用 `takeKeyEvents()`，尤其暂停时也不能停止读取，否则 `Q` 无法及时退出。

## 当前窗口注册表

当前主循环只提交一个窗口：

| 窗口名 | 图像来源 | 缩放 | 显示条件 | 说明 |
|---|---|---:|---|---|
| `RESULT` | `DebugFrameContext::display_bgr` | `0.5` | RESULT 图层开启且当前帧已处理 | 最终调试画面，包含 `DebugTiming` 的耗时文字 |

按 `0` 会切换 RESULT 图层。关闭时主线程调用 `clearFrame("RESULT")`，GUI 线程随后销毁该窗口；重新开启后，下一帧提交时会再次显示。

## 文字与图形登记

当前实现只注册 `DebugTiming` 的文字叠加：

| 窗口 | 位置 | 内容 | 颜色(BGR) | 样式 | 条件 |
|---|---|---|---|---|---|
| `RESULT` | 左上锚点 `(10, 24)` | 当前帧 `Process: <ms> ms` | `(0, 165, 255)` | `FONT_HERSHEY_SIMPLEX`，scale `0.6`，thickness `2`，`LINE_AA` | `display_bgr` 非空 |

新增显示元素时：

- 在表中登记窗口名、元素语义、BGR 颜色、样式和显示条件。
- 不要在 Observer 中直接使用 `imshow` 或 `waitKey`。
- 同一语义应保持稳定颜色；需要多个图层时优先在 `DebugLayer` 中声明，并在主循环中实现切换行为。
- 若新窗口需要跨线程长期持有图像，继续通过 `setFrame()` 提交，不能保存调用方 `cv::Mat` 的浅拷贝引用。

## 按键归属

`DebugGUI` 只负责将 `waitKeyEx` 的原始值交给 `DebugKeyHandler`。当前翻译规则如下：

| 按键 | `DebugKeyAction` |
|---|---|
| `Esc`、`q`、`Q` | `EXIT` |
| Space、`p`、`P` | `PAUSE_TOGGLE` |
| `0` | `TOGGLE_LAYER`，图层为 `RESULT` |

动作解释、暂停语义和退出顺序由主循环及 `Debug.md` 定义，不应放入 GUI 线程。
