---
name: plan-execute-workflow
description: 当任务已经有明确 Plan，需要由 Execute 保持主控并忠实实施，按需调用 Worker、Explore、Scout 和 Review，最后统一审查与验证时使用。
compatibility: opencode
metadata:
  workflow: plan-execute
  language: zh-CN
---

# Plan → Execute 工作流

## 适用场景

- 已经存在明确 Plan；
- 用户要求严格按既定方案实施；
- 弱 Build 可能在实现过程中误解或偏离关键细节。

普通、无需正式 Plan 的任务继续使用 Build。

## 主流程

```text
Plan → Execute
          ├─ 自己执行
          ├─ worker-routine：D1-D2 局部代码修改
          ├─ worker-expert：D3-D5 局部代码修改
          ├─ Explore：补充本地事实
          ├─ Scout：补充外部事实
          └─ Review：并行执行 Standards 与 Spec 审查
```

Execute 始终保持主控权，并对最终实现、审查和验证负责。

## Plan 交接

Plan 默认通过当前会话上下文交给 Execute；复杂、长期、跨会话或需要稳定记录时，可落盘到：

```text
spec/plan/<task-name>.md
```

Plan 至少应包含：目标、核心方案、修改范围、关键约束、非目标和验证标准。

## 动态阶段

Execute 根据实际任务自行划分和调整阶段。阶段只是执行地图，不是第二份 Plan。

每个阶段动态决定：

- Execute 自己完成；
- 调用 Worker；
- 或混合完成。

判断依据是歧义、关键语义、上下文连续性和委派收益，而不是代码量。

## 子 Agent 边界

- `worker-routine`：只完成无需重新规划的 D1-D2 局部代码修改。
- `worker-expert`：只完成无需重新规划、且已由 Execute 确定接口与语义的 D3-D5 局部代码修改。
- Explore：只补充本地代码事实。
- Scout：只补充外部文档、依赖和上游事实。
- Review：只读检查一个指定轴，不修改文件、不运行测试、不提交。

子 Agent 不拥有整体任务，也不替 Execute 作最终决策。

## Worker 任务合同

Execute 委派 Worker 时必须提供以下字段：

```text
objective
difficulty
allowed_files
forbidden_changes
test_seam
targeted_commands
done_when
```

Worker 只能在 `allowed_files` 内工作。需要扩大范围、新设计、公共接口变化或改变核心语义时，必须返回 `BLOCKED`。Worker 不运行完整测试套件、不提交、不推送，也不调用其他 Agent。

## 审查与偏离

Execute 必须检查子 Agent 的实际修改，不能只依赖总结。

- 小问题：直接修正；
- 明显误解：缩小任务重新委派或自己接管；
- 修改越界：恢复到允许范围；
- Plan 核心假设与代码现实冲突：停止并明确报告，不静默改方案。

## 最终验证

Worker 可以运行父 Agent 明确授权的定向命令，但不运行完整测试套件。最终的 diff 审查、编译、测试、运行时检查、基准以及文档一致性验证均由 Execute 统一负责。

只有 Plan 目标实现、必要验证通过且风险明确后，任务才算完成。
