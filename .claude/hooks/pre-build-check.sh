#!/bin/bash
# Pre-Commit hook: 按照 workflow.md 验收流程
# 在提交前自动运行编译检查

SCRIPT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"

echo "=== Workflow 验收: 编译检查 ==="

# 运行编译
cd "$SCRIPT_DIR"
if ! ./build.sh 2>&1; then
    echo "❌ 编译失败，请修复后再提交"
    exit 2  # 返回 2 会阻断提交
fi

echo "✅ 编译通过"
exit 0
