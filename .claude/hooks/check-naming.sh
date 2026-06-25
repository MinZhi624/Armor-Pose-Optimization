#!/bin/bash
# Pre-Commit hook: 按照 workflow.md 验收流程检查
# 1. 编译是否通过
# 2. grep 确认无残留引用

FILE="$CLAUDE_FILE_PATH"
ISSUES=""

# 只检查 C++ 文件
if [[ ! "$FILE" =~ \.(cpp|hpp|h|cc)$ ]]; then
    exit 0
fi

# === 命名规范检查 ===

# 检查类名是否 PascalCase
if grep -Pn '^\s*(class|struct)\s+[a-z]' "$FILE" 2>/dev/null; then
    ISSUES+="❌ 类名应使用 PascalCase\n"
fi

# 检查日志格式 (不应该使用 %d %s 等)
if grep -Pn '%[dsf]' "$FILE" 2>/dev/null | grep -v '//' | grep -v '#'; then
    ISSUES+="⚠️ 日志应使用 {} 占位符，不要用 %d/%s\n"
fi

# === 验收流程检查 ===

# 检查是否有 TODO/FIXME/HACK 残留
TODOS=$(grep -Pn '\b(TODO|FIXME|HACK|XXX)\b' "$FILE" 2>/dev/null)
if [ -n "$TODOS" ]; then
    ISSUES+="⚠️ 发现待处理标记:\n$TODOS\n"
fi

# 检查是否有调试代码残留 (cout/cerr 但不是日志)
DEBUGS=$(grep -Pn '\b(std::cout|std::cerr|printf|fprintf)\b' "$FILE" 2>/dev/null | grep -v '//.*cout' | grep -v 'logging')
if [ -n "$DEBUGS" ]; then
    ISSUES+="⚠️ 可能有调试代码残留:\n$DEBUGS\n"
fi

# 如果有问题，输出并返回非零
if [ -n "$ISSUES" ]; then
    echo -e "Workflow 验收检查:\n$ISSUES"
    exit 1
fi

exit 0
