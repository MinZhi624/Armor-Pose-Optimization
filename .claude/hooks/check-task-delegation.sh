#!/bin/bash
# PreToolUse hook: 任务分工检查
# 如果应该派子 agent，阻止写入并提示

# 获取当前修改的文件路径
FILE="$CLAUDE_FILE_PATH"
TOOL="$CLAUDE_TOOL_NAME"

# 如果没有文件路径，跳过检查
if [ -z "$FILE" ]; then
    exit 0
fi

# 状态文件记录本次会话修改的文件
STATE_FILE="/tmp/claude-task-delegation-count"
touch "$STATE_FILE"

# 读取当前已修改的文件数
CURRENT_COUNT=$(wc -l < "$STATE_FILE" 2>/dev/null || echo "0")

# 检查当前文件是否已经在列表中
if ! grep -qF "$FILE" "$STATE_FILE" 2>/dev/null; then
    # 新文件，添加到列表
    echo "$FILE" >> "$STATE_FILE"
    CURRENT_COUNT=$((CURRENT_COUNT + 1))
fi

# 如果修改超过 1 个文件，建议派子 agent（警告而非阻断）
if [ "$CURRENT_COUNT" -gt 1 ]; then
    echo "⚠️  任务分工建议：已修改 $CURRENT_COUNT 个文件"
    echo "   根据 workflow.md，多文件修改建议派子 agent (model: haiku)"
    echo "   当前文件: $FILE"
    echo "   已修改文件:"
    cat "$STATE_FILE" | sed 's/^/     - /'
    echo ""
    echo "   如需重新计数: rm -f $STATE_FILE"
    # 不 exit 2，让用户自己决定
fi

exit 0
