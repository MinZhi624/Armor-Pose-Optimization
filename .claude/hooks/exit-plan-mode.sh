#!/bin/bash
# PostToolUse hook: 退出计划模式
# 删除标记文件，允许写操作

rm -f /tmp/claude-plan-mode
echo "✅ 已退出计划模式：可以执行代码修改"
