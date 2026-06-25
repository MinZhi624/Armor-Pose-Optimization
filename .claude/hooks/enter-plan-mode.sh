#!/bin/bash
# PostToolUse hook: 进入计划模式
# 创建标记文件，启用只读守卫

touch /tmp/claude-plan-mode
echo "📋 已进入计划模式：只读不写，研究代码后输出方案"
