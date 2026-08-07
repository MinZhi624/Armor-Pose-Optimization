#!/usr/bin/env bash
# Video8 pose_refine 实验: pca + 所有方法 + none/none
# 用法: bash debug/pose_refine/run_video8_experiments.sh [start_index]
#   start_index: 从第几个实验开始（1-based，默认 1）

set -e

REPO_ROOT="/home/minzhi/Desktop/ws_homework1"
CONFIG="${REPO_ROOT}/src/armor_detector/config/config.yaml"
CONFIG_BAK="${CONFIG}.experiment.bak"
VIDEO="video8"
FRAME_COUNT="0"

START=${1:-1}

# 实验列表: tag corner_method single_method dual_method
EXPERIMENTS=(
  "1|pca+none+dual=none|pca_gradient|none|none"
  "2|pca+yaw_search+dual=none|pca_gradient|yaw_search|none"
  "3|pca+yaw_search_then_distance+dual=none|pca_gradient|yaw_search_then_distance|none"
  "4|pca+ba_6dof+dual=none|pca_gradient|pose_only_ba_6dof|none"
  "5|pca+ba_4dof_xyz+dual=none|pca_gradient|pose_only_ba_4dof_xyz|none"
  "6|pca+ba_4dof_ypd+dual=none|pca_gradient|pose_only_ba_4dof_ypd|none"
  "7|pca+yaw_search_then_distance+dual_ba|pca_gradient|yaw_search_then_distance|dual_armor_ba_3dof_ypd"
  "8|none+none+none|none|none|none"
)

TOTAL=${#EXPERIMENTS[@]}

# 首次运行备份
if [ ! -f "$CONFIG_BAK" ]; then
  cp "$CONFIG" "$CONFIG_BAK"
  echo "备份: $CONFIG_BAK"
fi

for EXP_RAW in "${EXPERIMENTS[@]}"; do
  IFS='|' read -r IDX TAG CORNER SINGLE DUAL <<< "$EXP_RAW"
  if [ "$IDX" -lt "$START" ]; then
    continue
  fi

  echo ""
  echo "============================================================"
  echo "  [$IDX/$TOTAL] $TAG"
  echo "  corner=${CORNER}  single=${SINGLE}  dual=${DUAL}"
  echo "============================================================"

  # 修改 config.yaml
  sed -i "s/        method: \".*\"/        method: \"${CORNER}\"/" "$CONFIG"
  sed -i "s/      single_refine_method: \".*\"/      single_refine_method: \"${SINGLE}\"/" "$CONFIG"
  sed -i "s/      dual_refine_method: \".*\"/      dual_refine_method: \"${DUAL}\"/" "$CONFIG"

  echo "  config updated"

  # 运行
  ros2 launch armor_detector pose_refine_experiment.launch.py \
    video:=${VIDEO} frame_count:=${FRAME_COUNT} use_foxglove:=false
  RC=$?
  if [ $RC -ne 0 ]; then
    echo "  [警告] 退出码=$RC"
  fi
  echo "  [完成] $TAG"
done

# 恢复原始 config
cp "$CONFIG_BAK" "$CONFIG"
echo ""
echo "配置已恢复"

# 验证 CSV
echo ""
echo "====== CSV 输出验证 ======"
find "${REPO_ROOT}/debug/pose_refine/log/${VIDEO}" -name "*.csv" | sort
echo "========================"
echo "全部完成"
