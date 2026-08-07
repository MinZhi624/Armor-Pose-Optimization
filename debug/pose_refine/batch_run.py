#!/usr/bin/env python3
"""
批量 pose_refine 实验: 遍历所有角点修正 × 位姿优化方法组合。

用法:
  python3 batch_run.py [video] [frame_count]

默认: video=video7, frame_count=150

流程:
  1. 备份 config.yaml
  2. 遍历 corner_method × refine_method 全部 16 种组合
  3. 每次修改 config.yaml 的 detector.corner_correction.method，直接 ros2 launch
  4. 恢复 config.yaml
  5. 输出汇总

CSV 输出:
  debug/pose_refine/log/<video>/<corner_method>/<refine_method>/<timestamp>.csv
"""

import os
import shutil
import subprocess
import sys

import yaml

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CONFIG_PATH = os.path.join(REPO_ROOT, "src", "armor_detector", "config", "config.yaml")
CONFIG_BACKUP = CONFIG_PATH + ".batch_run.bak"

CORNER_METHODS = ["none", "fit_ellipse", "min_area_rect", "pca_gradient"]
REFINE_METHODS = ["none", "yaw_search", "pose_only_ba_6dof", "pose_only_ba_4dof"]


def set_corner_method(method: str) -> None:
    with open(CONFIG_PATH) as f:
        data = yaml.safe_load(f)
    node_cfg = data.get("armor_detector_node_cpp", data)
    params = node_cfg.get("ros__parameters", node_cfg)
    params.setdefault("detector", {})
    params["detector"].setdefault("corner_correction", {})
    params["detector"]["corner_correction"]["method"] = method
    with open(CONFIG_PATH, "w") as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
    print(f"  config: corner_correction.method = {method}")


def main() -> None:
    video = sys.argv[1] if len(sys.argv) > 1 else "video7"
    frame_count = sys.argv[2] if len(sys.argv) > 2 else "150"

    if not os.path.exists(CONFIG_PATH):
        print(f"错误: 找不到 {CONFIG_PATH}")
        sys.exit(1)

    shutil.copy2(CONFIG_PATH, CONFIG_BACKUP)
    print(f"备份: {CONFIG_BACKUP}")

    results = []
    try:
        for cm in CORNER_METHODS:
            set_corner_method(cm)
            for rm in REFINE_METHODS:
                print(f"\n{'=' * 60}")
                print(f"  运行: {cm:15s} \u00d7 {rm:25s}  video={video}  frames={frame_count}")
                print(f"{'=' * 60}")
                cmd = [
                    "ros2", "launch", "armor_detector", "pose_refine_experiment.launch.py",
                    f"video:={video}", f"refine_method:={rm}",
                    f"frame_count:={frame_count}", "use_foxglove:=false",
                ]
                result = subprocess.run(cmd, cwd=REPO_ROOT)
                ok = result.returncode == 0
                print(f"  {'OK' if ok else 'FAIL(exit=' + str(result.returncode) + ')'}")
                results.append((cm, rm, ok))
    finally:
        shutil.move(CONFIG_BACKUP, CONFIG_PATH)
        print(f"\n已恢复: {CONFIG_PATH}")

    print(f"\n{'=' * 60}")
    print("  汇总")
    print(f"{'=' * 60}")
    for cm, rm, ok in results:
        print(f"  {'OK' if ok else 'FAIL'}  {cm:15s} \u00d7 {rm:25s}")
    print(f"\n  通过: {sum(1 for _, _, ok in results if ok)}/{len(results)}")


if __name__ == "__main__":
    main()
