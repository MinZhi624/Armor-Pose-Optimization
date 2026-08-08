# Armor Pose Estimation

This context covers armor-plate detections and their estimated poses.

## Language

**Pose yaw observation**:
The final estimated yaw angle of one successfully processed armor plate. Each successful armor result counts as one observation, including multiple armors detected in the same frame.
_Avoid_: frame yaw, video yaw

**Yaw coverage uniformity**:
How evenly pose yaw observations occupy a chosen angle range. Perfect coverage uniformity means equal-width angle intervals contain equal proportions of observations.
_Avoid_: yaw symmetry, yaw concentration

**Single-frame dual-armor hard-constrained joint BA（单帧双装甲板角度硬约束联合 BA）**:
A pose refinement for two adjacent armor plates observed in the same frame. It estimates one shared base pose yaw and translation parameters for both armors. Each candidate enforces an exact signed 90° yaw difference by construction; the accepted result is selected from the `+90°` and `-90°` hypotheses.
_Avoid_: EKF, multi-frame BA, full vehicle rigid-body BA, 90° soft constraint

**Shared base pose yaw（公共基础 yaw）**:
The pose yaw represented by image-left armor A in a dual-armor joint refinement. Image-right armor B has no independent pose yaw; its pose yaw is the shared base pose yaw plus the selected signed quarter-turn.

**Signed quarter-turn hypotheses（带符号四分之一圈假设）**:
The two hard-constraint candidates `normalize(theta_B - theta_A) = +pi/2` and `normalize(theta_B - theta_A) = -pi/2`. If exactly one candidate is physically usable, it is selected. If both are usable, the candidate with the lower final robust objective is selected, with `+pi/2` preferred on a tie. If neither is usable, the pair falls back atomically.

**Dual-armor 3DoF YPD joint BA（双装甲板 3DoF YPD 联合 BA）**:
The dual-armor model whose translation variables are one gimbal-frame YPD distance per armor while each armor's gimbal-frame sightline direction remains fixed at its selected PnP initialization. Together with the shared base pose yaw, it has three optimized degrees of freedom.

**Dual-armor 7DoF XYZ joint BA（双装甲板 7DoF XYZ 联合 BA）**:
The dual-armor model whose translation variables are independent gimbal-frame `x`, `y`, and `z` coordinates for each armor. Together with the shared base pose yaw, it has seven optimized degrees of freedom. Its objective contains image reprojection terms only: no PnP translation prior and no configured XYZ parameter bounds.

**Eligible dual-armor pair（可联合双板对）**:
Exactly two successfully initialized armor observations in one frame that share the same non-empty armor number and the same armor size type. A frame group with one armor or more than two armors, or a same-number pair with mismatched size types, is not an eligible pair in the first version.
The first version assumes a frame can contain at most one eligible dual-armor pair.

**Armor A / Armor B label（A/B 标签）**:
For an eligible pair, armor A is the observation with the smaller image-center x coordinate and armor B is the other observation. The ordering gives stable identities to the shared base pose yaw and diagnostics; it does not predetermine which signed quarter-turn hypothesis wins.

**Joint-refinement path（联合优化路径）**:
For an eligible pair, the dual-armor joint BA starts directly from each armor's selected PnP pose and replaces per-armor refinement. It does not consume the output of a single-armor refiner. Observations outside an eligible pair use the independently configured single-armor refinement method.

**Physically usable joint result（物理可用联合结果）**:
A joint result whose two translations and distances are finite, whose distances are strictly positive, whose reconstructed armor poses are finite, and whose armor points project validly in front of the camera. There is no configured working-range bound on distance.

**Joint mean reprojection error（联合平均重投影误差）**:
The unweighted arithmetic mean of the eight per-corner Euclidean pixel residual magnitudes across armor A and armor B. Per-armor errors and distance changes are diagnostics only and do not add residuals or acceptance thresholds.

**Atomic joint fallback（联合原子回退）**:
An eligible pair is committed only when at least one signed-quarter-turn hypothesis produces a physically usable result. Otherwise both armors retain their own selected PnP poses; the system neither commits a partial pair nor invokes a single-armor BA fallback.
