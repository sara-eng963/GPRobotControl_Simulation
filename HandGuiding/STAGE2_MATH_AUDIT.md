# Stage 2B — Targeted Mathematical Controls Hardening

**Branch:** `sara/can-replacement`. **Scope:** mathematical validation and dry-run adapter only.
This work deliberately preserves Stage 1 E062 host-cycle rephasing (`92ad454`), the mass–damper admittance law, the existing DLS solver, safety gates and Supervisor command authority. No hardware motion is authorized.

## Equations and frames

Each sensor-frame sample is tare-subtracted, rotated into flange coordinates and shifted to the flange origin by
`tau_F = R_F_S tau_S + p_F_S × (R_F_S f_S)`.
Shift from flange to TCP with `tau_TCP^F = tau_F - p_F_TCP × f_F`, then rotate both force and moment into base coordinates with `R_B_F = R_B_TCP R_F_TCP^T`.
The Jacobian and commanded twist refer to the **TCP origin, expressed in base frame**.

The diagonal zero-stiffness admittance uses backward Euler:
`v[k+1] = (M v[k] + dt w[k+1]) / (M + dt D)`.
The actual output also has per-axis acceleration and velocity limits. Linear channels use N, m/s and kg; angular channels use N·m, rad/s and kg·m². This discrete filter's stability does not prove physical interaction stability.

The mapper solves `(J^T J + lambda^2 I) qdot = J^T V_requested`,
applies existing per-joint velocity and acceleration limits, and integrates
`qref[k+1] = qref[k] + dt qdot_limited`.
Stage 2B adds **diagnostic-only** outputs:
`V_achieved = J(q_measured) qdot_limited` and
`V_residual = V_requested - V_achieved`.
Components 0–2 are m/s and 3–5 are rad/s. Do not compare their combined unweighted norm against an arbitrary threshold. `V_achieved` is a *kinematic prediction from commanded joint velocities*, not observed TCP velocity. Neither diagnostic modifies the command or authorizes physical motion.

## Small changes and why

1. **Rigid TCP transform:** Standalone `cartesian_joint_mapper_config_valid` now rejects scale, shear and reflected transforms by checking rotation orthonormality and determinant +1, consistent with `wrench_frame.c`. Finite entries and the homogeneous bottom row alone were insufficient. Valid proper rotations remain accepted.
2. **Cartesian direction visibility:** The mapper reports achieved twist and per-component residual *after* saturation. Independent analytical/NumPy-generated reference constants in `test_cartesian_joint_mapper.c` check nominal DLS and a deliberately constrained J4 case where angular direction reverses. No new runtime acceptance threshold was invented.
3. **Six-joint feedback coherence:** `HandGuidingDryRunConfig.max_feedback_skew_us` optionally rejects a copied set of feedback whose `max(timestamp_us)-min(timestamp_us)` exceeds the configured limit. All six timestamps must use the same monotonic clock. **Zero means disabled** until an empirically justified limit is specified; all existing per-joint age checks, sequence checks and safety gates remain. Test fixture sets 500 us only for boundary testing; **500 us is not an approved physical limit**.

The pipeline retains its atomic numerical-state update, fail-latched errors and fresh-feedback initialization. `supervisor_task.c` and its host-only Stage 1 rephasing are unchanged. No new task, queue, heap allocation or CAN access is added.

## Verification and reproducibility

Baseline *before* Stage 2B modifications, supplied from the user's WSL:
- Strict `-Wall -Wextra -Werror -Wpedantic` standalone build: **9/9 PASS**.
- `supervisor_silkit_virtual_ft_input`: **1/1 PASS**, 18.67 s.
These are not post-change test results.

Post-change validation commands on the updated branch:

```sh
cmake -S HandGuiding -B /tmp/gp-stage2b -DCMAKE_C_FLAGS="-Wall -Wextra -Werror -Wpedantic"
cmake --build /tmp/gp-stage2b -j2
ctest --test-dir /tmp/gp-stage2b --output-on-failure

cmake -S . -B build-can-silkit -DENABLE_SILKIT=ON -DSILKIT_ROOT="$SILKIT_ROOT"
cmake --build build-can-silkit -j2
ctest --test-dir build-can-silkit -R '^supervisor_silkit_virtual_ft_input$' --output-on-failure
```

Review post-change CTest output before declaring Batch 2B fully regression-validated.
WSL scheduling and SIL Kit motion do not establish a 500 Hz STM32 worst-case execution bound, motor response, calibrated FT mounting, real transmission latency, or physical human–robot stability.

## References

- Keemink, van der Kooij & Stienen (2018), *Admittance control for physical human–robot interaction*, https://doi.org/10.1177/0278364918768950
- Lynch & Park, *Modern Robotics*, wrench transforms and geometric Jacobians, https://modernrobotics.northwestern.edu/
- ROS 2 Admittance Controller: https://github.com/ros-controls/ros2_controllers/tree/master/admittance_controller
- MoveIt Servo (singularity handling, velocity scaling): https://github.com/moveit/moveit2/tree/main/moveit_ros/moveit_servo

**Limitations:** Hardware gains, allowable skew, saturation direction errors, TCP geometry, real drive tracking and human–robot passivity remain to be validated before physical hand-guiding.
