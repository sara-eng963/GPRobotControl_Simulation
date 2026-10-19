# Stage 4F2A — Virtual F/T Input (Host Simulation Only)

**Status:** implementation committed; validation in a full WSL SIL Kit checkout is
**pending**. No production STM32 source or physical motion permission changed.

## Boundaries and modules

```text
Python sender (4F2A) / Raylib GUI (planned 4F2B,C)
    -> UDP 127.0.0.1:5013, versioned atomic 6-axis base/TCP wrench
    -> virtual_ft_receiver (bounded nonblocking simulation input acquisition)
    -> virtual_ft_protocol (validation, sequence, watchdog; latched faults)
    -> virtual_ft_sensor (inverse rigid wrench map at measured robot q)
    -> host_guidance_demo synthetic sensor callback
    -> existing F/T validation -> sensor/flange -> base/TCP
    -> existing C admittance -> DLS/Jacobian -> command validator
    -> single M7 Supervisor -> CANopen -> SIL Kit AVATAR motors
    -> Supervisor feedback UDP :5005 -> MATLAB read-only visualization
```

The interactive mode is opt-in and mutually exclusive with the legacy
HMI Cartesian-pose input. Default is the existing constant 0.5 N synthetic
fixture. The existing 4F1 trace and timing hooks remain intact. All interactive
tuning/model parameters are synthetic test fixtures, not hardware approvals.
Interactive mode uses the unmodified shared UR5 geometry and an actual J5
home of 0.15 rad to avoid the singular wrist at zero. Teaching FK and MATLAB
therefore see the same pose as the guidance Jacobian. The older constant-force
observation fixture retains its separate perturbed model.

### Wrench and frame convention

Message components `[Fx,Fy,Fz,Tx,Ty,Tz]` represent force (N) and torque
(N·m) expressed in the **robot base frame B**, about the **TCP origin**.
Arrows refer to B, never to the MATLAB camera. A force applied on a sensor
handle is represented by an **equivalent wrench at TCP**. The host adapter
uses measured joint angles, tool offset and sensor mounting geometry to
construct the sensor-frame sample S required by the existing pipeline:

```text
R_B_F = R_B_TCP R_F_TCP^T
f_F = R_B_F^T f_B
tau_F = R_B_F^T tau_B_TCP + p_F_TCP x f_F
f_S = R_F_S^T f_F
tau_S = R_F_S^T (tau_F - p_F_S x f_F)
```

The last term is the flange-to-sensor reference shift. Do **not** add the
grip-to-sensor lever arm twice: the equivalent sensor torque already
includes that load-path moment. The existing forward conversion should
recover the original requested base/TCP wrench to numerical precision.
Static synthetic tare is zero; gravity/inertia and real mounting geometry
are not modeled here.

### UDP protocol v1

Dedicated port `127.0.0.1:5013`, separate from HMI command port 5010.
Exactly **40 bytes**, ten 32-bit big-endian words:

| Word | Meaning |
|---|---|
| 0 | Magic 0x56465431 (`VFT1`) |
| 1 | Version = 1 |
| 2 | uint32 sequence, strictly increasing modulo 2³² |
| 3 | Enabled = 1, disengaged = 0 |
| 4–6 | IEEE754 float32 Fx,Fy,Fz, N |
| 7–9 | IEEE754 float32 Tx,Ty,Tz, N·m |

Host receiver records the packet's **receive time from the same monotonic
FreeRTOS clock** used by the Supervisor. Sender timestamps are not trusted.
Default bounds are ±5 N per force component and ±1 N·m per moment
component; configurable in `GpVirtualFtLimits`. A valid ENABLED all-zero
packet is normal release. The admittance state retains its velocity and
decelerates through the existing virtual damping.

Nonfinite, oversized, truncated, range-invalid, duplicate, out-of-order
or unknown-version packets latch input failure; a timeout beyond **150 ms**
after the last valid packet latches failure. A disabled sample during
active hand-guiding also fails guidance. No stale wrench is held after
loss; existing fault path inhibits further candidates and simulated drive
dispatch. **Recovery is explicit simulation restart**, not automatic reset.
The receiver is polled in the existing simulation input task, independently
of HMI logging/CSV flushing; the Supervisor's 2 ms callback
reads only an in-memory snapshot and does no socket I/O.

Controller enable is separate from force input and measured rest. The host
reports settled only after zero wrench and 20 new complete feedback sets at
an unchanged attained integer target. This supplies Teaching's existing
`robot_motion_settled` input. The Record display also requires TEACHING and
its existing `record_allowed` output; Teaching remains the acceptance authority.

## Build and test (WSL)

```bash
cd ~/freertos-pc-test
git fetch origin sara/can-replacement
git switch sara/can-replacement
git pull --ff-only origin sara/can-replacement
export SILKIT_ROOT="$HOME/silkit_test/SilKit-5.0.7-ubuntu-24.04-x86_64-gcc/SilKit"

cmake -S . -B /tmp/gp-4f2 -DENABLE_SILKIT=ON -DSILKIT_ROOT="$SILKIT_ROOT"
cmake --build /tmp/gp-4f2 --target test_virtual_ft_protocol test_virtual_ft_sensor supervisor_silkit_sim avatar_m_silkit_motor_bank -j2
ctest --test-dir /tmp/gp-4f2 -R '^virtual_ft_(protocol|sensor_frame)$' --output-on-failure
ctest --test-dir /tmp/gp-4f2 -R '^supervisor_silkit_virtual_ft_input$' --output-on-failure
```

A small real-input sender (same protocol/receiver as the future Raylib GUI):

```bash
python3 Simulation/Tools/send_virtual_ft.py --fx 0.5 --duration 2 --zero-tail 1
```

For a manual multi-terminal run:

1. Start `$SILKIT_ROOT/bin/sil-kit-registry --listen-uri silkit://localhost:8500`.
2. Start `/tmp/gp-4f2/avatar_m_silkit_motor_bank`.
3. Start `GP_GUIDANCE_DRY_RUN=1 GP_GUIDANCE_SIM_MOTION=1 GP_GUIDANCE_VIRTUAL_FT=1 /tmp/gp-4f2/supervisor_silkit_sim`.
4. Wait until HOMING finishes and state IDLE is reached, **start the sender
   continuously with zero wrench**, then enter TEACHING through the existing
   HMI/panel. The sender can then apply force. For reproducibility use the
   integration test, which orders this startup automatically.
5. Open `MATLAB/can_robot_visualizer.m`; the normal port 5005 feedback
   remains the only MATLAB motion input.

The short standalone sender exits after its duration; while TEACHING is
still active, its termination deliberately triggers the watchdog fault.
For an uninterrupted manual run, the future GUI must stream valid zeros
even while no controls are held.

## Tests and acceptance

- `test_virtual_ft_protocol`: positive/negative/zero wrench, bounds,
  nonfinite and malformed datagrams, duplicates, disabled state, timeout.
- `test_virtual_ft_sensor_frame`: round-trip wrench with tool/sensor offsets,
  rotated sensor and two robot orientations, and invalid geometry.
- `supervisor_silkit_virtual_ft_input`: headless 50-Hz virtual sender through
  the actual 2-ms C pipeline, verify trace wrench/twist, feedback motion,
  zero-wrench decay and lost-packet Supervisor fault.
- Pre-existing 4F1 and 4E3 regressions remain registered for separate reruns.

**No test is claimed PASS until executed in WSL/CI.** No actual 500-Hz
MCU deadline, CAN bus electronics, real F/T acquisition, robot inertia,
rigidity, collision avoidance or certified E-stop/STO is validated.

## Next milestone

4F2B adds a separate Raylib executable with X± held buttons and a force
slider, emitting the same protocol. 4F2C adds all axes, torque controls
and status. MATLAB visualizer requires no command-path modification.


## Stage 4F2B — Minimal interactive Raylib GUI (2026-10-19)

**Scope:** standalone `virtual_ft_gui` executable, tested host transport,
positive/negative X force only. Full six-axis buttons and torque slider
remain Batch C. Only software simulation is in scope.

### GUI operation

- **Right arrow** or hold the **+X** button: positive X force.
- **Left arrow** or hold the **-X** button: negative X force.
- Both held simultaneously: zero force.
- Release controls: valid **zero wrench** samples remain streaming; the
  **existing C admittance controller** is responsible for velocity decay.
- Magnitude slider: 0–5 N (starts at 0.5 N). Software demonstration
  range only, not a sensor rating or physical safety limit.
- **Zero Wrench:** suppress force until held directional controls are released.
- Keyboard/window focus loss forces the transmitted wrench to zero.
- All other force/torque components are zero in Batch B.
- Directions are robot base-frame axes, about the TCP; neither button
  depends on MATLAB camera orientation.

The GUI uses the existing `hmi_theme` styles and `hmi_protocol` decoder.
The Supervisor sends a **read-only copy** of its existing 57-word status
packet to UDP **5014**, only while virtual-F/T mode is enabled.
Ports 5011 (supervisor_panel) and 5012 (teaching_sim) remain unchanged.
Status reports Disconnected, Ready/awaiting TEACHING, Guidance active,
or Fault/E-stop. Receiving this status is not a safety guarantee.

The GUI is exclusively an F/T measurement source (UDP 5013); it never
uses the legacy Cartesian-position command, HMI event output, or CANopen
interface. The existing panel can still select TEACHING. The old teaching
simulator's pose commands are ignored in virtual F/T mode.

### WSL build

```bash
cd ~/freertos-pc-test
git fetch origin sara/can-replacement
git switch sara/can-replacement
git pull --ff-only origin sara/can-replacement
export SILKIT_ROOT="$HOME/silkit_test/SilKit-5.0.7-ubuntu-24.04-x86_64-gcc/SilKit"

cmake -S . -B /tmp/gp-4f2 -DENABLE_SILKIT=ON -DSILKIT_ROOT="$SILKIT_ROOT"
cmake --build /tmp/gp-4f2 --target \
  virtual_ft_gui supervisor_panel supervisor_silkit_sim \
  avatar_m_silkit_motor_bank test_virtual_ft_protocol \
  test_virtual_ft_sensor test_virtual_ft_controls \
  test_virtual_ft_client -j2
ctest --test-dir /tmp/gp-4f2 \
  -R '^virtual_ft_(protocol|sensor_frame|controls|client_udp)$' \
  --output-on-failure
ctest --test-dir /tmp/gp-4f2 \
  -R '^supervisor_silkit_virtual_ft_input$' --output-on-failure
```

**One-command full interactive simulation (recommended):**

```bash
cd ~/freertos-pc-test
bash scripts/run_full_can_silkit_simulation.sh --virtual-ft
```

This extends the existing canonical launcher; **without** `--virtual-ft`
its previous normal teaching simulation remains unchanged. F/T mode builds
and starts the registry, six SIL Kit AVATAR motors, Supervisor with all
three opt-in guidance flags, passive CAN live monitor, `virtual_ft_gui`
and `supervisor_panel` in the correct order. It intentionally does
**not** launch `teaching_sim`, whose legacy Cartesian command interface
is not the interactive F/T source. The launcher detects the WSL Windows
host IP for MATLAB UDP :5005 and cleans up its child processes on
Ctrl-C or an interactive-GUI exit.

The launcher keeps separate F/T logs under
`.simulation-logs/virtual-ft/`, including `guidance.csv`,
`timing-metrics.csv`, `supervisor.log`, `can-frames.csv` and the GUI
log. No separate set of terminals is necessary.

For faster subsequent starts once everything has been compiled:

```bash
bash scripts/run_full_can_silkit_simulation.sh --virtual-ft --no-build
```

The incompatible combination `--virtual-ft --no-gui` is rejected: active
guidance requires a continuously transmitting GUI. MATLAB remains a
Windows-side process and must still be opened separately.

After the one-command launcher reports running and HOMING reaches IDLE, use
the started supervisor panel to select LINE / TEACHING. The GUI should show
Guidance active. Hold
Right Arrow or +X, then release. Observe a nonzero `fx` and `vx`,
evolving `q1..q6` and actual `actual_q1..actual_q6` in the existing
guidance trace. For MATLAB visualization open
`MATLAB/can_robot_visualizer.m` in MATLAB; it still receives only motor
feedback via UDP 5005. Set `MATLAB_IP` for the Windows host if required
in WSL, exactly as with existing visualization.

Do not run the automated Python F/T sender and GUI at the same time.
Both use the same input stream and conflicting sequences can latch a
protocol fault. The receiver currently locks to one monotonically
increasing sequence; after closing/restarting the GUI, **restart the
Supervisor simulator** before re-entering TEACHING. Ending GUI output
during TEACHING correctly expires the watchdog and faults simulated
guidance. No automatic fault reset is performed.

### Test boundaries and evidence

- `virtual_ft_protocol` adds a round-trip test for the shared production
  packet encoder, positive/negative/zero components and invalid values.
- `virtual_ft_controls` verifies held X+/X-, opposing cancel, release,
  invalid magnitude and disabled guidance.
- `virtual_ft_client_udp` creates an actual localhost UDP receiver and
  verifies packets from the same sender used by the GUI, with the
  existing decoder for +X, -X, and zero samples.
- `supervisor_silkit_virtual_ft_input` reuses the Batch A end-to-end
  scripted force input to exercise the real C admittance controller, gate,
  SIL Kit motor feedback, zero-wrench decay, and disconnect fault.
- GUI clicking, window focus, status rendering, and MATLAB visual motion
  require a manual interactive test; unit tests do not assert these.

**All new Batch B builds/tests and manual GUI behavior are pending WSL
execution.** Previous Batch A tests passed 3/3 in user-supplied logs.
Physical dynamics, STM32 500 Hz timing, F/T commissioning and safety
chains remain unverified.

### Batch B live fault diagnostics (follow-up)

In interactive mode the canonical launcher now mirrors state changes and
first-fault diagnostics to its **existing terminal**. No additional terminal
or GUI is required. The full trace remains at
`.simulation-logs/virtual-ft/supervisor.log`, and guidance samples in
`guidance.csv`.

The Supervisor reports its state/detail fault code, its last failed
hand-guiding stage, and the last validated virtual input's connected/
enabled/latched flags, protocol fault reason, sequence and age relative
to the 150 ms watchdog. The fault categories distinguish no input,
unusable feedback, sensor-frame conversion and downstream pipeline
rejection; they are diagnostics only and do not reset or bypass faults.

**For example:** `0x3E061` means the TEACHING Supervisor failed to obtain
a fresh guidance candidate (E061). The sensor diagnostic snapshot is
needed to tell whether this originated in the F/T link versus another
hand-guiding gate. Diagnostic printf and logfile tailing run at
lower priority, never in the time-critical control callback.

**Verification pending WSL:** Re-run the existing Batch A integration
test and protocol tests, then an interactive GUI session. Prior
successful 3/3 tests preceded these diagnostic changes.


### Follow-up: identify the rejected controller stage

When the first live GUI session produced Supervisor fault `0x3E061`,
input was fresh (`20 ms` age, `150 ms` watchdog), valid and enabled.
The earlier diagnostic label `ft_processing_admittance_or_mapper` could not
identify the rejecting numerical stage.

On a failed simulation-only guidance cycle, the host callback now saves the
**pre-failure pipeline state**, and only if the cycle fails performs a
**non-committing replay on copies** of the admittance and mapper states. It
uses the existing numerical implementations. The resulting first-fault
console message identifies whether sensor-to-base wrench conversion,
admittance, or mapping failed; for mapping, it separately reports Jacobian
singularity, tracking error and joint-limit checks, as well as sigma minimum.
This does not retune gains, disable limits, reset a fault, or issue a second
motor command. Diagnostics may be less specific for other mapper failures,
which are labeled `mapper_dls_or_other_constraint`.

**Validation pending WSL execution:** Rebuild and retry with the existing
`--virtual-ft` one-command launcher; if failure repeats, capture the new
`[FAULT]` lines. Rerun `supervisor_silkit_virtual_ft_input` and existing
unit/regression tests after this host-only diagnostic change.


### Suspected intermittent E061: mismatched sample timestamps (fixed in host adapter)

Source audit after the first-fault report found a specific time-domain issue:
`service_guidance_dry_run()` captures `cycle.now_us` before invoking the
callback, but the interactive `synthetic_ft()` was taking a **second**
`xTaskGetTickCount()` reading later in that same cycle and stamping the
simulated sensor sample with it. Crossing an RTOS tick boundary can make the
sample timestamp *later than* the Supervisor's `input.now_us`.
`ft_sensor_sample_usable()` uses a modulo-32-bit unsigned sample age,
so this appears as an extremely old sample, failing the existing **30 ms
sample freshness check** and producing `0x3E061` despite healthy UDP input.

The virtual sensor adapter now uses the already-captured Supervisor
`cycle.now_us` for its synthetic sensor timestamp and receiver read.
The underlying sample-age validation, UDP freshness watchdog, command
validator, safety gating, control cycle and mathematical controller are
**unchanged**. This repair is restricted to the host-only interactive mode.
The fault mechanism is proven possible from source inspection, but whether
it was the sole cause of the observed GUI fault requires rerunning WSL.

The follow-up stage classifier remains available to identify any other
F/T/wrench/admittance/mapper failure if a fault occurs again.


### Batch 4F2 — E062 command gate diagnostic (read-only)

After correcting a time-of-sample inconsistency, the interactive GUI reached
TEACHING and next entered `0x3E062`. Unlike `E061`, this reports failure
during the Supervisor's **existing** candidate validator / drive readiness /
CANopen target dispatch (not a virtual F/T packet/watchdog failure).

New host-only telemetry snapshots the latest two *generated* candidates and
their feedback. Only when a fault is observed, the low-priority HMI task
re-validates the last candidate with a **fresh independent temporary gate**
and displays maximum candidate-to-feedback error, candidate step difference,
future feedback timestamps, and configured synthetic gate thresholds.
This cannot determine the live gate's private historical state, so a
PASS in the independent probe does not establish that the live gate passed.
It also cannot distinguish CAN send failure from readiness failure.
It never calls send-targets, never resets a fault, never alters StateMachine
source or firmware, and never changes thresholds.

**Verification pending WSL:** rebuild with the existing single-command
launcher; inspect `[FAULT] E062 read-only candidate audit` lines. Then
run the focused hand-guiding and SIL Kit regression tests.
