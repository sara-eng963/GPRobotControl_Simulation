# Corrected unified simulation: safe WSL installation

This ZIP supersedes robot-delivery.zip. It contains ONE complete implementation
patch against sara/can-replacement commit
09f45623f3accb6c4099b7b6deb9f62874909878, including the retained CM7/CM4 integration
and the launcher corrections. Do not apply the old delivery or any review archive.

## 1. Preserve the existing checkout

The supplied installer creates a separate, independent checkout and applies the
patch THERE. It never switches branches, stashes, resets, cleans, stages, commits,
or overwrites files in your existing repository. It refuses an existing or nested
destination. Conflicting patches fail without touching your original checkout.

**Uncommitted/staged/untracked local work stays only in the original checkout. It
is not silently merged, copied, or tested in the new checkout.** Keep that checkout
intact; this installation does not require deleting or reverting anything.

In WSL, set the first two paths to your actual repository and downloaded ZIP:

```bash
sudo apt-get update
sudo apt-get install -y python3 git unzip
ORIGINAL_REPO="$HOME/path/to/GPRobotControl_Simulation"
CORRECTED_ZIP="$HOME/Downloads/robot-delivery-corrected.zip"
DELIVERY_DIR="$(mktemp -d "$HOME/robot-corrected-delivery.XXXXXXXX")"
unzip "$CORRECTED_ZIP" -d "$DELIVERY_DIR"
cd "$DELIVERY_DIR"
sha256sum -c SHA256SUMS
INSTALL_ROOT="$(mktemp -d "$HOME/robot-corrected.XXXXXXXX")"
NEW_REPO="$INSTALL_ROOT/GPRobotControl_Simulation"
python3 "$DELIVERY_DIR/install_robot_delivery.py" \
  --repo "$ORIGINAL_REPO" --destination "$NEW_REPO"
cd "$NEW_REPO"
git status --short --branch
```

The installer selects the existing repository's committed local
sara/can-replacement branch, requires the base commit in its history, creates an
independent clone on the same branch, and runs git apply --check before applying.
The new changes remain uncommitted and unpushed. If your committed branch has
conflicting changes, installation refuses; do not use reset/clean/force to bypass
that refusal. The new clone is retained for inspection and your original is safe.

Use the Linux filesystem under $HOME for this checkout. The source repository's
modified FreeRTOS dependency or other uncommitted dependencies are not copied.

## 2. Dependencies (Ubuntu 24.04, WSLg, Windows MATLAB)

Keep the shell variables above in this WSL terminal:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake python3 curl unzip pkg-config \
  libgl1-mesa-dev libx11-dev libxrandr-dev libxi-dev libxcursor-dev \
  libxinerama-dev gcc-arm-none-eabi libnewlib-arm-none-eabi
git submodule update --init --recursive
git clone --depth 1 --branch V11.1.0 \
  https://github.com/FreeRTOS/FreeRTOS-Kernel.git "$INSTALL_ROOT/FreeRTOS-Kernel"
mkdir "$INSTALL_ROOT/tools"
curl -fL https://github.com/vectorgrp/sil-kit/releases/download/v5.0.7/SilKit-5.0.7-ubuntu-24.04-x86_64-gcc.zip \
  -o "$INSTALL_ROOT/tools/silkit.zip"
unzip -q "$INSTALL_ROOT/tools/silkit.zip" -d "$INSTALL_ROOT/tools"
export SILKIT_ROOT="$INSTALL_ROOT/tools/SilKit-5.0.7-ubuntu-24.04-x86_64-gcc/SilKit"
curl -fL https://github.com/renode/renode/releases/download/v1.17.0/renode-1.17.0.linux-portable.tar.gz \
  -o "$INSTALL_ROOT/tools/renode.tar.gz"
tar -xzf "$INSTALL_ROOT/tools/renode.tar.gz" -C "$INSTALL_ROOT/tools"
export PATH="$INSTALL_ROOT/tools/renode_1.17.0-portable:$PATH"
export BUILD_DIR="$NEW_REPO/build/robot-simulation"
```

CMake must be 3.28 or newer. If you already have a working SIL Kit/Renode install,
export its paths instead of downloading another copy. No installer step overwrites
your existing SDK or existing FreeRTOS checkout.

MATLAB stays independent by default, as requested. Open your existing Windows
MATLAB project and run its visualizer as you already do. Its model/helpers are
not replaced, and the WSL launcher does not start or close that session.

Only if you choose optional `--matlab` autostart, check Windows interop:

```bash
command -v wslpath powershell.exe taskkill.exe
```

The optional helper discovers matlab.exe on PATH or under
/mnt/*/Program Files/MATLAB/R*. To select a version, set its WSL path:

```bash
export MATLAB_EXE='/mnt/c/Program Files/MATLAB/R2025b/bin/matlab.exe'
export MATLAB_STARTUP_TIMEOUT=120
```

The optional helper runs the repository visualizer with its existing model/helper
requirements unchanged. Your independent MATLAB visualizer need not use this
helper. Missing licensing/toolboxes/models, occupied UDP 5005, graphics exceptions,
PowerShell failures and timeouts abort `--matlab` with logs rather than pretending
startup succeeded. It starts a dedicated process and never attaches to or clears
an existing MATLAB session.

Set MATLAB_IP to your Windows host if the WSL default-route gateway is unsuitable
(for mirrored networking it may be 127.0.0.1). Allow MATLAB inbound UDP 5005 in the
Windows firewall. Automatic startup does not modify firewall rules.

## 3. Automated verification (nonzero force and real replay)

Close other simulation instances and sender GUIs first, then run:

```bash
./scripts/run_robot_simulation.sh --verify --output "$INSTALL_ROOT/verification"
```

--verify configures and builds ALL targets, checks that the required integration,
launcher/MATLAB/installer contracts and five Renode regressions are registered,
runs EVERY registered CTest without exclusions, and runs the complete P05
acceptance runner (mutations, ARM builds, evidence gates). Missing dependencies,
missing required tests, failed tests and NOT RUN checks cannot produce PASS.
The output directory must not already exist and must be outside the source tree.
Results, exact commands and raw logs are retained there. Native Windows MATLAB
rendering is not claimed by this software gate: it exercises MATLAB supervision
contracts and actual UDP telemetry; the native readiness handshake runs only with optional `--matlab` below.

Known legacy cloud failures remain enabled and cause --verify to exit nonzero.
They are recorded in VERIFICATION.md. Read summary.json and logs; do not treat a
failed verification as accepted. Do not suppress tests to turn the result green.

## 4. Optional startup SMOKE ONLY

```bash
./scripts/run_robot_simulation.sh --headless --duration 15
```

This starts a zero-wrench source and checks participant startup/liveness only.
It does not teach, apply force, validate, replay, check MATLAB graphics or claim
end-to-end acceptance. --verify and --headless are deliberately incompatible.

## 5. Normal interactive run with your independent MATLAB project

Open Windows MATLAB and run your own visualizer independently, as usual. Confirm
WSLg is available for the Linux operator/F/T windows:

```bash
printf 'WSLg DISPLAY=%s\n' "$DISPLAY"
./scripts/run_robot_simulation.sh
```

No additional MATLAB instance is started. The launcher starts registry, six
motors, production CM7/CM4 runtime, operator panel, virtual F/T GUI and CAN monitor.
Watch your existing MATLAB visualizer follow actual joint feedback. Wait for
IDLE, select LINE, apply/release force, record distinct settled points,
validate/preview, run production, and exercise pause/resume. E-stop remains
latched: restart the whole simulation for a new simulated authorization session.

Ctrl-C stops owned simulation participants; your independently opened MATLAB
session remains alone. Logs are under $BUILD_DIR/logs/virtual-ft.

Optional supervised WSL autostart (not needed for your normal MATLAB workflow):

```bash
./scripts/run_robot_simulation.sh --matlab
```

This option waits for UDP binding and successful figure creation before starting
simulation participants. It gets a unique windows-matlab-* log directory with
powershell.log, matlab.log and readiness/error markers. Startup failure aborts
rather than silently falling back. Closing its owned visualizer ends that
simulation session. Ctrl-C first requests graceful closure of its dedicated
MATLAB instance; if necessary, cleanup targets only the PID obtained from its own
Windows Start-Process, never taskkill /IM matlab.exe. `--no-matlab` explicitly
selects the default independent/manual behavior.

The physical firmware and motion-denial boundary remain unchanged. Nothing is
committed or pushed by these commands.
