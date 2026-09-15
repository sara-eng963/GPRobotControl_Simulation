# Teammate Setup — Full EtherCAT Simulation

This guide recreates the PC simulation environment used by the control project.

The setup script downloads and builds:

- FreeRTOS-Kernel
- SOEM
- KickCAT
- Raylib submodule
- GPRobotControl controller + HMI

The only file that is **not** stored in this repository is the A6-EC ESI file:

```text
A6-EC.xml
```

Sara should send that XML separately.

## 1. Install WSL + Ubuntu

If WSL is not already installed, open **PowerShell as Administrator** and run:

```powershell
wsl --install -d Ubuntu
```

Restart Windows if requested, then open **Ubuntu** from the Start menu.

For WSL GUI support, it is also a good idea to update WSL from PowerShell:

```powershell
wsl --update
```

## 2. Clone the project

Inside Ubuntu/WSL:

```bash
cd ~
sudo apt update
sudo apt install -y git

git clone -b feature/circular-trajectories \
    --recurse-submodules \
    https://github.com/sara-eng963/GPRobotControl_Simulation.git \
    freertos-pc-test

cd ~/freertos-pc-test
```

If the repository is private, GitHub access to the repository is required.

## 3. Put `A6-EC.xml` somewhere accessible

The easiest option is to copy the XML into the project root so the setup script finds it automatically:

```text
~/freertos-pc-test/A6-EC.xml
```

If the file is still in the Windows Downloads folder, it can also be passed directly to the setup script using its `/mnt/c/...` path.

Example:

```bash
bash scripts/setup_environment.sh "/mnt/c/Users/<WINDOWS_USERNAME>/Downloads/A6-EC.xml"
```

## 4. Run the one-time environment setup

If `A6-EC.xml` is already in the project root:

```bash
cd ~/freertos-pc-test
bash scripts/setup_environment.sh
```

Or pass the XML explicitly:

```bash
bash scripts/setup_environment.sh /path/to/A6-EC.xml
```

The script will:

1. install the required Ubuntu packages,
2. install a local CMake/Conan tool environment,
3. download pinned FreeRTOS-Kernel,
4. download pinned SOEM,
5. download and build pinned KickCAT,
6. create the tested `A6-EC.json`,
7. copy `A6-EC.xml` into KickCAT,
8. initialize Raylib,
9. build `freertos_pc_test`,
10. build `mock_hmi`.

Expected layout after setup:

```text
~/
├── FreeRTOS-Kernel/
├── SOEM/
├── KickCAT/
│   └── simulation/slave_configs/
│       ├── A6-EC.json
│       └── A6-EC.xml
└── freertos-pc-test/
    ├── build/
    │   ├── freertos_pc_test
    │   └── mock_hmi
    └── scripts/
        ├── setup_environment.sh
        └── run_simulation.sh
```

## 5. Run the full system

After setup succeeds:

```bash
cd ~/freertos-pc-test
bash scripts/run_simulation.sh
```

The launcher automatically:

- removes stale simulation processes,
- creates `ecatA` and `ecatB`,
- brings both virtual EtherCAT interfaces up,
- starts six KickCAT A6-EC simulated slaves,
- starts FreeRTOS + SOEM,
- starts the HMI,
- isolates KickCAT, controller and HMI onto separate CPUs,
- cleans everything when the HMI closes or `Ctrl+C` is pressed.

Architecture:

```text
FreeRTOS + SOEM
      │
    ecatA
      │
  virtual veth
      │
    ecatB
      │
KickCAT x6 A6-EC slaves

HMI communicates with the controller separately over localhost UDP.
```

## Stop the simulation

Either close the HMI window or press:

```text
Ctrl+C
```

The launcher will stop the controller, stop KickCAT, and remove `ecatA` / `ecatB`.

## Logs

Runtime logs are written under:

```text
/tmp/gprobot_simulation_<linux-user>/
```

To watch the controller live:

```bash
tail -f /tmp/gprobot_simulation_$USER/controller.log
```

## Dependency revisions

The setup script pins the dependency revisions instead of always downloading the newest version. This keeps teammate machines reproducible and avoids a future dependency update unexpectedly breaking the tested environment.
