# HMI API Handoff

The desktop HMI is a behavioral substitute for the future physical panel.
The HMI-to-controller API is now independent of Raylib and UDP.

## Stable controller-facing API

Use:

```
HMI/hmi_api.h
HMI/hmi_state_bridge.h
```

The physical HMI should produce the same `HmiEvent` values:

```
HMI_EVENT_START
HMI_EVENT_SELECT_LINE
HMI_EVENT_SELECT_ARC
HMI_EVENT_SELECT_CIRCLE
HMI_EVENT_RECORD
HMI_EVENT_VALIDATE_PREVIEW
HMI_EVENT_SPEED_INCREASE
HMI_EVENT_SPEED_DECREASE
HMI_EVENT_SPEED_DEFAULT
HMI_EVENT_PAUSE
HMI_EVENT_RESUME
HMI_EVENT_RESET
HMI_EVENT_HOME
```

## Example for the future real main.c

```c
#include "HMI/hmi_state_bridge.h"

static HmiStateBridge hmi_bridge;
static StateMachine machine;
static StateMachineInputs inputs;

void app_init(void)
{
    hmi_state_bridge_init(&hmi_bridge);
    state_machine_init(&machine, 1U);
}

void on_real_hmi_event(HmiEvent event)
{
    (void)hmi_state_bridge_handle_event(
        &hmi_bridge,
        event,
        &machine
    );
}

void supervisor_cycle(void)
{
    memset(&inputs, 0, sizeof(inputs));

    /* Fill real safety, guidance, calibration and runtime inputs here. */

    hmi_state_bridge_apply_inputs(
        &hmi_bridge,
        &machine,
        &inputs
    );

    RobotState previous = machine.current_state;

    (void)state_machine_step(
        &machine,
        &dependencies,
        &inputs
    );

    if (machine.current_state != previous)
    {
        hmi_state_bridge_on_state_transition(
            &hmi_bridge,
            previous,
            &machine
        );
    }
}
```

The HMI never sets `machine.current_state` itself.

## Event mapping examples

```
START
 -> IDLE_COMMAND_TEACH

SELECT_LINE
 -> TEACH_EVENT_SELECT_LINE

RECORD
 -> TEACH_EVENT_RECORD_POINT

VALIDATE_PREVIEW while TEACHING
 -> TEACH_EVENT_VALIDATE_PATH

VALIDATE_PREVIEW after PV_RESULT_VALID
 -> APPROACH_OPERATION_PREVIEW
```

## Transport replacement

Today:

```
Raylib button
 -> HmiEvent
 -> PC UDP
 -> hmi_state_bridge
 -> StateMachineInputs
 -> state_machine_step()
```

Later:

```
Physical HMI button
 -> HmiEvent
 -> real panel transport
 -> hmi_state_bridge
 -> StateMachineInputs
 -> state_machine_step()
```

Only the transport changes.

Simulation-only items are intentionally NOT part of the stable HMI event API:
mouse TCP guidance, software E-stop toggle, KickCAT, MATLAB telemetry and mock
Preview execution.
