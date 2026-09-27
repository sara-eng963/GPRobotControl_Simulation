#ifndef HMI_STATE_BRIDGE_H
#define HMI_STATE_BRIDGE_H

#include "hmi_api.h"

#include "../StateMachine/state_machine.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Shared controller-side HMI adapter.
 *
 * Simulation/simulator_main.c uses this now.
 * The future real main.c should use the SAME functions.
 *
 * The HMI sends operator intent (HmiEvent); it never directly chooses
 * RobotState. This adapter maps events to the existing StateMachine inputs and
 * state APIs.
 */
typedef struct
{
    HmiProgramSelection selected_program;

    bool start_teach_requested;
    bool preview_requested;

    TeachingEvent pending_teaching_event;

    bool paused;

    bool approach_reset_requested;
    bool approach_home_requested;

} HmiStateBridge;


void hmi_state_bridge_init(
    HmiStateBridge *bridge
);


bool hmi_state_bridge_handle_event(
    HmiStateBridge *bridge,
    HmiEvent event,
    StateMachine *machine
);


void hmi_state_bridge_apply_inputs(
    HmiStateBridge *bridge,
    const StateMachine *machine,
    StateMachineInputs *inputs
);


void hmi_state_bridge_on_state_transition(
    HmiStateBridge *bridge,
    RobotState previous_state,
    const StateMachine *machine
);


HmiProgramSelection hmi_state_bridge_selected_program(
    const HmiStateBridge *bridge
);


bool hmi_state_bridge_is_paused(
    const HmiStateBridge *bridge
);


#ifdef __cplusplus
}
#endif

#endif /* HMI_STATE_BRIDGE_H */
