#include "hmi_state_bridge.h"

#include <stddef.h>
#include <string.h>


static TeachingEvent selected_program_event(
    HmiProgramSelection program
)
{
    switch (program)
    {
        case HMI_PROGRAM_LINE:
            return TEACH_EVENT_SELECT_LINE;

        case HMI_PROGRAM_ARC:
            return TEACH_EVENT_SELECT_ARC;

        case HMI_PROGRAM_CIRCLE:
            return TEACH_EVENT_SELECT_CIRCLE;

        case HMI_PROGRAM_NONE:
        default:
            return TEACH_EVENT_NONE;
    }
}


void hmi_state_bridge_init(
    HmiStateBridge *bridge
)
{
    if (bridge == NULL)
    {
        return;
    }

    memset(
        bridge,
        0,
        sizeof(*bridge)
    );

    bridge->selected_program =
        HMI_PROGRAM_LINE;

    bridge->pending_teaching_event =
        TEACH_EVENT_NONE;
}


bool hmi_state_bridge_handle_event(
    HmiStateBridge *bridge,
    HmiEvent event,
    StateMachine *machine
)
{
    if (
        bridge == NULL ||
        machine == NULL ||
        !hmi_event_is_valid(event)
    )
    {
        return false;
    }

    switch (event)
    {
        case HMI_EVENT_SELECT_LINE:
            bridge->selected_program =
                HMI_PROGRAM_LINE;

            if (
                machine->current_state ==
                ROBOT_STATE_TEACHING
            )
            {
                bridge->pending_teaching_event =
                    TEACH_EVENT_SELECT_LINE;
            }

            return true;

        case HMI_EVENT_SELECT_ARC:
            bridge->selected_program =
                HMI_PROGRAM_ARC;

            if (
                machine->current_state ==
                ROBOT_STATE_TEACHING
            )
            {
                bridge->pending_teaching_event =
                    TEACH_EVENT_SELECT_ARC;
            }

            return true;

        case HMI_EVENT_SELECT_CIRCLE:
            bridge->selected_program =
                HMI_PROGRAM_CIRCLE;

            if (
                machine->current_state ==
                ROBOT_STATE_TEACHING
            )
            {
                bridge->pending_teaching_event =
                    TEACH_EVENT_SELECT_CIRCLE;
            }

            return true;

        case HMI_EVENT_RECORD:
            if (
                machine->current_state !=
                ROBOT_STATE_TEACHING
            )
            {
                return false;
            }

            bridge->pending_teaching_event =
                TEACH_EVENT_RECORD_POINT;

            return true;

        case HMI_EVENT_VALIDATE_PREVIEW:
            if (
                machine->current_state ==
                ROBOT_STATE_TEACHING
            )
            {
                bridge->pending_teaching_event =
                    TEACH_EVENT_VALIDATE_PATH;

                return true;
            }

            if (
                machine->current_state ==
                    ROBOT_STATE_PATH_VALIDATION
                &&
                machine->path_validation_outputs.report.result ==
                    PV_RESULT_VALID
            )
            {
                bridge->preview_requested =
                    true;

                return true;
            }

            return false;

        case HMI_EVENT_SPEED_INCREASE:
            if (
                machine->current_state !=
                ROBOT_STATE_TEACHING
            )
            {
                return false;
            }

            bridge->pending_teaching_event =
                TEACH_EVENT_SPEED_INCREASE;

            return true;

        case HMI_EVENT_SPEED_DECREASE:
            if (
                machine->current_state !=
                ROBOT_STATE_TEACHING
            )
            {
                return false;
            }

            bridge->pending_teaching_event =
                TEACH_EVENT_SPEED_DECREASE;

            return true;

        case HMI_EVENT_SPEED_DEFAULT:
            if (
                machine->current_state !=
                ROBOT_STATE_TEACHING
            )
            {
                return false;
            }

            bridge->pending_teaching_event =
                TEACH_EVENT_SPEED_DEFAULT;

            return true;

        case HMI_EVENT_START:
            if (
                machine->current_state !=
                    ROBOT_STATE_IDLE
                ||
                machine->idle.phase ==
                    IDLE_PHASE_FAILED
            )
            {
                return false;
            }

            bridge->start_teach_requested =
                true;

            return true;

        case HMI_EVENT_PAUSE:
            bridge->paused =
                true;

            return true;

        case HMI_EVENT_RESUME:
            bridge->paused =
                false;

            return true;

        case HMI_EVENT_RESET:
            if (
                machine->current_state ==
                ROBOT_STATE_TEACHING
            )
            {
                bridge->pending_teaching_event =
                    TEACH_EVENT_RESET;

                return true;
            }

            if (
                machine->current_state ==
                ROBOT_STATE_PATH_VALIDATION
            )
            {
                /*
                 * Current global inputs do not yet expose a generic
                 * Path-Validation cancel request. Hide that detail here so the
                 * HMI contract does not change when the FSM gains one.
                 */
                state_path_validation_cancel(
                    &machine->path_validation
                );

                return true;
            }

            if (
                machine->current_state ==
                ROBOT_STATE_APPROACH
            )
            {
                bridge->approach_reset_requested =
                    true;

                return true;
            }

            return false;

        case HMI_EVENT_HOME:
            if (
                machine->current_state ==
                ROBOT_STATE_APPROACH
            )
            {
                bridge->approach_home_requested =
                    true;

                return true;
            }

            if (
                machine->current_state ==
                ROBOT_STATE_IDLE
            )
            {
                /*
                 * Current global inputs do not yet expose HOME from IDLE.
                 * Encapsulate this temporary transition here instead of
                 * duplicating it in simulator_main.c and future main.c.
                 */
                machine->previous_state =
                    machine->current_state;

                state_homing_enter(
                    &machine->homing
                );

                machine->current_state =
                    ROBOT_STATE_HOMING;

                return true;
            }

            return false;

        case HMI_EVENT_NONE:
        default:
            return false;
    }
}


void hmi_state_bridge_apply_inputs(
    HmiStateBridge *bridge,
    const StateMachine *machine,
    StateMachineInputs *inputs
)
{
    if (
        bridge == NULL ||
        machine == NULL ||
        inputs == NULL
    )
    {
        return;
    }

    if (
        machine->current_state ==
            ROBOT_STATE_IDLE
        &&
        bridge->start_teach_requested
    )
    {
        inputs->idle_command =
            IDLE_COMMAND_TEACH;
    }

    if (
        machine->current_state ==
            ROBOT_STATE_TEACHING
        &&
        bridge->pending_teaching_event !=
            TEACH_EVENT_NONE
    )
    {
        inputs->teaching_event =
            bridge->pending_teaching_event;

        bridge->pending_teaching_event =
            TEACH_EVENT_NONE;
    }

    if (
        machine->current_state ==
            ROBOT_STATE_PATH_VALIDATION
        &&
        bridge->preview_requested
    )
    {
        inputs->approach_operation =
            APPROACH_OPERATION_PREVIEW;
    }

    inputs->approach_control.pause_requested =
        bridge->paused;

    if (
        machine->current_state ==
            ROBOT_STATE_APPROACH
    )
    {
        inputs->approach_control.reset_requested =
            bridge->approach_reset_requested;

        inputs->approach_control.home_requested =
            bridge->approach_home_requested;

        bridge->approach_reset_requested =
            false;

        bridge->approach_home_requested =
            false;
    }
}


void hmi_state_bridge_on_state_transition(
    HmiStateBridge *bridge,
    RobotState previous_state,
    const StateMachine *machine
)
{
    (void)previous_state;

    if (
        bridge == NULL ||
        machine == NULL
    )
    {
        return;
    }

    if (
        machine->current_state ==
        ROBOT_STATE_TEACHING
    )
    {
        bridge->start_teach_requested =
            false;

        bridge->pending_teaching_event =
            selected_program_event(
                bridge->selected_program
            );
    }

    if (
        machine->current_state ==
        ROBOT_STATE_APPROACH
    )
    {
        bridge->preview_requested =
            false;
    }
}


HmiProgramSelection hmi_state_bridge_selected_program(
    const HmiStateBridge *bridge
)
{
    if (bridge == NULL)
    {
        return HMI_PROGRAM_NONE;
    }

    return bridge->selected_program;
}


bool hmi_state_bridge_is_paused(
    const HmiStateBridge *bridge
)
{
    return
        bridge != NULL &&
        bridge->paused;
}
