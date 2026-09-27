#include "state_machine.h"

#include <stddef.h>
#include <string.h>


/* ============================================================================
 * INTERNAL STATE TRANSITION
 * ============================================================================ */

static void change_state(
    StateMachine *machine,
    RobotState next_state
)
{
    machine->previous_state =
        machine->current_state;

    machine->current_state =
        next_state;
}


/* ============================================================================
 * INITIALIZATION
 * ============================================================================ */

bool state_machine_init(
    StateMachine *machine,
    uint32_t teaching_program_id
)
{
    if (machine == NULL)
    {
        return false;
    }


    memset(
        machine,
        0,
        sizeof(*machine)
    );


    machine->current_state =
        ROBOT_STATE_BOOT;

    machine->previous_state =
        ROBOT_STATE_BOOT;

    machine->teaching_program_id =
        teaching_program_id;

    machine->initialized =
        true;


    state_boot_enter(
        &machine->boot
    );


    return true;
}


/* ============================================================================
 * GLOBAL STEP
 * ============================================================================ */

StateStepResult state_machine_step(
    StateMachine *machine,
    const StateMachineDependencies *dependencies,
    const StateMachineInputs *inputs
)
{
    if (
        machine == NULL ||
        dependencies == NULL ||
        inputs == NULL ||
        !machine->initialized
    )
    {
        return
            STATE_STEP_FAILED;
    }


    switch (machine->current_state)
    {
        /* ====================================================================
         * BOOT
         * ==================================================================== */

        case ROBOT_STATE_BOOT:
        {
            if (dependencies->ethercat_config == NULL)
            {
                return
                    STATE_STEP_FAILED;
            }


            StateStepResult result =
                state_boot_step(
                    &machine->boot,
                    dependencies->ethercat_config
                );


            if (result == STATE_STEP_COMPLETE)
            {
                state_homing_enter(
                    &machine->homing
                );

                change_state(
                    machine,
                    ROBOT_STATE_HOMING
                );

                return
                    STATE_STEP_RUNNING;
            }


            return
                result;
        }


        /* ====================================================================
         * HOMING
         * ==================================================================== */

        case ROBOT_STATE_HOMING:
        {
            if (
                dependencies->homing_config == NULL ||
                dependencies->robot == NULL
            )
            {
                return
                    STATE_STEP_FAILED;
            }


            StateStepResult result =
                state_homing_step(
                    &machine->homing,
                    dependencies->homing_config,
                    dependencies->robot
                );


            if (result == STATE_STEP_COMPLETE)
            {
                state_idle_enter(
                    &machine->idle
                );

                change_state(
                    machine,
                    ROBOT_STATE_IDLE
                );

                return
                    STATE_STEP_RUNNING;
            }


            return
                result;
        }


        /* ====================================================================
         * IDLE
         * ==================================================================== */

        case ROBOT_STATE_IDLE:
        {
            StateStepResult result =
                state_idle_step(
                    &machine->idle,
                    inputs->idle_command
                );


            if (result != STATE_STEP_COMPLETE)
            {
                return
                    result;
            }


            switch (machine->idle.exitCommand)
            {
                case IDLE_COMMAND_TEACH:
                {
                    if (
                        dependencies->teaching_config == NULL
                    )
                    {
                        return
                            STATE_STEP_FAILED;
                    }


                    state_teaching_enter(
                        &machine->teaching,
                        dependencies->teaching_config,
                        machine->teaching_program_id
                    );


                    change_state(
                        machine,
                        ROBOT_STATE_TEACHING
                    );


                    return
                        STATE_STEP_RUNNING;
                }


                /*
                 * Replay / Preview is not integrated yet.
                 *
                 * Do not invent a transition until that state exists.
                 */
                case IDLE_COMMAND_REPLAY:
                case IDLE_COMMAND_NONE:
                default:
                {
                    return
                        STATE_STEP_FAILED;
                }
            }
        }


        /* ====================================================================
         * TEACHING
         * ==================================================================== */

        case ROBOT_STATE_TEACHING:
        {
            if (dependencies->robot == NULL)
            {
                return
                    STATE_STEP_FAILED;
            }


            StateStepResult result =
                state_teaching_step(
                    &machine->teaching,
                    dependencies->robot,
                    &inputs->teaching_runtime,
                    inputs->teaching_event,
                    &machine->teaching_outputs
                );


            if (result == STATE_STEP_FAILED)
            {
                return
                    STATE_STEP_FAILED;
            }


            /*
             * Teaching remains active until the operator submits the draft.
             */
            if (result == STATE_STEP_RUNNING)
            {
                return
                    STATE_STEP_RUNNING;
            }


            /*
             * Teaching returning COMPLETE should mean:
             *
             *      validation_request == true
             */
            if (
                !machine->teaching_outputs.validation_request
            )
            {
                return
                    STATE_STEP_FAILED;
            }


            /* ---------------------------------------------------------------
             * TEACHING -> PATH VALIDATION
             * --------------------------------------------------------------- */

            if (
                dependencies->robot == NULL ||
                dependencies->path_validation_config == NULL ||
                dependencies->path_validation_workspace == NULL ||
                dependencies->path_validation_storage == NULL ||
                dependencies->validated_trajectory == NULL
            )
            {
                return
                    STATE_STEP_FAILED;
            }


            state_path_validation_enter(
                &machine->path_validation,

                dependencies->robot,

                dependencies->path_validation_config,

                dependencies->path_validation_services,

                dependencies->path_validation_workspace,

                dependencies->path_validation_storage,

                dependencies->validated_trajectory,

                &machine->teaching.draft,

                machine->teaching_outputs.submitted_revision,

                machine->teaching_outputs.submitted_crc
            );


            /*
             * Make sure enter() actually succeeded.
             */
            if (
                machine->path_validation.result !=
                PV_RESULT_RUNNING
            )
            {
                state_path_validation_get_outputs(
                    &machine->path_validation,
                    &machine->path_validation_outputs
                );

                return
                    STATE_STEP_FAILED;
            }


            change_state(
                machine,
                ROBOT_STATE_PATH_VALIDATION
            );


            return
                STATE_STEP_RUNNING;
        }


        /* ====================================================================
         * PATH VALIDATION
         * ==================================================================== */

        case ROBOT_STATE_PATH_VALIDATION:
        {
            StateStepResult result =
                state_path_validation_step(
                    &machine->path_validation,
                    inputs->path_validation_sample_budget,
                    &machine->path_validation_outputs
                );


            if (result == STATE_STEP_FAILED)
            {
                return
                    STATE_STEP_FAILED;
            }


            if (result == STATE_STEP_RUNNING)
            {
                return
                    STATE_STEP_RUNNING;
            }


            /*
             * Validation has now finished.
             *
             * For now DO NOT automatically transition elsewhere.
             *
             * Inspect:
             *
             *      machine->path_validation_outputs.report.result
             *
             * It can be:
             *
             *      PV_RESULT_VALID
             *      PV_RESULT_INVALID
             *      PV_RESULT_CANCELLED
             *
             * When Preview / Edit behavior is implemented, the transition
             * belongs here.
             */

            return
                STATE_STEP_COMPLETE;
        }


        default:
        {
            return
                STATE_STEP_FAILED;
        }
    }
}


/* ============================================================================
 * STATUS HELPERS
 * ============================================================================ */

RobotState state_machine_current_state(
    const StateMachine *machine
)
{
    if (machine == NULL)
    {
        return
            ROBOT_STATE_BOOT;
    }


    return
        machine->current_state;
}


const char *state_machine_state_name(
    RobotState state
)
{
    switch (state)
    {
        case ROBOT_STATE_BOOT:
            return "BOOT";

        case ROBOT_STATE_HOMING:
            return "HOMING";

        case ROBOT_STATE_IDLE:
            return "IDLE";

        case ROBOT_STATE_TEACHING:
            return "TEACHING";

        case ROBOT_STATE_PATH_VALIDATION:
            return "PATH_VALIDATION";

        default:
            return "UNKNOWN";
    }
}