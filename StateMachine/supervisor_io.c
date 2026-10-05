#include "supervisor_io.h"
#include <string.h>

void supervisor_io_make_status(const StateMachine *machine,
                               SupervisorOutputSnapshot *outputs)
{
    if (outputs == NULL) return;
    memset(outputs, 0, sizeof(*outputs));
    if (machine == NULL || !machine->initialized) return;
    outputs->asserted[SUP_IO_STATUS_READY] =
        machine->activeState == ROBOT_STATE_IDLE && state_machine_can_move(machine);
    outputs->asserted[SUP_IO_STATUS_RUNNING] =
        machine->activeState == ROBOT_STATE_APPROACH ||
        machine->activeState == ROBOT_STATE_PATH_EXECUTION ||
        machine->activeState == ROBOT_STATE_HOMING;
    outputs->asserted[SUP_IO_STATUS_PAUSED] = machine->activeState == ROBOT_STATE_PAUSED;
    outputs->asserted[SUP_IO_STATUS_FAULT] =
        machine->activeState == ROBOT_STATE_FAULT ||
        machine->activeState == ROBOT_STATE_EMERGENCY_STOP;
}
