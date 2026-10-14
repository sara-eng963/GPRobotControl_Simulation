#include "hmi_lifecycle.h"

bool gp_hmi_status_decode(const GpIpcMessage *msg, uint32_t last_sequence,
                          GpHmiControllerStatus *out)
{
    if (msg == NULL || out == NULL || msg->type != GP_IPC_CONTROLLER_STATUS ||
        msg->sequence == 0U ||
        (last_sequence != 0U &&
         (int32_t)(msg->sequence - last_sequence) <= 0) ||
        msg->data[0] >= (uint32_t)ROBOT_STATE_COUNT ||
        msg->data[1] > 1U || msg->data[4] > 1U)
        return false;

    *out = (GpHmiControllerStatus) {
        .ipc_sequence = msg->sequence,
        .robot_state = msg->data[0],
        .motion_permitted = msg->data[1] != 0U,
        .active_fault = msg->data[2],
        .transition_count = msg->data[3],
        .estop_active = msg->data[4] != 0U,
        .last_hmi_received_sequence = msg->data[5],
        .rejected_event_count = msg->data[6]
    };
    return true;
}

bool gp_hmi_status_is_fresh(uint32_t now, uint32_t received,
                            uint32_t max_age_ticks)
{
    /* FreeRTOS CM4 configuration has a 32-bit tick; subtraction tolerates
     * rollover while refusing any status more than max_age_ticks old. */
    return max_age_ticks != 0U &&
           (uint32_t)(now - received) <= max_age_ticks;
}

bool gp_hmi_command_allowed(const GpHmiControllerStatus *status,
                            SupervisorHmiCommand command)
{
    if (status == NULL || (unsigned)command >= (unsigned)SUP_HMI_COMMAND_COUNT ||
        status->robot_state >= (uint32_t)ROBOT_STATE_COUNT ||
        status->estop_active ||
        status->robot_state == (uint32_t)ROBOT_STATE_EMERGENCY_STOP)
        return false;

    /* RESET and HOME may be requested after a fault, subject to M7 policy
     * and physically cleared safety circuits. Never replay buffered motion
     * commands while the M7 reports a fault. */
    if (status->robot_state == (uint32_t)ROBOT_STATE_FAULT ||
        status->active_fault != 0U)
        return command == SUP_HMI_RESET || command == SUP_HMI_HOME;

    return true;
}

bool gp_hmi_should_drop_pending(const GpHmiControllerStatus *old_status,
                                const GpHmiControllerStatus *new_status)
{
    if (new_status == NULL) return true;
    if (new_status->estop_active ||
        new_status->robot_state == (uint32_t)ROBOT_STATE_FAULT ||
        new_status->robot_state == (uint32_t)ROBOT_STATE_EMERGENCY_STOP ||
        new_status->active_fault != 0U)
        return true;
    return old_status != NULL &&
           (old_status->robot_state != new_status->robot_state ||
            old_status->transition_count != new_status->transition_count);
}
