#ifndef GP_HMI_LIFECYCLE_H
#define GP_HMI_LIFECYCLE_H

#include "ipc_mailbox.h"
#include "../StateMachine/state_machine_types.h"
#include "../StateMachine/supervisor_hmi.h"
#include <stdbool.h>
#include <stdint.h>

/* Controller telemetry is advisory; M7 remains the ONLY authority for
 * robot motion and event acceptance. All times are M4-local tick values.
 * The IPC sequence acknowledges RECEIPT, never execution of an HMI command. */
typedef struct {
    uint32_t ipc_sequence;
    uint32_t robot_state;
    bool motion_permitted;
    uint32_t active_fault;
    uint32_t transition_count;
    bool estop_active;
    uint32_t last_hmi_received_sequence;
    uint32_t rejected_event_count;
} GpHmiControllerStatus;

/* Validates the existing seven-word M7 status ABI; rejects invalid and
 * out-of-order frames. No new message type or memory layout is introduced. */
bool gp_hmi_status_decode(const GpIpcMessage *msg, uint32_t last_sequence,
                          GpHmiControllerStatus *out);

/* Called at the M4 UI submission boundary AND just before IPC send.
 * max_age_ticks=0 means no extra expiry check (diagnostic access only);
 * commands must always supply a nonzero limit. */
bool gp_hmi_status_is_fresh(uint32_t local_now, uint32_t receipt_tick,
                            uint32_t max_age_ticks);
bool gp_hmi_command_allowed(const GpHmiControllerStatus *status,
                            SupervisorHmiCommand command);

/* A changed controller state or unsafe new status discards queued UI events.
 * This does not replace physical protective stop or M7 Supervisor gating. */
bool gp_hmi_should_drop_pending(const GpHmiControllerStatus *old_status,
                                const GpHmiControllerStatus *new_status);

#endif
