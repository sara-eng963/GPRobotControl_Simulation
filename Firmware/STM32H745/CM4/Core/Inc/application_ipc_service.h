#ifndef APPLICATION_IPC_SERVICE_H
#define APPLICATION_IPC_SERVICE_H

#include <stdbool.h>
#include <stdint.h>
#include "supervisor_hmi.h"

/* CM4-local view of the M7 supervisor's existing seven status data fields.
 * last_hmi_received_sequence acknowledges IPC receipt, NOT command acceptance.
 * Status from the remote kernel is advisory and never authorizes motion. */
typedef struct {
    uint32_t ipc_sequence;
    uint32_t robot_state;
    bool motion_permitted;
    uint32_t active_fault;
    uint32_t transition_count;
    bool estop_active;
    uint32_t last_hmi_received_sequence;
    uint32_t rejected_event_count;
} ApplicationControllerStatus;

/* Call once in the CM4 default task after the scheduler has started.
 * Requires Stage 2 M7-owned shared memory to be valid. */
bool application_ipc_service_initialize(void);

/* Sole consumer/producer of the two shared message-buffer directions on M4.
 * Runs inside the existing generated CM4 default task; never returns. */
void application_ipc_service_run(void);

/* Task-context, same-M4-kernel APIs. No ISR or cross-core FreeRTOS handles.
 * Failure means invalid command, unavailable/stale M7 status or full queue.
 * Success means locally queued, NOT accepted or executed by the supervisor. */
bool application_ipc_service_submit_hmi(SupervisorHmiCommand command);

/* Latest M7 status, false when absent or older than max_age_ms.
 * Age uses CM4 ticks measured at receipt, not unsynchronized M7 timestamps.
 * max_age_ms=0 requests a snapshot without a freshness guarantee. */
bool application_ipc_service_read_status(
    ApplicationControllerStatus *out, uint32_t max_age_ms);

#endif
