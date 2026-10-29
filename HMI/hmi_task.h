#ifndef HMI_TASK_H
#define HMI_TASK_H
#include "hmi_api.h"
#if defined(ROBOT_PRODUCTION_SIMULATION)
#include "../Firmware/STM32H745/CM7/Application/StateMachine/supervisor_task.h"
#else
#include "../StateMachine/supervisor_task.h"
#endif
/* Optional transport callbacks must be nonblocking. Return false when no event.
 * GUI/socket or future RS485 code owns decoding, not this task or the states. */
typedef struct {
    bool (*receive_event)(void *context, HmiEvent *event);
    void (*publish_status)(void *context, const SupervisorDiagnostics *status);
    void *context;
    TickType_t period_ticks;
} HmiTaskConfig;
bool hmi_task_init(const HmiTaskConfig *config);
bool hmi_task_start(UBaseType_t priority);
/* Used by an existing GUI bridge, or scripted tests of the same HmiEvent API.
 * Success means queued, not accepted by the current robot state. */
bool hmi_task_submit(HmiEvent event, TickType_t wait);
#endif
