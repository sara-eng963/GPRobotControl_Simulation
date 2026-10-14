#ifndef APPLICATION_SUPERVISOR_H
#define APPLICATION_SUPERVISOR_H
#include <stdbool.h>
#include "supervisor_task.h"

/* Fail-closed default until an actual STM32 board port supplies real I/O. */
bool application_supervisor_board_configure(SupervisorTaskConfig *config);
/* Target-specific composition of the existing supervisor, not a new RTOS. */
bool application_supervisor_start(const SupervisorTaskConfig *config);
#endif
