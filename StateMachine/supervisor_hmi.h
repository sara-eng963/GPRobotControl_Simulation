#ifndef SUPERVISOR_HMI_H
#define SUPERVISOR_HMI_H

/* Transport-neutral commands. The screen/RS485 driver translates its own IDs
 * into these values, once per button activation. No GPIO/HAL dependencies. */
typedef enum {
    SUP_HMI_LINE = 0, SUP_HMI_ARC, SUP_HMI_CIRCLE,
    SUP_HMI_RECORD, SUP_HMI_VALIDATE_PREVIEW, SUP_HMI_START_REPLAY,
    SUP_HMI_PAUSE_RESUME, SUP_HMI_RESET, SUP_HMI_HOME,
    SUP_HMI_SPEED_UP, SUP_HMI_SPEED_DOWN, SUP_HMI_SPEED_DEFAULT,
    SUP_HMI_PAUSE, SUP_HMI_RESUME,
    SUP_HMI_COMMAND_COUNT
} SupervisorHmiCommand;
#endif
