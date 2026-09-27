#ifndef HMI_API_H
#define HMI_API_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Stable HMI <-> controller event contract.
 *
 * This header deliberately contains no sockets, Raylib, Linux, KickCAT or
 * simulator types. The current desktop HMI and the future physical HMI should
 * emit the same HmiEvent values.
 *
 * A real emergency stop is NOT an HmiEvent. It belongs to the hardware safety
 * chain; software may observe/display that safety state.
 */

typedef enum
{
    HMI_PROGRAM_NONE = 0,
    HMI_PROGRAM_LINE,
    HMI_PROGRAM_ARC,
    HMI_PROGRAM_CIRCLE

} HmiProgramSelection;


typedef enum
{
    HMI_EVENT_NONE = 0,

    HMI_EVENT_SELECT_LINE = 1,
    HMI_EVENT_SELECT_ARC = 2,
    HMI_EVENT_SELECT_CIRCLE = 3,

    HMI_EVENT_RECORD = 4,

    /*
     * One operator button, context dependent:
     * TEACHING -> validate, VALID Path Validation -> preview request.
     */
    HMI_EVENT_VALIDATE_PREVIEW = 5,

    HMI_EVENT_SPEED_INCREASE = 6,
    HMI_EVENT_SPEED_DECREASE = 7,
    HMI_EVENT_SPEED_DEFAULT = 8,

    HMI_EVENT_START = 9,

    /* Explicit events prevent HMI/controller toggle desynchronization. */
    HMI_EVENT_PAUSE = 10,
    HMI_EVENT_RESUME = 11,

    HMI_EVENT_RESET = 12,
    HMI_EVENT_HOME = 13

} HmiEvent;


static inline bool hmi_event_is_valid(
    HmiEvent event
)
{
    return
        event > HMI_EVENT_NONE &&
        event <= HMI_EVENT_HOME;
}


#ifdef __cplusplus
}
#endif

#endif /* HMI_API_H */
