#ifndef SUPERVISOR_IO_H
#define SUPERVISOR_IO_H

#include "state_machine.h"
#include <stdbool.h>
#include <stdint.h>

/* Logical channels: the board adapter owns pin numbers and active polarity. */
typedef enum {
    SUP_IO_ESTOP, SUP_IO_PROTECTIVE_STOP, SUP_IO_START,
    SUP_IO_PAUSE_RESUME, SUP_IO_RESET, SUP_IO_HOME,
    SUP_IO_RECORD, SUP_IO_VALIDATE_PREVIEW,
    SUP_IO_HOME_J1, SUP_IO_HOME_J2, SUP_IO_HOME_J3,
    SUP_IO_HOME_J4, SUP_IO_HOME_J5, SUP_IO_HOME_J6,
    SUP_IO_INPUT_COUNT
} SupervisorInputId;

typedef enum {
    SUP_IO_STATUS_READY, SUP_IO_STATUS_RUNNING, SUP_IO_STATUS_PAUSED,
    SUP_IO_STATUS_FAULT, SUP_IO_OUTPUT_COUNT
} SupervisorOutputId;

typedef struct {
    bool asserted[SUP_IO_INPUT_COUNT];
    uint32_t timestamp_ms;
    bool valid;
} SupervisorInputSnapshot;

typedef struct {
    bool asserted[SUP_IO_OUTPUT_COUNT];
} SupervisorOutputSnapshot;

/* Read a complete coherent snapshot. Return false on acquisition failure.
 * Debounce, edge detection and command interpretation belong in the adapter.
 * These functions run in task context and must not block.
 */
typedef bool (*SupervisorReadInputs)(void *, SupervisorInputSnapshot *);
typedef bool (*SupervisorWriteOutputs)(void *, const SupervisorOutputSnapshot *);
typedef struct {
    void *context;
    SupervisorReadInputs read_inputs;
    SupervisorWriteOutputs write_outputs;
} SupervisorIoServices;

void supervisor_io_make_status(const StateMachine *, SupervisorOutputSnapshot *);

/* Wire feed, drive enable and brakes remain owned by the existing state
 * service callbacks. Do not write those outputs from this status interface.
 */
#endif
