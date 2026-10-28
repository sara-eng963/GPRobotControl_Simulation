#ifndef DRIVE_RESTART_GATE_H
#define DRIVE_RESTART_GATE_H
/* Pure C2B policy. Software decision only: NOT a functional-safety channel.
 * `operator_pressed` MUST be a live physical-board input, never an HMI,
 * retained memory or IPC command. Missing callback is always denied. */
#include <stdbool.h>
#include <stdint.h>
#include "reset_diagnostics.h"
#include "supervision_progress.h"

typedef struct {
    /* All fields must refer to the CURRENT boot/session and CURRENT cycle. */
    bool valid;
    bool operator_pressed;
    bool board_ready;
    bool reset_reviewed;
    bool ipc_session_fresh;
    bool old_commands_discarded;
    bool feedback_revalidated;
} DriveRestartBoardSample;

typedef struct {
    bool (*read)(void *context, DriveRestartBoardSample *out);
    void *context;
} DriveRestartBoardPort;

typedef struct {
    bool authorized;
    bool inhibited;
    bool observed_release;
    bool previous_pressed;
} DriveRestartGate;

typedef struct {
    bool communication_ready;
    bool state_ready;
    bool safety_valid;
    bool estop_clear;
    bool protective_stop_clear;
    bool fault_clear;
    bool ipc_valid;
    bool preenable_status_checked;
} DriveRestartSystemSample;

static inline void drive_restart_gate_init(DriveRestartGate *gate)
{
    if (gate == NULL) return;
    *gate = (DriveRestartGate){0};
}

static inline void drive_restart_gate_inhibit(DriveRestartGate *gate)
{
    if (gate == NULL) return;
    gate->authorized = false;
    gate->inhibited = true; /* Latch until NEW BOOT: RESET is not approval. */
    gate->observed_release = false;
    gate->previous_pressed = false;
}

static inline bool drive_restart_gate_tick(
    DriveRestartGate *gate, const ResetDiagnostics *reset,
    const DriveRestartBoardPort *port,
    const DriveRestartSystemSample *system)
{
    if (gate == NULL || gate->inhibited) return false;
    DriveRestartBoardSample board = {0};
    if (reset == NULL || !reset->captured ||
        port == NULL || port->read == NULL ||
        system == NULL || !port->read(port->context, &board) ||
        !board.valid || !board.board_ready ||
        !system->communication_ready || !system->state_ready ||
        !system->safety_valid || !system->estop_clear ||
        !system->protective_stop_clear || !system->fault_clear ||
        !system->ipc_valid || !system->preenable_status_checked)
    {
        if (gate->authorized) drive_restart_gate_inhibit(gate);
        else { gate->observed_release = false; gate->previous_pressed = false; }
        return false;
    }
    SupervisionResetReason reason = SUP_RESET_OTHER;
    switch (reset->cause) {
        case RESET_CAUSE_POWER_ON:
            reason = SUP_RESET_EXPECTED_POWER_ON; break;
        case RESET_CAUSE_BROWNOUT:
            reason = SUP_RESET_BROWNOUT; break;
        case RESET_CAUSE_WATCHDOG:
            reason = SUP_RESET_WATCHDOG; break;
        default: reason = SUP_RESET_OTHER; break;
    }
    /* C2A restart policy additionally blocks stale IPC and no-approval cases. */
    const bool policy_without_operator = supervision_restart_authorized(
        &(SupervisionRestartInputs){
            .reason = reason,
            .reset_cause_reviewed = board.reset_reviewed,
            .fresh_ipc_session = board.ipc_session_fresh,
            .stale_commands_discarded = board.old_commands_discarded,
            .independent_safety_ready = board.board_ready,
            .drive_feedback_revalidated = board.feedback_revalidated,
            .operator_reauthorized = true
        });
    if (!policy_without_operator) {
        if (gate->authorized) drive_restart_gate_inhibit(gate);
        else { gate->observed_release = false; gate->previous_pressed = false; }
        return false;
    }
    if (!gate->authorized) {
        /* A held-high input at startup does NOT authorize. A low sample must
         * occur AFTER all required startup conditions became valid. */
        if (!board.operator_pressed) gate->observed_release = true;
        if (gate->observed_release && !gate->previous_pressed &&
            board.operator_pressed)
            gate->authorized = true;
    }
    gate->previous_pressed = board.operator_pressed;
    return gate->authorized;
}
#endif
