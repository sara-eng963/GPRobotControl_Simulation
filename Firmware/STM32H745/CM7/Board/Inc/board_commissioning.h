#ifndef BOARD_COMMISSIONING_H
#define BOARD_COMMISSIONING_H

#include "../../Application/StateMachine/supervisor_task.h"
#include "../../../../../HandGuiding/supervisor_dry_run_adapter.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * Board-specific provision interface, NOT a safety certification API.
 *
 * A real, separately reviewed board module must supply this function.
 * The firmware deliberately provides no default implementation, and
 * application_supervisor_board_configure() refuses to start without one.
 * The production implementation must use verified physical inputs, external
 * E-stop/STO/brakes, installed transceiver/pin mapping, and the selected
 * drive's vendor EDS/object dictionary. Never substitute simulated "ready".
 */
typedef struct {
    SupervisorTaskConfig supervisor;
    uint8_t node_ids[JOINT_DRIVE_AXES];
    uint8_t node_count;
    uint32_t heartbeat_timeout_ms;
    uint32_t sdo_timeout_ms;
    /* Optional observation-only F/T pipeline configuration supplied by a
     * real commissioned board. NULL disables this callback entirely.
     * It does not grant motion authority or supply synthetic sensors. */
    const HandGuidingDryRunConfig *guidance_dry_run_config;
    /* Explicit interlock integration must exist in the physical board port. */
    bool (*hardware_interlocks_verified)(void *context);
    void *hardware_interlock_context;
} BoardCommissioning;

/* Declared strong here. In the composition implementation, this symbol is
 * referenced weakly so an absent production board port links and fails closed.
 * No drive node IDs, safety status, or physical pin mapping are invented. */
bool board_supply_commissioned_config(BoardCommissioning *out);

#endif /* BOARD_COMMISSIONING_H */
