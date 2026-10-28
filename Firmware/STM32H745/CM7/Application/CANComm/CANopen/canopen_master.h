#ifndef STM32_CANOPEN_MASTER_H
#define STM32_CANOPEN_MASTER_H

#include "../can_backend.h"
#include "canopen_nmt.h"
#include "canopen_sdo.h"
#include "../../ServoDrive/AvatarM/avatar_m_drive.h"
#include "../../ServoDrive/JointDrive/joint_drive_port.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * AVATAR manual, section 11.2: consumer 0x1016:01 defaults to monitoring
 * producer Node-ID 0x7F, with a 2000 ms timeout (0x007F07D0).
 * The controller must have a distinct CANopen Node-ID.
 */
#define CANOPEN_CONTROLLER_NODE_ID         127U
#define CANOPEN_CONTROLLER_HEARTBEAT_MS    100U
#define CANOPEN_DRIVE_CONSUMER_TIMEOUT_MS  2000U
#define CANOPEN_MASTER_MAX_RX_PER_POLL     32U /* 2 x CubeMX RX FIFO0 depth */
#define CANOPEN_DRIVE_CONSUMER_SUBINDEX    1U
#define CANOPEN_DRIVE_CONSUMER_VALUE \
    (((uint32_t)CANOPEN_CONTROLLER_NODE_ID << 16) | \
     (uint32_t)CANOPEN_DRIVE_CONSUMER_TIMEOUT_MS)

#include "../../../../../../CANComm/CANopen/canopen_master_types.h"

/* Initialize before task scheduling; bind exactly once inside owner task.
 * Unbound coordinators retain standalone-test/initialization semantics.
 * Bound calls from another task fail without touching the CAN transport. */
bool canopen_master_bind_runtime_owner(CanopenMaster *master,
                                       CanopenMasterOwnerCheck check,
                                       void *context);

/* Single-owner explicit gate; never accepts a permission from an RX frame. */
void canopen_master_set_drive_permission(CanopenMaster *master,
                                         bool permitted);

/*
 * Initialize the six-axis CANopen coordinator around an already-created
 * transport backend. The backend can be SIL Kit now and STM32 FDCAN later.
 */
bool canopen_master_init(
    CanopenMaster *master,
    const CanopenMasterConfig *config
);

void canopen_master_close(
    CanopenMaster *master
);

/*
 * Clear feedback/heartbeat/SDO runtime state while preserving the configured
 * backend and node IDs. Used when BOOT deliberately restarts the CANopen
 * network with NMT Reset Communication.
 */
void canopen_master_clear_runtime(
    CanopenMaster *master
);

/* Change the drive-heartbeat freshness threshold after initialization.
 * Zero retains the existing semantics of disabling heartbeat age checking.
 * Only CanopenMaster may mutate its heartbeat timeout configuration.
 */
void canopen_master_set_heartbeat_timeout(
    CanopenMaster *master,
    uint32_t timeout_ms
);

/*
 * Drain a BOUNDED number of RX frames (CANOPEN_MASTER_MAX_RX_PER_POLL).
 * Excess frames remain in the CAN backend until the next owner cycle.
 * Continuous RX cannot monopolize the entire 2 ms controller task.
 * This is an operation-count bound, not measured WCET.
 *
 * Recognized traffic:
 *   - AVATAR TPDO4 feedback
 *   - AVATAR heartbeat
 *   - response to the one outstanding SDO boot/config transaction
 *
 * now_ms is supplied by the caller so this module remains OS/MCU independent.
 */
bool canopen_master_poll(
    CanopenMaster *master,
    uint32_t now_ms
);

/* NMT command broadcast to all configured AVATAR nodes. */
bool canopen_master_send_nmt_all(
    CanopenMaster *master,
    CanopenNmtCommand command
);

/*
 * Transmit one controller heartbeat at most once per period.
 * Call from the supervisor's periodic task even in PAUSED/FAULT/E-STOP,
 * and from poll() for users of the master outside that task.
 * A non-running controller cannot produce this signal.
 */
bool canopen_master_service_heartbeat(
    CanopenMaster *master,
    uint32_t now_ms
);

/* CANopen SYNC, COB-ID 0x080, DLC 0. */
bool canopen_master_send_sync(
    CanopenMaster *master
);

/*
 * Send one RPDO4 target to each axis, then one SYNC.
 *
 * This matches the AVATAR default RPDO4 transport type = 1:
 * the position cache is updated by RPDO4 and released after the next SYNC.
 */
bool canopen_master_send_target_cycle(
    CanopenMaster *master,
    const int32_t *target_positions,
    size_t target_count
);

const AvatarMDrive *canopen_master_drive(
    const CanopenMaster *master,
    size_t node_index
);

uint32_t canopen_master_tpdo_rx_count(
    const CanopenMaster *master,
    size_t node_index
);

bool canopen_master_all_feedback_valid(
    const CanopenMaster *master
);

bool canopen_master_all_heartbeats_operational(
    const CanopenMaster *master
);

bool canopen_master_all_drives_operation_enabled(
    const CanopenMaster *master
);

/*
 * Communication health = every node has produced an OPERATIONAL heartbeat,
 * no node reports AVATAR alarm state, and heartbeat age is within timeout
 * when timeout checking is enabled.
 */
bool canopen_master_healthy(
    const CanopenMaster *master,
    uint32_t now_ms
);

/*
 * Motion-ready is intentionally stricter than communication health:
 * current TPDO4 feedback must exist and every CiA-402 drive must decode as
 * OPERATION ENABLED.
 */
bool canopen_master_ready_for_motion(
    const CanopenMaster *master,
    uint32_t now_ms
);

/* ------------------------------------------------------------------------- */
/* Nonblocking boot/configuration SDO transaction API                        */
/* ------------------------------------------------------------------------- */

CanopenMasterSdoState canopen_master_sdo_state(
    const CanopenMaster *master
);

const CanopenSdoResponse *canopen_master_sdo_response(
    const CanopenMaster *master
);

void canopen_master_sdo_clear(
    CanopenMaster *master
);

bool canopen_master_begin_set_interpolation_mode(
    CanopenMaster *master,
    size_t node_index,
    uint32_t now_ms
);

bool canopen_master_begin_read_work_mode(
    CanopenMaster *master,
    size_t node_index,
    uint32_t now_ms
);

bool canopen_master_begin_read_mode_display(
    CanopenMaster *master,
    size_t node_index,
    uint32_t now_ms
);

/* Read AVATAR identity record 0x1018 sub-index 1..4. */
bool canopen_master_begin_read_identity(
    CanopenMaster *master,
    size_t node_index,
    uint8_t subindex,
    uint32_t now_ms
);

bool canopen_master_begin_set_heartbeat_period(
    CanopenMaster *master,
    size_t node_index,
    uint16_t heartbeat_period_ms,
    uint32_t now_ms
);

/* AVATAR 0x1016:01: configure and verify heartbeat consumer (UINT32). */
bool canopen_master_begin_set_heartbeat_consumer(
    CanopenMaster *master,
    size_t node_index,
    uint32_t now_ms
);

bool canopen_master_begin_read_heartbeat_consumer(
    CanopenMaster *master,
    size_t node_index,
    uint32_t now_ms
);

bool canopen_master_begin_write_controlword(
    CanopenMaster *master,
    size_t node_index,
    uint16_t controlword,
    uint32_t now_ms
);

bool canopen_master_begin_read_statusword(
    CanopenMaster *master,
    size_t node_index,
    uint32_t now_ms
);

bool canopen_master_begin_read_actual_position(
    CanopenMaster *master,
    size_t node_index,
    uint32_t now_ms
);

bool canopen_master_begin_read_alarm(
    CanopenMaster *master,
    size_t node_index,
    uint32_t now_ms
);

#endif /* STM32_CANOPEN_MASTER_H */
