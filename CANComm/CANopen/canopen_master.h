#ifndef CANOPEN_MASTER_H
#define CANOPEN_MASTER_H

#include "../can_backend.h"
#include "canopen_nmt.h"
#include "canopen_sdo.h"
#include "../../ServoDrive/AvatarM/avatar_m_drive.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CANOPEN_MASTER_MAX_NODES 6U

typedef enum
{
    CANOPEN_MASTER_SDO_IDLE = 0,
    CANOPEN_MASTER_SDO_PENDING,
    CANOPEN_MASTER_SDO_COMPLETE,
    CANOPEN_MASTER_SDO_ABORT,
    CANOPEN_MASTER_SDO_TIMEOUT,
    CANOPEN_MASTER_SDO_TRANSPORT_ERROR
} CanopenMasterSdoState;

typedef struct
{
    CanBackend *backend;

    uint8_t node_ids[CANOPEN_MASTER_MAX_NODES];
    uint8_t node_count;

    /*
     * Heartbeat age limit used by canopen_master_healthy().
     * Set to zero to require the state but disable age checking.
     */
    uint32_t heartbeat_timeout_ms;

    /* One boot/configuration SDO transaction is outstanding at a time. */
    uint32_t sdo_timeout_ms;

} CanopenMasterConfig;

typedef struct
{
    CanopenMasterSdoState state;

    uint8_t node_index;
    uint16_t index;
    uint8_t subindex;

    bool expects_read;

    uint32_t started_ms;

    CanopenSdoResponse response;

} CanopenMasterSdoTransaction;

typedef struct
{
    CanBackend *backend;

    AvatarMDrive drives[CANOPEN_MASTER_MAX_NODES];
    uint8_t node_count;

    uint32_t heartbeat_timeout_ms;
    uint32_t sdo_timeout_ms;

    uint32_t last_heartbeat_ms[CANOPEN_MASTER_MAX_NODES];
    bool heartbeat_timestamp_valid[CANOPEN_MASTER_MAX_NODES];

    CanopenMasterSdoTransaction sdo;

    bool initialized;

} CanopenMaster;

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
 * Drain all currently available RX frames.
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

bool canopen_master_begin_set_heartbeat_period(
    CanopenMaster *master,
    size_t node_index,
    uint16_t heartbeat_period_ms,
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

#endif /* CANOPEN_MASTER_H */
