#ifndef CANOPEN_MASTER_TYPES_H
#define CANOPEN_MASTER_TYPES_H

/*
 * Canonical CANopen master type contract for both STM32 CM7 and desktop.
 * This header owns all CanopenMaster-related enum/struct/callback types.
 * All consumers, including the production master and the real drive ports,
 * must compile this single declaration; platform behavior remains in .c.
 *
 * drive_command_permission is stored in both environments to preserve the
 * ABI, but the desktop implementation does NOT enforce the firmware C2B
 * authorization policy. Only the STM32 CANopen master implements that veto.
 */
#include "../can_backend.h"
#include "canopen_sdo.h"
#include "../../ServoDrive/AvatarM/avatar_m_drive.h"
#include "../../ServoDrive/JointDrive/joint_drive_port.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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

    uint8_t node_ids[JOINT_DRIVE_AXES];
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

/* RTOS-independent callback: runtime access must originate from its one owner. */
typedef bool (*CanopenMasterOwnerCheck)(void *context);

typedef struct {
    int32_t value;
    uint64_t timestamp_us;
    bool valid;
} CanopenTpdoLatest;

typedef struct
{
    CanBackend *backend;

    AvatarMDrive drives[JOINT_DRIVE_AXES];
    uint8_t node_count;

    uint32_t heartbeat_timeout_ms;
    uint32_t sdo_timeout_ms;

    /* Host-produced heartbeat is independent of any one motion state. */
    uint32_t last_controller_heartbeat_ms;
    bool controller_heartbeat_sent;

    uint32_t last_heartbeat_ms[JOINT_DRIVE_AXES];
    bool heartbeat_timestamp_valid[JOINT_DRIVE_AXES];

    /* Monotonic per-node TPDO4 receive counters for cyclic freshness checks. */
    uint32_t tpdo_rx_count[JOINT_DRIVE_AXES];
    CanopenTpdoLatest latest_tpdo[JOINT_DRIVE_AXES];

    CanopenMasterSdoTransaction sdo;

    /* Software veto only. Cleared at init and every new NMT reset cycle. */
    bool drive_command_permission;

    bool initialized;

    /* Bound once by the controller task after initialization and before use. */
    CanopenMasterOwnerCheck owner_check;
    void *owner_check_context;

} CanopenMaster;

#endif /* CANOPEN_MASTER_TYPES_H */
