#ifndef AVATAR_M_DRIVE_H
#define AVATAR_M_DRIVE_H

#include "avatar_m_pdo.h"
#include "../../CANComm/CANopen/canopen_nmt.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * ============================================================================
 *  AVATAR M DRIVE INTERFACE
 * ============================================================================
 *
 * Device-specific layer for the AVATAR M-Series actuator.
 *
 * Responsibilities:
 *   - select AVATAR-documented object-dictionary entries
 *   - build the drive-specific SDO/NMT requests used by the controller
 *   - build the AVATAR interpolation RPDO4 command
 *   - decode AVATAR TPDO4 feedback
 *   - interpret the AVATAR vendor heartbeat meaning
 *
 * This module deliberately does NOT know about SIL Kit, STM32 FDCAN, sockets,
 * or any other CAN transport backend. Generic CANopen framing remains under
 * CANComm/CANopen/, while generic CiA-402 state decoding remains under
 * ServoDrive/CiA402/.
 *
 * Position values stay in AVATAR raw position units here. Joint-radian
 * conversion is not introduced until the exact actuator gearing / zero / sign
 * convention is fixed.
 * ============================================================================
 */

typedef enum
{
    AVATAR_M_HEARTBEAT_STATE_UNKNOWN = 0,
    AVATAR_M_HEARTBEAT_STATE_BOOTUP,
    AVATAR_M_HEARTBEAT_STATE_OPERATIONAL,
    AVATAR_M_HEARTBEAT_STATE_PRE_OPERATIONAL,
    AVATAR_M_HEARTBEAT_STATE_ALARM
} AvatarMHeartbeatState;

typedef struct
{
    uint8_t node_id;

    AvatarMFeedback feedback;
    bool feedback_valid;

    /* Result of generic CiA-402 decoding of feedback.statusword. */
    uint16_t cia402_state;

    /* Last AVATAR heartbeat received for this node. */
    bool heartbeat_seen;
    uint8_t heartbeat_raw;
    AvatarMHeartbeatState heartbeat_state;
} AvatarMDrive;

bool avatar_m_drive_init(
    AvatarMDrive *drive,
    uint8_t node_id
);

/* Build an NMT command addressed to this drive only. */
bool avatar_m_drive_build_nmt(
    const AvatarMDrive *drive,
    CanopenNmtCommand command,
    CanFrame *frame
);

/* 0x6060:00 = 7, AVATAR CANopen interpolation mode. */
bool avatar_m_drive_build_set_interpolation_mode(
    const AvatarMDrive *drive,
    CanFrame *frame
);

/* Read back 0x6060:00 (work/mode selection object). */
bool avatar_m_drive_build_read_work_mode(
    const AvatarMDrive *drive,
    CanFrame *frame
);

/* 0x1017:00 producer heartbeat time, in milliseconds. */
bool avatar_m_drive_build_set_heartbeat_period(
    const AvatarMDrive *drive,
    uint16_t heartbeat_period_ms,
    CanFrame *frame
);

/* Write the AVATAR/CiA-402 controlword object 0x6040:00. */
bool avatar_m_drive_build_write_controlword(
    const AvatarMDrive *drive,
    uint16_t controlword,
    CanFrame *frame
);

bool avatar_m_drive_build_read_statusword(
    const AvatarMDrive *drive,
    CanFrame *frame
);

bool avatar_m_drive_build_read_actual_position(
    const AvatarMDrive *drive,
    CanFrame *frame
);

/* AVATAR vendor alarm object 0x260E:00. */
bool avatar_m_drive_build_read_alarm(
    const AvatarMDrive *drive,
    CanFrame *frame
);

/* Build node-specific RPDO4 target-position command. */
bool avatar_m_drive_build_rpdo4_target(
    const AvatarMDrive *drive,
    int32_t target_position,
    CanFrame *frame
);

/* Decode TPDO4 and update raw position, Statusword and CiA-402 state. */
bool avatar_m_drive_process_tpdo4(
    AvatarMDrive *drive,
    const CanFrame *frame
);

/*
 * Parse a heartbeat for this node and apply AVATAR vendor semantics:
 *   0x00 = boot-up
 *   0x05 = operational
 *   0x7F = pre-operational
 *   0x04 = alarm (AVATAR manual)
 *
 * The AVATAR NMT table says heartbeat is not produced while stopped, so this
 * device layer intentionally treats received 0x04 as the documented alarm
 * indication rather than generic CANopen "stopped" semantics.
 */
bool avatar_m_drive_process_heartbeat(
    AvatarMDrive *drive,
    const CanFrame *frame
);

bool avatar_m_drive_heartbeat_is_alarm(
    const AvatarMDrive *drive
);

#endif /* AVATAR_M_DRIVE_H */
