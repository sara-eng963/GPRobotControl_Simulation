#include "avatar_m_drive.h"

#include "avatar_m_registers.h"
#include "../../CANComm/CANopen/canopen_heartbeat.h"
#include "../../CANComm/CANopen/canopen_ids.h"
#include "../../CANComm/CANopen/canopen_sdo.h"
#include "../CiA402/cia402.h"

#include <stddef.h>
#include <string.h>

bool avatar_m_drive_init(
    AvatarMDrive *drive,
    uint8_t node_id
)
{
    if (
        drive == NULL ||
        !canopen_node_id_valid(node_id)
    )
    {
        return false;
    }

    memset(drive, 0, sizeof(*drive));

    drive->node_id = node_id;
    drive->cia402_state = CIA402_STATE_UNKNOWN;
    drive->heartbeat_state = AVATAR_M_HEARTBEAT_STATE_UNKNOWN;

    return true;
}

bool avatar_m_drive_build_nmt(
    const AvatarMDrive *drive,
    CanopenNmtCommand command,
    CanFrame *frame
)
{
    if (drive == NULL)
    {
        return false;
    }

    return canopen_nmt_build(
        command,
        drive->node_id,
        frame
    );
}

bool avatar_m_drive_build_set_interpolation_mode(
    const AvatarMDrive *drive,
    CanFrame *frame
)
{
    if (drive == NULL)
    {
        return false;
    }

    return canopen_sdo_build_write_u8(
        drive->node_id,
        AVATAR_M_OD_MODES_OF_OPERATION,
        AVATAR_M_SUBINDEX_0,
        (uint8_t)AVATAR_M_MODE_INTERPOLATION,
        frame
    );
}

bool avatar_m_drive_build_read_work_mode(
    const AvatarMDrive *drive,
    CanFrame *frame
)
{
    if (drive == NULL)
    {
        return false;
    }

    return canopen_sdo_build_read(
        drive->node_id,
        AVATAR_M_OD_MODES_OF_OPERATION,
        AVATAR_M_SUBINDEX_0,
        frame
    );
}

bool avatar_m_drive_build_read_mode_display(
    const AvatarMDrive *drive,
    CanFrame *frame
)
{
    if (drive == NULL)
    {
        return false;
    }

    return canopen_sdo_build_read(
        drive->node_id,
        AVATAR_M_OD_MODES_OF_OPERATION_DISPLAY,
        AVATAR_M_SUBINDEX_0,
        frame
    );
}

bool avatar_m_drive_build_set_heartbeat_period(
    const AvatarMDrive *drive,
    uint16_t heartbeat_period_ms,
    CanFrame *frame
)
{
    if (drive == NULL)
    {
        return false;
    }

    return canopen_sdo_build_write_u16(
        drive->node_id,
        AVATAR_M_OD_HEARTBEAT_PRODUCER_TIME,
        AVATAR_M_SUBINDEX_0,
        heartbeat_period_ms,
        frame
    );
}

bool avatar_m_drive_build_write_controlword(
    const AvatarMDrive *drive,
    uint16_t controlword,
    CanFrame *frame
)
{
    if (drive == NULL)
    {
        return false;
    }

    return canopen_sdo_build_write_u16(
        drive->node_id,
        AVATAR_M_OD_CONTROLWORD,
        AVATAR_M_SUBINDEX_0,
        controlword,
        frame
    );
}

bool avatar_m_drive_build_read_statusword(
    const AvatarMDrive *drive,
    CanFrame *frame
)
{
    if (drive == NULL)
    {
        return false;
    }

    return canopen_sdo_build_read(
        drive->node_id,
        AVATAR_M_OD_STATUSWORD,
        AVATAR_M_SUBINDEX_0,
        frame
    );
}

bool avatar_m_drive_build_read_actual_position(
    const AvatarMDrive *drive,
    CanFrame *frame
)
{
    if (drive == NULL)
    {
        return false;
    }

    return canopen_sdo_build_read(
        drive->node_id,
        AVATAR_M_OD_POSITION_ACTUAL_VALUE,
        AVATAR_M_SUBINDEX_0,
        frame
    );
}

bool avatar_m_drive_build_read_alarm(
    const AvatarMDrive *drive,
    CanFrame *frame
)
{
    if (drive == NULL)
    {
        return false;
    }

    return canopen_sdo_build_read(
        drive->node_id,
        AVATAR_M_OD_ALARM,
        AVATAR_M_SUBINDEX_0,
        frame
    );
}

bool avatar_m_drive_build_rpdo4_target(
    const AvatarMDrive *drive,
    int32_t target_position,
    CanFrame *frame
)
{
    if (drive == NULL)
    {
        return false;
    }

    return avatar_m_build_rpdo4(
        drive->node_id,
        target_position,
        frame
    );
}

bool avatar_m_drive_process_tpdo4(
    AvatarMDrive *drive,
    const CanFrame *frame
)
{
    if (
        drive == NULL ||
        frame == NULL
    )
    {
        return false;
    }

    AvatarMFeedback feedback;

    if (!avatar_m_parse_tpdo4(
            drive->node_id,
            frame,
            &feedback))
    {
        return false;
    }

    drive->feedback = feedback;
    drive->feedback_valid = true;
    drive->cia402_state = cia402_get_state(feedback.statusword);

    return true;
}

bool avatar_m_drive_process_heartbeat(
    AvatarMDrive *drive,
    const CanFrame *frame
)
{
    if (
        drive == NULL ||
        frame == NULL
    )
    {
        return false;
    }

    CanopenHeartbeat heartbeat;

    if (!canopen_heartbeat_parse(frame, &heartbeat))
    {
        return false;
    }

    if (heartbeat.node_id != drive->node_id)
    {
        return false;
    }

    drive->heartbeat_seen = true;
    drive->heartbeat_raw = heartbeat.state;

    switch (heartbeat.state)
    {
        case CANOPEN_HEARTBEAT_BOOTUP:
            drive->heartbeat_state = AVATAR_M_HEARTBEAT_STATE_BOOTUP;
            break;

        case CANOPEN_HEARTBEAT_OPERATIONAL:
            drive->heartbeat_state = AVATAR_M_HEARTBEAT_STATE_OPERATIONAL;
            break;

        case CANOPEN_HEARTBEAT_PRE_OPERATIONAL:
            drive->heartbeat_state = AVATAR_M_HEARTBEAT_STATE_PRE_OPERATIONAL;
            break;

        case AVATAR_M_HEARTBEAT_ALARM:
            drive->heartbeat_state = AVATAR_M_HEARTBEAT_STATE_ALARM;
            break;

        default:
            drive->heartbeat_state = AVATAR_M_HEARTBEAT_STATE_UNKNOWN;
            break;
    }

    return true;
}

bool avatar_m_drive_heartbeat_is_alarm(
    const AvatarMDrive *drive
)
{
    return
        drive != NULL &&
        drive->heartbeat_seen &&
        drive->heartbeat_state == AVATAR_M_HEARTBEAT_STATE_ALARM;
}
