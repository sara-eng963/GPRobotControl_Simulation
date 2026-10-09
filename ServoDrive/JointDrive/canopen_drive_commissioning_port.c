#include "canopen_drive_commissioning_port.h"

#include "../AvatarM/avatar_m_registers.h"

static bool initialized(void *context)
{
    const CanopenMaster *master = (const CanopenMaster *)context;
    return master != NULL && master->initialized;
}

static bool configured(void *context)
{
    const CanopenMaster *master = (const CanopenMaster *)context;
    if (master == NULL || !master->initialized ||
        master->node_count != JOINT_DRIVE_AXES)
        return false;

    for (size_t axis = 0U; axis < JOINT_DRIVE_AXES; ++axis) {
        const AvatarMDrive *drive = canopen_master_drive(master, axis);
        if (drive == NULL || drive->node_id != (uint8_t)(axis + 1U))
            return false;
    }
    return true;
}

static void reset_runtime(void *context, uint32_t heartbeat_timeout_ms)
{
    CanopenMaster *master = (CanopenMaster *)context;
    canopen_master_clear_runtime(master);
    canopen_master_set_heartbeat_timeout(master, heartbeat_timeout_ms);
}

static bool send_network_command(void *context, DriveNetworkCommand command)
{
    CanopenNmtCommand nmt;
    switch (command) {
        case DRIVE_NETWORK_RESET_COMMUNICATION:
            nmt = CANOPEN_NMT_RESET_COMMUNICATION;
            break;
        case DRIVE_NETWORK_START:
            nmt = CANOPEN_NMT_START;
            break;
        default:
            return false;
    }
    return canopen_master_send_nmt_all((CanopenMaster *)context, nmt);
}

static bool all_heartbeat_state(void *context, DriveNodeHeartbeatState expected)
{
    const CanopenMaster *master = (const CanopenMaster *)context;
    AvatarMHeartbeatState avatar_expected;
    switch (expected) {
        case DRIVE_NODE_BOOTUP:
            avatar_expected = AVATAR_M_HEARTBEAT_STATE_BOOTUP;
            break;
        case DRIVE_NODE_OPERATIONAL:
            avatar_expected = AVATAR_M_HEARTBEAT_STATE_OPERATIONAL;
            break;
        default:
            return false;
    }
    if (!configured(context)) return false;
    for (size_t axis = 0U; axis < JOINT_DRIVE_AXES; ++axis) {
        const AvatarMDrive *drive = canopen_master_drive(master, axis);
        if (drive == NULL || !drive->heartbeat_seen ||
            drive->heartbeat_state != avatar_expected)
            return false;
    }
    return true;
}

static bool begin_operation(void *context, DriveCommissionOperation operation,
                            size_t axis, uint32_t argument, uint32_t now_ms)
{
    CanopenMaster *master = (CanopenMaster *)context;
    if (!configured(context) || axis >= JOINT_DRIVE_AXES) return false;
    switch (operation) {
        case DRIVE_COMMISSION_READ_IDENTITY:
            if (argument < 1U || argument > 2U) return false;
            return canopen_master_begin_read_identity(
                master, axis, (uint8_t)argument, now_ms);
        case DRIVE_COMMISSION_SET_HEARTBEAT_PERIOD:
            if (argument > UINT16_MAX) return false;
            return canopen_master_begin_set_heartbeat_period(
                master, axis, (uint16_t)argument, now_ms);
        case DRIVE_COMMISSION_SET_HEARTBEAT_CONSUMER:
            return canopen_master_begin_set_heartbeat_consumer(master, axis, now_ms);
        case DRIVE_COMMISSION_READ_HEARTBEAT_CONSUMER:
            return canopen_master_begin_read_heartbeat_consumer(master, axis, now_ms);
        case DRIVE_COMMISSION_SET_INTERPOLATION_MODE:
            return canopen_master_begin_set_interpolation_mode(master, axis, now_ms);
        case DRIVE_COMMISSION_READ_MODE_DISPLAY:
            return canopen_master_begin_read_mode_display(master, axis, now_ms);
        case DRIVE_COMMISSION_READ_STATUSWORD:
            return canopen_master_begin_read_statusword(master, axis, now_ms);
        case DRIVE_COMMISSION_WRITE_CONTROLWORD:
            if (argument > UINT16_MAX) return false;
            return canopen_master_begin_write_controlword(
                master, axis, (uint16_t)argument, now_ms);
        case DRIVE_COMMISSION_READ_ACTUAL_POSITION:
            return canopen_master_begin_read_actual_position(master, axis, now_ms);
        default:
            return false;
    }
}

static DriveOperationResult operation_result(void *context)
{
    const CanopenMaster *master = (const CanopenMaster *)context;
    const CanopenMasterSdoState sdo = canopen_master_sdo_state(master);
    DriveOperationResult result = {0};
    switch (sdo) {
        case CANOPEN_MASTER_SDO_IDLE:            result.status = DRIVE_OPERATION_IDLE; break;
        case CANOPEN_MASTER_SDO_PENDING:         result.status = DRIVE_OPERATION_PENDING; break;
        case CANOPEN_MASTER_SDO_COMPLETE:        result.status = DRIVE_OPERATION_COMPLETE; break;
        case CANOPEN_MASTER_SDO_ABORT:           result.status = DRIVE_OPERATION_ABORT; break;
        case CANOPEN_MASTER_SDO_TIMEOUT:         result.status = DRIVE_OPERATION_TIMEOUT; break;
        case CANOPEN_MASTER_SDO_TRANSPORT_ERROR: result.status = DRIVE_OPERATION_TRANSPORT_ERROR; break;
        default:                                 result.status = DRIVE_OPERATION_TRANSPORT_ERROR; break;
    }
    const CanopenSdoResponse *response = canopen_master_sdo_response(master);
    if (response != NULL) {
        result.has_response = true;
        result.is_read = response->type == CANOPEN_SDO_RESPONSE_READ;
        result.is_write_ok = response->type == CANOPEN_SDO_RESPONSE_WRITE_OK;
        result.data_size = response->data_size;
        result.value = response->value;
        result.abort_code = response->abort_code;
    }
    return result;
}

static void clear_operation(void *context)
{
    canopen_master_sdo_clear((CanopenMaster *)context);
}

DriveCommissioningPort canopen_drive_commissioning_port_make(CanopenMaster *master)
{
    const DriveCommissioningPort port = {
        .context = master,
        .is_initialized = initialized,
        .is_configured = configured,
        .reset_runtime = reset_runtime,
        .send_network_command = send_network_command,
        .all_heartbeat_state = all_heartbeat_state,
        .begin_operation = begin_operation,
        .operation_result = operation_result,
        .clear_operation = clear_operation,
        .expected_vendor_id = AVATAR_M_EXPECTED_VENDOR_ID,
        .expected_product_code = AVATAR_M_EXPECTED_PRODUCT_CODE,
        .expected_heartbeat_consumer = CANOPEN_DRIVE_CONSUMER_VALUE,
        .expected_mode_display = AVATAR_M_MODE_INTERPOLATION,
    };
    return port;
}
