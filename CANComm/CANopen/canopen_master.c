#include "canopen_master.h"

#include "canopen_heartbeat.h"
#include "canopen_ids.h"
#include "canopen_objects.h"
#include "../../ServoDrive/CiA402/cia402.h"

#include <stddef.h>
#include <string.h>

static bool node_ids_valid_and_unique(
    const CanopenMasterConfig *config
)
{
    if (
        config->node_count == 0U ||
        config->node_count > CANOPEN_MASTER_MAX_NODES
    )
    {
        return false;
    }

    for (uint8_t i = 0U; i < config->node_count; ++i)
    {
        if (!canopen_node_id_valid(config->node_ids[i]) ||
            config->node_ids[i] == CANOPEN_CONTROLLER_NODE_ID)
        {
            return false;
        }

        for (uint8_t j = 0U; j < i; ++j)
        {
            if (config->node_ids[i] == config->node_ids[j])
            {
                return false;
            }
        }
    }

    return true;
}

static int find_node_index(
    const CanopenMaster *master,
    uint8_t node_id
)
{
    if (master == NULL)
    {
        return -1;
    }

    for (uint8_t i = 0U; i < master->node_count; ++i)
    {
        if (master->drives[i].node_id == node_id)
        {
            return (int)i;
        }
    }

    return -1;
}

static bool send_frame(
    CanopenMaster *master,
    const CanFrame *frame
)
{
    return
        master != NULL &&
        master->initialized &&
        can_backend_send(
            master->backend,
            frame
        ) == CAN_BACKEND_OK;
}

bool canopen_master_init(
    CanopenMaster *master,
    const CanopenMasterConfig *config
)
{
    if (
        master == NULL ||
        config == NULL ||
        !can_backend_valid(config->backend) ||
        !node_ids_valid_and_unique(config)
    )
    {
        return false;
    }

    memset(
        master,
        0,
        sizeof(*master)
    );

    master->backend =
        config->backend;

    master->node_count =
        config->node_count;

    master->heartbeat_timeout_ms =
        config->heartbeat_timeout_ms;

    master->sdo_timeout_ms =
        config->sdo_timeout_ms;

    for (uint8_t i = 0U; i < master->node_count; ++i)
    {
        if (!avatar_m_drive_init(
                &master->drives[i],
                config->node_ids[i]))
        {
            memset(master, 0, sizeof(*master));
            return false;
        }
    }

    master->sdo.state =
        CANOPEN_MASTER_SDO_IDLE;

    master->initialized =
        true;

    return true;
}

void canopen_master_close(
    CanopenMaster *master
)
{
    if (master == NULL)
    {
        return;
    }

    if (master->backend != NULL)
    {
        can_backend_close(
            master->backend
        );
    }

    memset(
        master,
        0,
        sizeof(*master)
    );
}

void canopen_master_clear_runtime(
    CanopenMaster *master
)
{
    if (
        master == NULL ||
        !master->initialized
    )
    {
        return;
    }

    for (uint8_t i = 0U; i < master->node_count; ++i)
    {
        const uint8_t node_id =
            master->drives[i].node_id;

        (void)avatar_m_drive_init(
            &master->drives[i],
            node_id
        );

        master->last_heartbeat_ms[i] =
            0U;

        master->heartbeat_timestamp_valid[i] =
            false;

        master->tpdo_rx_count[i] =
            0U;
    }

    canopen_master_sdo_clear(
        master
    );
}

static void process_heartbeat(
    CanopenMaster *master,
    const CanFrame *frame,
    uint32_t now_ms
)
{
    CanopenHeartbeat heartbeat;

    if (!canopen_heartbeat_parse(
            frame,
            &heartbeat))
    {
        return;
    }

    const int index =
        find_node_index(
            master,
            heartbeat.node_id
        );

    if (index < 0)
    {
        return;
    }

    if (avatar_m_drive_process_heartbeat(
            &master->drives[index],
            frame))
    {
        master->last_heartbeat_ms[index] =
            now_ms;

        master->heartbeat_timestamp_valid[index] =
            true;
    }
}

static bool process_tpdo4(
    CanopenMaster *master,
    const CanFrame *frame
)
{
    for (uint8_t i = 0U; i < master->node_count; ++i)
    {
        if (
            frame->id ==
            canopen_tpdo4_id(
                master->drives[i].node_id
            )
        )
        {
            const bool parsed =
                avatar_m_drive_process_tpdo4(
                    &master->drives[i],
                    frame
                );

            if (parsed)
            {
                master->tpdo_rx_count[i]++;
            }

            return parsed;
        }
    }

    return false;
}

static void process_sdo_response(
    CanopenMaster *master,
    const CanFrame *frame
)
{
    if (
        master->sdo.state !=
        CANOPEN_MASTER_SDO_PENDING
    )
    {
        return;
    }

    const AvatarMDrive *drive =
        &master->drives[
            master->sdo.node_index
        ];

    if (
        frame->id !=
        canopen_sdo_tx_id(
            drive->node_id
        )
    )
    {
        return;
    }

    CanopenSdoResponse response;

    if (!canopen_sdo_parse_response(
            drive->node_id,
            frame,
            &response))
    {
        return;
    }

    if (
        response.index != master->sdo.index ||
        response.subindex != master->sdo.subindex
    )
    {
        return;
    }

    master->sdo.response =
        response;

    if (
        response.type ==
        CANOPEN_SDO_RESPONSE_ABORT
    )
    {
        master->sdo.state =
            CANOPEN_MASTER_SDO_ABORT;

        return;
    }

    if (
        master->sdo.expects_read &&
        response.type ==
            CANOPEN_SDO_RESPONSE_READ
    )
    {
        master->sdo.state =
            CANOPEN_MASTER_SDO_COMPLETE;

        return;
    }

    if (
        !master->sdo.expects_read &&
        response.type ==
            CANOPEN_SDO_RESPONSE_WRITE_OK
    )
    {
        master->sdo.state =
            CANOPEN_MASTER_SDO_COMPLETE;
    }
}

bool canopen_master_service_heartbeat(
    CanopenMaster *master,
    uint32_t now_ms
)
{
    if (master == NULL || !master->initialized)
    {
        return false;
    }

    if (master->controller_heartbeat_sent &&
        (uint32_t)(now_ms - master->last_controller_heartbeat_ms) <
            CANOPEN_CONTROLLER_HEARTBEAT_MS)
    {
        return true;
    }

    CanFrame frame;
    if (!canopen_heartbeat_build(
            CANOPEN_CONTROLLER_NODE_ID,
            CANOPEN_HEARTBEAT_OPERATIONAL,
            &frame) ||
        !send_frame(master, &frame))
    {
        /* Do not mark failed transmissions as sent. Retry on next tick. */
        return false;
    }

    master->last_controller_heartbeat_ms = now_ms;
    master->controller_heartbeat_sent = true;
    return true;
}

bool canopen_master_poll(
    CanopenMaster *master,
    uint32_t now_ms
)
{
    if (
        master == NULL ||
        !master->initialized
    )
    {
        return false;
    }

    if (!canopen_master_service_heartbeat(master, now_ms))
    {
        return false;
    }

    for (;;)
    {
        CanFrame frame;

        const CanBackendResult result =
            can_backend_receive(
                master->backend,
                &frame
            );

        if (
            result ==
            CAN_BACKEND_WOULD_BLOCK
        )
        {
            break;
        }

        if (
            result ==
            CAN_BACKEND_ERROR
        )
        {
            if (
                master->sdo.state ==
                CANOPEN_MASTER_SDO_PENDING
            )
            {
                master->sdo.state =
                    CANOPEN_MASTER_SDO_TRANSPORT_ERROR;
            }

            return false;
        }

        process_heartbeat(
            master,
            &frame,
            now_ms
        );

        (void)process_tpdo4(
            master,
            &frame
        );

        process_sdo_response(
            master,
            &frame
        );
    }

    if (
        master->sdo.state ==
            CANOPEN_MASTER_SDO_PENDING &&
        master->sdo_timeout_ms > 0U &&
        (uint32_t)(
            now_ms -
            master->sdo.started_ms
        ) >= master->sdo_timeout_ms
    )
    {
        master->sdo.state =
            CANOPEN_MASTER_SDO_TIMEOUT;
    }

    return true;
}

bool canopen_master_send_nmt_all(
    CanopenMaster *master,
    CanopenNmtCommand command
)
{
    if (
        master == NULL ||
        !master->initialized
    )
    {
        return false;
    }

    CanFrame frame;

    if (!canopen_nmt_build(
            command,
            0U,
            &frame))
    {
        return false;
    }

    return
        send_frame(
            master,
            &frame
        );
}

bool canopen_master_send_sync(
    CanopenMaster *master
)
{
    if (
        master == NULL ||
        !master->initialized
    )
    {
        return false;
    }

    CanFrame frame;

    memset(
        &frame,
        0,
        sizeof(frame)
    );

    frame.id =
        CANOPEN_COBID_SYNC;

    frame.dlc =
        0U;

    return
        send_frame(
            master,
            &frame
        );
}

bool canopen_master_send_target_cycle(
    CanopenMaster *master,
    const int32_t *target_positions,
    size_t target_count
)
{
    if (
        master == NULL ||
        !master->initialized ||
        target_positions == NULL ||
        target_count != master->node_count
    )
    {
        return false;
    }

    for (uint8_t i = 0U; i < master->node_count; ++i)
    {
        CanFrame frame;

        if (!avatar_m_drive_build_rpdo4_target(
                &master->drives[i],
                target_positions[i],
                &frame))
        {
            return false;
        }

        if (!send_frame(
                master,
                &frame))
        {
            return false;
        }
    }

    return
        canopen_master_send_sync(
            master
        );
}

const AvatarMDrive *canopen_master_drive(
    const CanopenMaster *master,
    size_t node_index
)
{
    if (
        master == NULL ||
        !master->initialized ||
        node_index >= master->node_count
    )
    {
        return NULL;
    }

    return
        &master->drives[node_index];
}

uint32_t canopen_master_tpdo_rx_count(
    const CanopenMaster *master,
    size_t node_index
)
{
    if (
        master == NULL ||
        !master->initialized ||
        node_index >= master->node_count
    )
    {
        return 0U;
    }

    return
        master->tpdo_rx_count[node_index];
}

bool canopen_master_all_feedback_valid(
    const CanopenMaster *master
)
{
    if (
        master == NULL ||
        !master->initialized ||
        master->node_count == 0U
    )
    {
        return false;
    }

    for (uint8_t i = 0U; i < master->node_count; ++i)
    {
        if (!master->drives[i].feedback_valid)
        {
            return false;
        }
    }

    return true;
}

bool canopen_master_all_heartbeats_operational(
    const CanopenMaster *master
)
{
    if (
        master == NULL ||
        !master->initialized ||
        master->node_count == 0U
    )
    {
        return false;
    }

    for (uint8_t i = 0U; i < master->node_count; ++i)
    {
        const AvatarMDrive *drive =
            &master->drives[i];

        if (
            !drive->heartbeat_seen ||
            drive->heartbeat_state !=
                AVATAR_M_HEARTBEAT_STATE_OPERATIONAL ||
            avatar_m_drive_heartbeat_is_alarm(drive)
        )
        {
            return false;
        }
    }

    return true;
}

bool canopen_master_all_drives_operation_enabled(
    const CanopenMaster *master
)
{
    if (!canopen_master_all_feedback_valid(master))
    {
        return false;
    }

    for (uint8_t i = 0U; i < master->node_count; ++i)
    {
        if (
            master->drives[i].cia402_state !=
            CIA402_STATE_OPERATION_ENABLED
        )
        {
            return false;
        }
    }

    return true;
}

bool canopen_master_healthy(
    const CanopenMaster *master,
    uint32_t now_ms
)
{
    if (!canopen_master_all_heartbeats_operational(master))
    {
        return false;
    }

    if (master->heartbeat_timeout_ms == 0U)
    {
        return true;
    }

    for (uint8_t i = 0U; i < master->node_count; ++i)
    {
        if (
            !master->heartbeat_timestamp_valid[i] ||
            (uint32_t)(
                now_ms -
                master->last_heartbeat_ms[i]
            ) > master->heartbeat_timeout_ms
        )
        {
            return false;
        }
    }

    return true;
}

bool canopen_master_ready_for_motion(
    const CanopenMaster *master,
    uint32_t now_ms
)
{
    return
        canopen_master_healthy(
            master,
            now_ms
        ) &&
        canopen_master_all_drives_operation_enabled(
            master
        );
}

CanopenMasterSdoState canopen_master_sdo_state(
    const CanopenMaster *master
)
{
    if (master == NULL)
    {
        return
            CANOPEN_MASTER_SDO_TRANSPORT_ERROR;
    }

    return
        master->sdo.state;
}

const CanopenSdoResponse *canopen_master_sdo_response(
    const CanopenMaster *master
)
{
    if (
        master == NULL ||
        (
            master->sdo.state !=
                CANOPEN_MASTER_SDO_COMPLETE &&
            master->sdo.state !=
                CANOPEN_MASTER_SDO_ABORT
        )
    )
    {
        return NULL;
    }

    return
        &master->sdo.response;
}

void canopen_master_sdo_clear(
    CanopenMaster *master
)
{
    if (master == NULL)
    {
        return;
    }

    memset(
        &master->sdo,
        0,
        sizeof(master->sdo)
    );

    master->sdo.state =
        CANOPEN_MASTER_SDO_IDLE;
}

static bool begin_sdo(
    CanopenMaster *master,
    size_t node_index,
    const CanFrame *frame,
    uint16_t index,
    uint8_t subindex,
    bool expects_read,
    uint32_t now_ms
)
{
    if (
        master == NULL ||
        !master->initialized ||
        frame == NULL ||
        node_index >= master->node_count ||
        master->sdo.state ==
            CANOPEN_MASTER_SDO_PENDING
    )
    {
        return false;
    }

    canopen_master_sdo_clear(
        master
    );

    if (!send_frame(
            master,
            frame))
    {
        master->sdo.state =
            CANOPEN_MASTER_SDO_TRANSPORT_ERROR;

        return false;
    }

    master->sdo.state =
        CANOPEN_MASTER_SDO_PENDING;

    master->sdo.node_index =
        (uint8_t)node_index;

    master->sdo.index =
        index;

    master->sdo.subindex =
        subindex;

    master->sdo.expects_read =
        expects_read;

    master->sdo.started_ms =
        now_ms;

    return true;
}

bool canopen_master_begin_set_interpolation_mode(
    CanopenMaster *master,
    size_t node_index,
    uint32_t now_ms
)
{
    if (
        master == NULL ||
        node_index >= master->node_count
    )
    {
        return false;
    }

    CanFrame frame;

    if (!avatar_m_drive_build_set_interpolation_mode(
            &master->drives[node_index],
            &frame))
    {
        return false;
    }

    return
        begin_sdo(
            master,
            node_index,
            &frame,
            0x6060U,
            0x00U,
            false,
            now_ms
        );
}

bool canopen_master_begin_read_work_mode(
    CanopenMaster *master,
    size_t node_index,
    uint32_t now_ms
)
{
    if (
        master == NULL ||
        node_index >= master->node_count
    )
    {
        return false;
    }

    CanFrame frame;

    if (!avatar_m_drive_build_read_work_mode(
            &master->drives[node_index],
            &frame))
    {
        return false;
    }

    return
        begin_sdo(
            master,
            node_index,
            &frame,
            0x6060U,
            0x00U,
            true,
            now_ms
        );
}

bool canopen_master_begin_read_mode_display(
    CanopenMaster *master,
    size_t node_index,
    uint32_t now_ms
)
{
    if (
        master == NULL ||
        node_index >= master->node_count
    )
    {
        return false;
    }

    CanFrame frame;

    if (!avatar_m_drive_build_read_mode_display(
            &master->drives[node_index],
            &frame))
    {
        return false;
    }

    return
        begin_sdo(
            master,
            node_index,
            &frame,
            0x6061U,
            0x00U,
            true,
            now_ms
        );
}

bool canopen_master_begin_read_identity(
    CanopenMaster *master,
    size_t node_index,
    uint8_t subindex,
    uint32_t now_ms
)
{
    if (
        master == NULL ||
        node_index >= master->node_count ||
        subindex < 1U ||
        subindex > 4U
    )
    {
        return false;
    }

    CanFrame frame;

    const uint8_t node_id =
        master->drives[node_index].node_id;

    if (!canopen_sdo_build_read(
            node_id,
            0x1018U,
            subindex,
            &frame))
    {
        return false;
    }

    return
        begin_sdo(
            master,
            node_index,
            &frame,
            0x1018U,
            subindex,
            true,
            now_ms
        );
}

bool canopen_master_begin_set_heartbeat_period(
    CanopenMaster *master,
    size_t node_index,
    uint16_t heartbeat_period_ms,
    uint32_t now_ms
)
{
    if (
        master == NULL ||
        node_index >= master->node_count
    )
    {
        return false;
    }

    CanFrame frame;

    if (!avatar_m_drive_build_set_heartbeat_period(
            &master->drives[node_index],
            heartbeat_period_ms,
            &frame))
    {
        return false;
    }

    return
        begin_sdo(
            master,
            node_index,
            &frame,
            0x1017U,
            0x00U,
            false,
            now_ms
        );
}

bool canopen_master_begin_set_heartbeat_consumer(
    CanopenMaster *master,
    size_t node_index,
    uint32_t now_ms
)
{
    if (master == NULL || node_index >= master->node_count)
    {
        return false;
    }

    CanFrame frame;
    if (!avatar_m_drive_build_set_heartbeat_consumer(
            &master->drives[node_index],
            CANOPEN_CONTROLLER_NODE_ID,
            CANOPEN_DRIVE_CONSUMER_TIMEOUT_MS,
            &frame))
    {
        return false;
    }

    return begin_sdo(
        master, node_index, &frame,
        CANOPEN_OD_HEARTBEAT_CONSUMER_TIME,
        CANOPEN_DRIVE_CONSUMER_SUBINDEX,
        false, now_ms
    );
}

bool canopen_master_begin_read_heartbeat_consumer(
    CanopenMaster *master,
    size_t node_index,
    uint32_t now_ms
)
{
    if (master == NULL || node_index >= master->node_count)
    {
        return false;
    }

    CanFrame frame;
    if (!avatar_m_drive_build_read_heartbeat_consumer(
            &master->drives[node_index], &frame))
    {
        return false;
    }

    return begin_sdo(
        master, node_index, &frame,
        CANOPEN_OD_HEARTBEAT_CONSUMER_TIME,
        CANOPEN_DRIVE_CONSUMER_SUBINDEX,
        true, now_ms
    );
}

bool canopen_master_begin_write_controlword(
    CanopenMaster *master,
    size_t node_index,
    uint16_t controlword,
    uint32_t now_ms
)
{
    if (
        master == NULL ||
        node_index >= master->node_count
    )
    {
        return false;
    }

    CanFrame frame;

    if (!avatar_m_drive_build_write_controlword(
            &master->drives[node_index],
            controlword,
            &frame))
    {
        return false;
    }

    return
        begin_sdo(
            master,
            node_index,
            &frame,
            0x6040U,
            0x00U,
            false,
            now_ms
        );
}

bool canopen_master_begin_read_statusword(
    CanopenMaster *master,
    size_t node_index,
    uint32_t now_ms
)
{
    if (
        master == NULL ||
        node_index >= master->node_count
    )
    {
        return false;
    }

    CanFrame frame;

    if (!avatar_m_drive_build_read_statusword(
            &master->drives[node_index],
            &frame))
    {
        return false;
    }

    return
        begin_sdo(
            master,
            node_index,
            &frame,
            0x6041U,
            0x00U,
            true,
            now_ms
        );
}

bool canopen_master_begin_read_actual_position(
    CanopenMaster *master,
    size_t node_index,
    uint32_t now_ms
)
{
    if (
        master == NULL ||
        node_index >= master->node_count
    )
    {
        return false;
    }

    CanFrame frame;

    if (!avatar_m_drive_build_read_actual_position(
            &master->drives[node_index],
            &frame))
    {
        return false;
    }

    return
        begin_sdo(
            master,
            node_index,
            &frame,
            0x6064U,
            0x00U,
            true,
            now_ms
        );
}

bool canopen_master_begin_read_alarm(
    CanopenMaster *master,
    size_t node_index,
    uint32_t now_ms
)
{
    if (
        master == NULL ||
        node_index >= master->node_count
    )
    {
        return false;
    }

    CanFrame frame;

    if (!avatar_m_drive_build_read_alarm(
            &master->drives[node_index],
            &frame))
    {
        return false;
    }

    return
        begin_sdo(
            master,
            node_index,
            &frame,
            0x260EU,
            0x00U,
            true,
            now_ms
        );
}
