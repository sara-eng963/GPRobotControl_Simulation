#include "avatar_m_node_sim.h"

#include "../../../CANComm/CANopen/canopen_heartbeat.h"
#include "../../../CANComm/CANopen/canopen_ids.h"
#include "../../../CANComm/CANopen/canopen_nmt.h"
#include "../../CiA402/cia402.h"
#include "../avatar_m_pdo.h"
#include "../avatar_m_registers.h"

#include <stddef.h>
#include <string.h>

#define SDO_READ_REQUEST       0x40U
#define SDO_WRITE_U8           0x2FU
#define SDO_WRITE_U16          0x2BU
#define SDO_WRITE_U32          0x23U

#define SDO_READ_U8_REPLY      0x4FU
#define SDO_READ_U16_REPLY     0x4BU
#define SDO_READ_U32_REPLY     0x43U
#define SDO_WRITE_OK_REPLY     0x60U
#define SDO_ABORT_REPLY        0x80U

#define SDO_ABORT_OBJECT_NOT_EXIST 0x06020000UL

/* AVATAR manual §11.2 default: Node 127, 2000 ms. */
#define AVATAR_CONSUMER_DEFAULT 0x007F07D0UL

static uint16_t read_u16_le(
    const uint8_t *data
)
{
    return (uint16_t)(
        ((uint16_t)data[0]) |
        ((uint16_t)data[1] << 8)
    );
}

static uint32_t read_u32_le(
    const uint8_t *data
)
{
    return
        ((uint32_t)data[0]) |
        ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[3] << 24);
}

static void write_u16_le(
    uint8_t *data,
    uint16_t value
)
{
    data[0] =
        (uint8_t)(value & 0xFFU);

    data[1] =
        (uint8_t)((value >> 8) & 0xFFU);
}

static void write_u32_le(
    uint8_t *data,
    uint32_t value
)
{
    data[0] =
        (uint8_t)(value & 0xFFU);

    data[1] =
        (uint8_t)((value >> 8) & 0xFFU);

    data[2] =
        (uint8_t)((value >> 16) & 0xFFU);

    data[3] =
        (uint8_t)((value >> 24) & 0xFFU);
}

static bool nmt_targets_node(
    const AvatarMNodeSim *node,
    const CanFrame *frame
)
{
    if (
        frame->id != CANOPEN_COBID_NMT ||
        frame->dlc != 2U
    )
    {
        return false;
    }

    return
        frame->data[1] == 0U ||
        frame->data[1] == node->node_id;
}

static void build_heartbeat(
    const AvatarMNodeSim *node,
    uint8_t state,
    CanFrame *frame
)
{
    memset(
        frame,
        0,
        sizeof(*frame)
    );

    frame->id =
        canopen_heartbeat_id(
            node->node_id
        );

    frame->dlc =
        1U;

    frame->data[0] =
        state;
}

static void build_sdo_header(
    const AvatarMNodeSim *node,
    uint8_t command,
    uint16_t index,
    uint8_t subindex,
    CanFrame *response
)
{
    memset(
        response,
        0,
        sizeof(*response)
    );

    response->id =
        canopen_sdo_tx_id(
            node->node_id
        );

    response->dlc =
        8U;

    response->data[0] =
        command;

    response->data[1] =
        (uint8_t)(index & 0xFFU);

    response->data[2] =
        (uint8_t)((index >> 8) & 0xFFU);

    response->data[3] =
        subindex;
}

static AvatarMNodeResult sdo_abort(
    const AvatarMNodeSim *node,
    uint16_t index,
    uint8_t subindex,
    uint32_t abort_code,
    CanFrame *response
)
{
    build_sdo_header(
        node,
        SDO_ABORT_REPLY,
        index,
        subindex,
        response
    );

    write_u32_le(
        &response->data[4],
        abort_code
    );

    return
        AVATAR_M_NODE_RESPONSE;
}

static AvatarMNodeResult sdo_read_u8(
    const AvatarMNodeSim *node,
    uint16_t index,
    uint8_t subindex,
    uint8_t value,
    CanFrame *response
)
{
    build_sdo_header(
        node,
        SDO_READ_U8_REPLY,
        index,
        subindex,
        response
    );

    response->data[4] =
        value;

    return
        AVATAR_M_NODE_RESPONSE;
}

static AvatarMNodeResult sdo_read_u16(
    const AvatarMNodeSim *node,
    uint16_t index,
    uint8_t subindex,
    uint16_t value,
    CanFrame *response
)
{
    build_sdo_header(
        node,
        SDO_READ_U16_REPLY,
        index,
        subindex,
        response
    );

    write_u16_le(
        &response->data[4],
        value
    );

    return
        AVATAR_M_NODE_RESPONSE;
}

static AvatarMNodeResult sdo_read_u32(
    const AvatarMNodeSim *node,
    uint16_t index,
    uint8_t subindex,
    uint32_t value,
    CanFrame *response
)
{
    build_sdo_header(
        node,
        SDO_READ_U32_REPLY,
        index,
        subindex,
        response
    );

    write_u32_le(
        &response->data[4],
        value
    );

    return
        AVATAR_M_NODE_RESPONSE;
}

static AvatarMNodeResult sdo_write_ok(
    const AvatarMNodeSim *node,
    uint16_t index,
    uint8_t subindex,
    CanFrame *response
)
{
    build_sdo_header(
        node,
        SDO_WRITE_OK_REPLY,
        index,
        subindex,
        response
    );

    return
        AVATAR_M_NODE_RESPONSE;
}

static void apply_controlword(
    AvatarMNodeSim *node,
    uint16_t controlword
)
{
    const uint16_t state =
        cia402_get_state(
            node->statusword
        );

    if (
        controlword ==
        CIA402_CONTROLWORD_FAULT_RESET
    )
    {
        if (cia402_is_fault(state))
        {
            node->statusword =
                CIA402_STATE_SWITCH_ON_DISABLED;
        }

        return;
    }

    if (
        controlword ==
        CIA402_CONTROLWORD_DISABLE_VOLTAGE
    )
    {
        node->statusword =
            CIA402_STATE_SWITCH_ON_DISABLED;

        return;
    }

    if (
        controlword ==
        CIA402_CONTROLWORD_SHUTDOWN
    )
    {
        if (
            state ==
                CIA402_STATE_SWITCH_ON_DISABLED ||
            state ==
                CIA402_STATE_SWITCHED_ON ||
            state ==
                CIA402_STATE_OPERATION_ENABLED
        )
        {
            node->statusword =
                CIA402_STATE_READY_TO_SWITCH_ON;
        }

        return;
    }

    if (
        controlword ==
        CIA402_CONTROLWORD_SWITCH_ON
    )
    {
        if (
            state ==
            CIA402_STATE_READY_TO_SWITCH_ON
        )
        {
            node->statusword =
                CIA402_STATE_SWITCHED_ON;
        }
        else if (
            state ==
            CIA402_STATE_OPERATION_ENABLED
        )
        {
            node->statusword =
                CIA402_STATE_SWITCHED_ON;
        }

        return;
    }

    if (
        controlword ==
        CIA402_CONTROLWORD_ENABLE_OPERATION &&
        state ==
            CIA402_STATE_SWITCHED_ON
    )
    {
        node->statusword =
            CIA402_STATE_OPERATION_ENABLED;
    }
}

static AvatarMNodeResult handle_sdo_read(
    AvatarMNodeSim *node,
    uint16_t index,
    uint8_t subindex,
    CanFrame *response
)
{
    if (
        index ==
            AVATAR_M_OD_IDENTITY
    )
    {
        switch (subindex)
        {
            case 1U:
                return sdo_read_u32(
                    node,
                    index,
                    subindex,
                    (uint32_t)AVATAR_M_EXPECTED_VENDOR_ID,
                    response
                );

            case 2U:
                return sdo_read_u32(
                    node,
                    index,
                    subindex,
                    (uint32_t)AVATAR_M_EXPECTED_PRODUCT_CODE,
                    response
                );

            case 3U:
                return sdo_read_u32(
                    node,
                    index,
                    subindex,
                    (uint32_t)AVATAR_M_EXPECTED_VERSION,
                    response
                );

            case 4U:
                return sdo_read_u32(
                    node,
                    index,
                    subindex,
                    1U,
                    response
                );

            default:
                return sdo_abort(
                    node,
                    index,
                    subindex,
                    SDO_ABORT_OBJECT_NOT_EXIST,
                    response
                );
        }
    }

    if (
        index ==
            AVATAR_M_OD_MODES_OF_OPERATION &&
        subindex ==
            AVATAR_M_SUBINDEX_0
    )
    {
        return sdo_read_u8(
            node,
            index,
            subindex,
            node->work_mode,
            response
        );
    }

    if (
        index ==
            AVATAR_M_OD_MODES_OF_OPERATION_DISPLAY &&
        subindex ==
            AVATAR_M_SUBINDEX_0
    )
    {
        return sdo_read_u8(
            node,
            index,
            subindex,
            node->work_mode,
            response
        );
    }

    if (index == AVATAR_M_OD_HEARTBEAT_CONSUMER_TIME && subindex == 1U)
        return sdo_read_u32(node, index, subindex,
                            node->heartbeat_consumer_value, response);

    if (
        index ==
            AVATAR_M_OD_HEARTBEAT_PRODUCER_TIME &&
        subindex ==
            AVATAR_M_SUBINDEX_0
    )
    {
        return sdo_read_u16(
            node,
            index,
            subindex,
            (uint16_t)node->heartbeat_period_ms,
            response
        );
    }

    if (
        index ==
            AVATAR_M_OD_STATUSWORD &&
        subindex ==
            AVATAR_M_SUBINDEX_0
    )
    {
        return sdo_read_u16(
            node,
            index,
            subindex,
            node->statusword,
            response
        );
    }

    if (
        index ==
            AVATAR_M_OD_POSITION_ACTUAL_VALUE &&
        subindex ==
            AVATAR_M_SUBINDEX_0
    )
    {
        return sdo_read_u32(
            node,
            index,
            subindex,
            (uint32_t)node->actual_position,
            response
        );
    }

    if (
        index ==
            AVATAR_M_OD_ALARM &&
        subindex ==
            AVATAR_M_SUBINDEX_0
    )
    {
        return sdo_read_u32(
            node,
            index,
            subindex,
            node->communication_drop_alarm ? 0x20U : 0U,
            response
        );
    }

    return sdo_abort(
        node,
        index,
        subindex,
        SDO_ABORT_OBJECT_NOT_EXIST,
        response
    );
}

static AvatarMNodeResult handle_sdo_write(
    AvatarMNodeSim *node,
    const CanFrame *input,
    uint16_t index,
    uint8_t subindex,
    CanFrame *response
)
{
    if (
        index ==
            AVATAR_M_OD_MODES_OF_OPERATION &&
        subindex ==
            AVATAR_M_SUBINDEX_0 &&
        input->data[0] ==
            SDO_WRITE_U8
    )
    {
        node->work_mode =
            input->data[4];

        return sdo_write_ok(
            node,
            index,
            subindex,
            response
        );
    }

    if (index == AVATAR_M_OD_HEARTBEAT_CONSUMER_TIME &&
        subindex == 1U && input->data[0] == SDO_WRITE_U32)
    {
        const uint32_t value = read_u32_le(&input->data[4]);
        const uint8_t producer = (uint8_t)((value >> 16) & 0xFFU);
        if ((value & 0xFF000000UL) != 0U ||
            !canopen_node_id_valid(producer))
            return sdo_abort(node, index, subindex, 0x06090030UL, response);
        node->heartbeat_consumer_value = value;
        node->consumer_heartbeat_seen = false;
        node->consumer_elapsed_ms = 0U;
        node->communication_drop_alarm = false;
        return sdo_write_ok(node, index, subindex, response);
    }

    if (
        index ==
            AVATAR_M_OD_HEARTBEAT_PRODUCER_TIME &&
        subindex ==
            AVATAR_M_SUBINDEX_0 &&
        input->data[0] ==
            SDO_WRITE_U16
    )
    {
        node->heartbeat_period_ms =
            read_u16_le(
                &input->data[4]
            );

        node->heartbeat_elapsed_ms =
            0U;

        return sdo_write_ok(
            node,
            index,
            subindex,
            response
        );
    }

    if (
        index ==
            AVATAR_M_OD_CONTROLWORD &&
        subindex ==
            AVATAR_M_SUBINDEX_0 &&
        input->data[0] ==
            SDO_WRITE_U16
    )
    {
        apply_controlword(
            node,
            read_u16_le(
                &input->data[4]
            )
        );

        return sdo_write_ok(
            node,
            index,
            subindex,
            response
        );
    }

    if (
        input->data[0] ==
            SDO_WRITE_U32
    )
    {
        /*
         * No currently required 32-bit BOOT write is modeled. Keep the
         * command recognized so unsupported-object behavior remains explicit.
         */
        (void)read_u32_le(
            &input->data[4]
        );
    }

    return sdo_abort(
        node,
        index,
        subindex,
        SDO_ABORT_OBJECT_NOT_EXIST,
        response
    );
}

static AvatarMNodeResult handle_sdo(
    AvatarMNodeSim *node,
    const CanFrame *input,
    CanFrame *response
)
{
    if (
        input->id !=
            canopen_sdo_rx_id(
                node->node_id
            )
    )
    {
        return
            AVATAR_M_NODE_NO_RESPONSE;
    }

    if (
        node->nmt_state ==
            AVATAR_M_NMT_STOPPED
    )
    {
        return
            AVATAR_M_NODE_NO_RESPONSE;
    }

    if (input->dlc != 8U)
    {
        return
            AVATAR_M_NODE_INVALID_FRAME;
    }

    const uint16_t index =
        read_u16_le(
            &input->data[1]
        );

    const uint8_t subindex =
        input->data[3];

    if (
        input->data[0] ==
            SDO_READ_REQUEST
    )
    {
        return handle_sdo_read(
            node,
            index,
            subindex,
            response
        );
    }

    if (
        input->data[0] ==
            SDO_WRITE_U8 ||
        input->data[0] ==
            SDO_WRITE_U16 ||
        input->data[0] ==
            SDO_WRITE_U32
    )
    {
        return handle_sdo_write(
            node,
            input,
            index,
            subindex,
            response
        );
    }

    return
        AVATAR_M_NODE_INVALID_FRAME;
}

static AvatarMNodeResult handle_nmt(
    AvatarMNodeSim *node,
    const CanFrame *input,
    CanFrame *response
)
{
    if (!nmt_targets_node(node, input))
    {
        return AVATAR_M_NODE_NO_RESPONSE;
    }

    switch ((CanopenNmtCommand)input->data[0])
    {
        case CANOPEN_NMT_START:
            node->nmt_state =
                AVATAR_M_NMT_OPERATIONAL;

            node->heartbeat_elapsed_ms =
                0U;

            return
                AVATAR_M_NODE_NO_RESPONSE;

        case CANOPEN_NMT_STOP:
            node->nmt_state =
                AVATAR_M_NMT_STOPPED;

            node->heartbeat_elapsed_ms =
                0U;

            return
                AVATAR_M_NODE_NO_RESPONSE;

        case CANOPEN_NMT_PRE_OPERATIONAL:
            node->nmt_state =
                AVATAR_M_NMT_PRE_OPERATIONAL;

            node->heartbeat_elapsed_ms =
                0U;

            return
                AVATAR_M_NODE_NO_RESPONSE;

        /* Runtime reset clears the watchdog latch and waits for heartbeat. */
        case CANOPEN_NMT_RESET_APPLICATION:
        case CANOPEN_NMT_RESET_COMMUNICATION:
            node->consumer_heartbeat_seen = false;
            node->consumer_elapsed_ms = 0U;
            node->communication_drop_alarm = false;
            node->nmt_state =
                AVATAR_M_NMT_PRE_OPERATIONAL;

            node->heartbeat_elapsed_ms =
                0U;

            node->cached_target_valid =
                false;

            node->active_target_valid =
                false;

            build_heartbeat(
                node,
                CANOPEN_HEARTBEAT_BOOTUP,
                response
            );

            return
                AVATAR_M_NODE_RESPONSE;

        default:
            return
                AVATAR_M_NODE_INVALID_FRAME;
    }
}

static AvatarMNodeResult handle_rpdo4(
    AvatarMNodeSim *node,
    const CanFrame *input,
    CanFrame *response
)
{
    if (node->communication_drop_alarm)
        return AVATAR_M_NODE_NO_RESPONSE;
    int32_t target_position =
        0;

    if (
        node->nmt_state !=
        AVATAR_M_NMT_OPERATIONAL
    )
    {
        return
            AVATAR_M_NODE_NO_RESPONSE;
    }

    if (
        node->work_mode !=
        (uint8_t)AVATAR_M_MODE_INTERPOLATION
    )
    {
        return
            AVATAR_M_NODE_NO_RESPONSE;
    }

    if (!avatar_m_parse_rpdo4(
            node->node_id,
            input,
            &target_position))
    {
        return
            AVATAR_M_NODE_INVALID_FRAME;
    }

    node->cached_target_position =
        target_position;

    node->cached_target_valid =
        true;

    if (!avatar_m_build_tpdo4(
            node->node_id,
            node->actual_position,
            node->statusword,
            response))
    {
        return
            AVATAR_M_NODE_INVALID_FRAME;
    }

    return
        AVATAR_M_NODE_RESPONSE;
}

static AvatarMNodeResult handle_sync(
    AvatarMNodeSim *node,
    const CanFrame *input
)
{
    if (node->communication_drop_alarm)
        return AVATAR_M_NODE_NO_RESPONSE;
    if (
        input->id !=
            CANOPEN_COBID_SYNC ||
        input->dlc != 0U
    )
    {
        return
            AVATAR_M_NODE_INVALID_FRAME;
    }

    if (
        node->nmt_state !=
            AVATAR_M_NMT_OPERATIONAL ||
        node->work_mode !=
            (uint8_t)AVATAR_M_MODE_INTERPOLATION ||
        !node->cached_target_valid
    )
    {
        return
            AVATAR_M_NODE_NO_RESPONSE;
    }

    node->active_target_position =
        node->cached_target_position;

    node->active_target_valid =
        true;

    node->cached_target_valid =
        false;

    return
        AVATAR_M_NODE_NO_RESPONSE;
}

bool avatar_m_node_init(
    AvatarMNodeSim *node,
    uint8_t node_id
)
{
    if (
        node == NULL ||
        !canopen_node_id_valid(node_id)
    )
    {
        return false;
    }

    memset(
        node,
        0,
        sizeof(*node)
    );

    node->node_id =
        node_id;

    node->nmt_state =
        AVATAR_M_NMT_PRE_OPERATIONAL;

    node->heartbeat_period_ms =
        AVATAR_M_DEFAULT_HEARTBEAT_PRODUCER_MS;

    node->heartbeat_consumer_value = AVATAR_CONSUMER_DEFAULT;

    return
        true;
}

AvatarMNodeResult avatar_m_node_process_frame(
    AvatarMNodeSim *node,
    const CanFrame *input,
    CanFrame *response
)
{
    if (
        node == NULL ||
        input == NULL ||
        response == NULL
    )
    {
        return
            AVATAR_M_NODE_INVALID_FRAME;
    }

    const uint8_t producer =
        (uint8_t)((node->heartbeat_consumer_value >> 16) & 0xFFU);
    if (input->dlc == 1U &&
        canopen_node_id_valid(producer) &&
        input->id == canopen_heartbeat_id(producer))
    {
        if ((node->heartbeat_consumer_value & 0xFFFFU) != 0U &&
            input->data[0] == CANOPEN_HEARTBEAT_OPERATIONAL)
        {
            node->consumer_heartbeat_seen = true;
            node->consumer_elapsed_ms = 0U;
            node->communication_drop_alarm = false;
        }
        return AVATAR_M_NODE_NO_RESPONSE;
    }

    if (
        input->id ==
        CANOPEN_COBID_NMT
    )
    {
        return handle_nmt(
            node,
            input,
            response
        );
    }

    if (
        input->id ==
        CANOPEN_COBID_SYNC
    )
    {
        return handle_sync(
            node,
            input
        );
    }

    if (
        input->id ==
        canopen_sdo_rx_id(
            node->node_id
        )
    )
    {
        return handle_sdo(
            node,
            input,
            response
        );
    }

    if (
        input->id ==
        canopen_rpdo4_id(
            node->node_id
        )
    )
    {
        return handle_rpdo4(
            node,
            input,
            response
        );
    }

    return
        AVATAR_M_NODE_NO_RESPONSE;
}

bool avatar_m_node_tick_ms(
    AvatarMNodeSim *node,
    uint32_t elapsed_ms,
    CanFrame *heartbeat
)
{
    if (
        node == NULL ||
        heartbeat == NULL
    )
    {
        return false;
    }

    if (
        node->nmt_state ==
        AVATAR_M_NMT_STOPPED
    )
    {
        return false;
    }

    if (
        node->heartbeat_period_ms ==
        0U
    )
    {
        return false;
    }

    /* The watchdog arms only after receiving the first host heartbeat.
     * Clearing active/cached target is a simulation of the documented stop,
     * NOT an electrical or safety-rated drive-disable model. */
    if (node->consumer_heartbeat_seen &&
        (node->heartbeat_consumer_value & 0xFFFFU) != 0U)
    {
        const uint32_t deadline = node->heartbeat_consumer_value & 0xFFFFU;
        if (elapsed_ms >= deadline ||
            node->consumer_elapsed_ms >= deadline - elapsed_ms)
        {
            node->consumer_elapsed_ms = deadline;
            node->communication_drop_alarm = true;
            node->cached_target_valid = false;
            node->active_target_valid = false;
        }
        else node->consumer_elapsed_ms += elapsed_ms;
    }

    node->heartbeat_elapsed_ms +=
        elapsed_ms;

    if (
        node->heartbeat_elapsed_ms <
        node->heartbeat_period_ms
    )
    {
        return false;
    }

    node->heartbeat_elapsed_ms -=
        node->heartbeat_period_ms;

    if (
        node->nmt_state ==
        AVATAR_M_NMT_OPERATIONAL
    )
    {
        build_heartbeat(
            node,
            CANOPEN_HEARTBEAT_OPERATIONAL,
            heartbeat
        );
    }
    else
    {
        build_heartbeat(
            node,
            CANOPEN_HEARTBEAT_PRE_OPERATIONAL,
            heartbeat
        );
    }

    return true;
}

void avatar_m_node_set_actual_position(
    AvatarMNodeSim *node,
    int32_t actual_position
)
{
    if (node != NULL)
    {
        node->actual_position =
            actual_position;
    }
}

void avatar_m_node_set_statusword(
    AvatarMNodeSim *node,
    uint16_t statusword
)
{
    if (node != NULL)
    {
        node->statusword =
            statusword;
    }
}

void avatar_m_node_set_work_mode(
    AvatarMNodeSim *node,
    uint8_t work_mode
)
{
    if (node != NULL)
    {
        node->work_mode =
            work_mode;
    }
}
