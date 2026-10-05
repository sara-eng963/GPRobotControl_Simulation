#include "../CANopen/canopen_heartbeat.h"
#include "../CANopen/canopen_nmt.h"
#include "../CANopen/canopen_sdo.h"

#include <stdio.h>
#include <string.h>

static int failures = 0;

#define CHECK(condition, message) \
    do \
    { \
        if (condition) \
        { \
            printf("[PASS] %s\n", message); \
        } \
        else \
        { \
            printf("[FAIL] %s\n", message); \
            failures++; \
        } \
    } while (0)

static bool frame_matches(
    const CanFrame *frame,
    uint16_t id,
    uint8_t dlc,
    const uint8_t *data
)
{
    return
        frame->id == id &&
        frame->dlc == dlc &&
        memcmp(frame->data, data, dlc) == 0;
}

int main(void)
{
    CanFrame frame = {0};

    const uint8_t nmt_start_node1[] = {0x01U, 0x01U};

    CHECK(
        canopen_nmt_build(
            CANOPEN_NMT_START,
            1U,
            &frame
        ) &&
        frame_matches(
            &frame,
            0x000U,
            2U,
            nmt_start_node1
        ),
        "NMT Start node 1 = 000 [2] 01 01"
    );

    const uint8_t nmt_preop_broadcast[] = {0x80U, 0x00U};

    CHECK(
        canopen_nmt_build(
            CANOPEN_NMT_PRE_OPERATIONAL,
            0U,
            &frame
        ) &&
        frame_matches(
            &frame,
            0x000U,
            2U,
            nmt_preop_broadcast
        ),
        "NMT Pre-operational broadcast supported"
    );

    /* Manual format: read 0x6041:00 from node 1. */
    const uint8_t sdo_read_statusword[] = {
        0x40U, 0x41U, 0x60U, 0x00U,
        0x00U, 0x00U, 0x00U, 0x00U
    };

    CHECK(
        canopen_sdo_build_read(
            1U,
            0x6041U,
            0x00U,
            &frame
        ) &&
        frame_matches(
            &frame,
            0x601U,
            8U,
            sdo_read_statusword
        ),
        "SDO read request uses 0x600 + node"
    );

    /* Manual example: 0x6060:00 = 1 (position mode). */
    const uint8_t sdo_write_mode[] = {
        0x2FU, 0x60U, 0x60U, 0x00U,
        0x01U, 0x00U, 0x00U, 0x00U
    };

    CHECK(
        canopen_sdo_build_write_u8(
            1U,
            0x6060U,
            0x00U,
            0x01U,
            &frame
        ) &&
        frame_matches(
            &frame,
            0x601U,
            8U,
            sdo_write_mode
        ),
        "SDO 1-byte write uses command 0x2F"
    );

    /* Manual example: 0x6040:00 = 0x000F. */
    const uint8_t sdo_write_controlword[] = {
        0x2BU, 0x40U, 0x60U, 0x00U,
        0x0FU, 0x00U, 0x00U, 0x00U
    };

    CHECK(
        canopen_sdo_build_write_u16(
            1U,
            0x6040U,
            0x00U,
            0x000FU,
            &frame
        ) &&
        frame_matches(
            &frame,
            0x601U,
            8U,
            sdo_write_controlword
        ),
        "SDO 2-byte write uses command 0x2B"
    );

    /* Manual example: target position 50000 at 0x607A:00. */
    const uint8_t sdo_write_target[] = {
        0x23U, 0x7AU, 0x60U, 0x00U,
        0x50U, 0xC3U, 0x00U, 0x00U
    };

    CHECK(
        canopen_sdo_build_write_u32(
            1U,
            0x607AU,
            0x00U,
            50000U,
            &frame
        ) &&
        frame_matches(
            &frame,
            0x601U,
            8U,
            sdo_write_target
        ),
        "SDO 4-byte write reproduces target 50000 example"
    );

    CanopenSdoResponse response = {0};

    CanFrame statusword_reply = {
        .id = 0x581U,
        .dlc = 8U,
        .data = {
            0x4BU, 0x41U, 0x60U, 0x00U,
            0x37U, 0x04U, 0x00U, 0x00U
        }
    };

    CHECK(
        canopen_sdo_parse_response(
            1U,
            &statusword_reply,
            &response
        ) &&
        response.type == CANOPEN_SDO_RESPONSE_READ &&
        response.index == 0x6041U &&
        response.subindex == 0x00U &&
        response.data_size == 2U &&
        response.value == 0x0437U,
        "SDO read response parses statusword 0x0437"
    );

    CanFrame write_ok_reply = {
        .id = 0x581U,
        .dlc = 8U,
        .data = {
            0x60U, 0x7AU, 0x60U, 0x00U,
            0x00U, 0x00U, 0x00U, 0x00U
        }
    };

    CHECK(
        canopen_sdo_parse_response(
            1U,
            &write_ok_reply,
            &response
        ) &&
        response.type == CANOPEN_SDO_RESPONSE_WRITE_OK &&
        response.index == 0x607AU,
        "SDO write-success response uses command 0x60"
    );

    CanFrame abort_reply = {
        .id = 0x581U,
        .dlc = 8U,
        .data = {
            0x80U, 0x7AU, 0x60U, 0x00U,
            0x11U, 0x22U, 0x33U, 0x44U
        }
    };

    CHECK(
        canopen_sdo_parse_response(
            1U,
            &abort_reply,
            &response
        ) &&
        response.type == CANOPEN_SDO_RESPONSE_ABORT &&
        response.abort_code == 0x44332211U,
        "SDO abort response preserves raw abort code"
    );

    CanopenHeartbeat heartbeat = {0};

    CanFrame operational_heartbeat = {
        .id = 0x701U,
        .dlc = 1U,
        .data = {0x05U}
    };

    CHECK(
        canopen_heartbeat_parse(
            &operational_heartbeat,
            &heartbeat
        ) &&
        heartbeat.node_id == 1U &&
        heartbeat.state == 0x05U,
        "Heartbeat 0x701 / 0x05 parsed for node 1"
    );

    CanFrame preop_heartbeat = {
        .id = 0x706U,
        .dlc = 1U,
        .data = {0x7FU}
    };

    CHECK(
        canopen_heartbeat_parse(
            &preop_heartbeat,
            &heartbeat
        ) &&
        heartbeat.node_id == 6U &&
        heartbeat.state == 0x7FU,
        "Heartbeat parser preserves raw pre-operational state"
    );

    CanFrame alarm_heartbeat = {
        .id = 0x703U,
        .dlc = 1U,
        .data = {0x04U}
    };

    CHECK(
        canopen_heartbeat_parse(
            &alarm_heartbeat,
            &heartbeat
        ) &&
        heartbeat.node_id == 3U &&
        heartbeat.state == 0x04U,
        "Heartbeat parser preserves vendor state 0x04 without reinterpretation"
    );

    printf("\nTests failed: %d\n", failures);
    return failures != 0;
}
