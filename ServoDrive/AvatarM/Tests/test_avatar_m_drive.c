#include "../avatar_m_drive.h"
#include "../avatar_m_registers.h"
#include "../../CiA402/cia402.h"

#include <stdbool.h>
#include <stdint.h>
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

static bool bytes_equal(
    const uint8_t *actual,
    const uint8_t *expected,
    size_t count
)
{
    return memcmp(actual, expected, count) == 0;
}

int main(void)
{
    AvatarMDrive drive;

    CHECK(
        avatar_m_drive_init(&drive, 1U),
        "AVATAR M drive node 1 initialized"
    );

    CHECK(
        drive.node_id == 1U &&
        !drive.feedback_valid &&
        !drive.heartbeat_seen &&
        drive.cia402_state == CIA402_STATE_UNKNOWN,
        "Drive starts with no invented feedback/state"
    );

    CanFrame frame;

    CHECK(
        avatar_m_drive_build_nmt(
            &drive,
            CANOPEN_NMT_START,
            &frame
        ) &&
        frame.id == 0x000U &&
        frame.dlc == 2U &&
        frame.data[0] == 0x01U &&
        frame.data[1] == 0x01U,
        "Drive builds node-specific NMT Start"
    );

    {
        const uint8_t expected[8] =
            {0x2FU, 0x60U, 0x60U, 0x00U, 0x07U, 0x00U, 0x00U, 0x00U};

        CHECK(
            avatar_m_drive_build_set_interpolation_mode(
                &drive,
                &frame
            ) &&
            frame.id == 0x601U &&
            frame.dlc == 8U &&
            bytes_equal(frame.data, expected, sizeof(expected)),
            "Interpolation mode writes 0x6060:00 = 7"
        );
    }

    {
        const uint8_t expected[8] =
            {0x40U, 0x60U, 0x60U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};

        CHECK(
            avatar_m_drive_build_read_work_mode(
                &drive,
                &frame
            ) &&
            frame.id == 0x601U &&
            frame.dlc == 8U &&
            bytes_equal(frame.data, expected, sizeof(expected)),
            "Work mode read targets 0x6060:00"
        );
    }

    {
        const uint8_t expected[8] =
            {0x2BU, 0x17U, 0x10U, 0x00U, 0xE8U, 0x03U, 0x00U, 0x00U};

        CHECK(
            avatar_m_drive_build_set_heartbeat_period(
                &drive,
                AVATAR_M_DEFAULT_HEARTBEAT_PRODUCER_MS,
                &frame
            ) &&
            frame.id == 0x601U &&
            frame.dlc == 8U &&
            bytes_equal(frame.data, expected, sizeof(expected)),
            "Heartbeat period writes 0x1017:00 = 1000 ms"
        );
    }

    {
        const uint8_t expected[8] =
            {0x2BU, 0x40U, 0x60U, 0x00U, 0x0FU, 0x00U, 0x00U, 0x00U};

        CHECK(
            avatar_m_drive_build_write_controlword(
                &drive,
                0x000FU,
                &frame
            ) &&
            frame.id == 0x601U &&
            frame.dlc == 8U &&
            bytes_equal(frame.data, expected, sizeof(expected)),
            "Controlword write targets 0x6040:00"
        );
    }

    {
        const uint8_t expected[8] =
            {0x40U, 0x41U, 0x60U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};

        CHECK(
            avatar_m_drive_build_read_statusword(
                &drive,
                &frame
            ) &&
            bytes_equal(frame.data, expected, sizeof(expected)),
            "Statusword read targets 0x6041:00"
        );
    }

    {
        const uint8_t expected[8] =
            {0x40U, 0x64U, 0x60U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};

        CHECK(
            avatar_m_drive_build_read_actual_position(
                &drive,
                &frame
            ) &&
            bytes_equal(frame.data, expected, sizeof(expected)),
            "Actual-position read targets 0x6064:00"
        );
    }

    {
        const uint8_t expected[8] =
            {0x40U, 0x0EU, 0x26U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};

        CHECK(
            avatar_m_drive_build_read_alarm(
                &drive,
                &frame
            ) &&
            bytes_equal(frame.data, expected, sizeof(expected)),
            "AVATAR alarm read targets vendor object 0x260E:00"
        );
    }

    {
        const uint8_t expected[4] =
            {0x50U, 0xC3U, 0x00U, 0x00U};

        CHECK(
            avatar_m_drive_build_rpdo4_target(
                &drive,
                50000,
                &frame
            ) &&
            frame.id == 0x501U &&
            frame.dlc == 4U &&
            bytes_equal(frame.data, expected, sizeof(expected)),
            "Drive builds AVATAR RPDO4 target 50000"
        );
    }

    /* Manual interpolation feedback example: actual 3448, Statusword 0x0437. */
    memset(&frame, 0, sizeof(frame));
    frame.id = 0x481U;
    frame.dlc = 6U;
    frame.data[0] = 0x78U;
    frame.data[1] = 0x0DU;
    frame.data[2] = 0x00U;
    frame.data[3] = 0x00U;
    frame.data[4] = 0x37U;
    frame.data[5] = 0x04U;

    CHECK(
        avatar_m_drive_process_tpdo4(
            &drive,
            &frame
        ),
        "TPDO4 feedback accepted for node 1"
    );

    CHECK(
        drive.feedback_valid &&
        drive.feedback.actual_position == 3448 &&
        drive.feedback.statusword == 0x0437U,
        "TPDO4 updates raw AVATAR feedback"
    );

    CHECK(
        drive.cia402_state == CIA402_STATE_OPERATION_ENABLED,
        "Statusword 0x0437 decodes through generic CiA-402 layer"
    );

    memset(&frame, 0, sizeof(frame));
    frame.id = 0x701U;
    frame.dlc = 1U;
    frame.data[0] = 0x05U;

    CHECK(
        avatar_m_drive_process_heartbeat(
            &drive,
            &frame
        ) &&
        drive.heartbeat_state == AVATAR_M_HEARTBEAT_STATE_OPERATIONAL &&
        !avatar_m_drive_heartbeat_is_alarm(&drive),
        "AVATAR heartbeat 0x05 interpreted as Operational"
    );

    frame.data[0] = 0x04U;

    CHECK(
        avatar_m_drive_process_heartbeat(
            &drive,
            &frame
        ) &&
        drive.heartbeat_state == AVATAR_M_HEARTBEAT_STATE_ALARM &&
        avatar_m_drive_heartbeat_is_alarm(&drive),
        "AVATAR vendor heartbeat 0x04 interpreted as alarm"
    );

    frame.id = 0x702U;
    frame.data[0] = 0x05U;

    CHECK(
        !avatar_m_drive_process_heartbeat(
            &drive,
            &frame
        ),
        "Drive rejects heartbeat belonging to another node"
    );

    CHECK(
        !avatar_m_drive_init(&drive, 0U),
        "Drive rejects CANopen node ID 0"
    );

    printf("\nTests failed: %d\n", failures);

    return failures != 0;
}
