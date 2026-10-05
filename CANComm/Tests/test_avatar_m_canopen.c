#include "../../ServoDrive/AvatarM/avatar_m_pdo.h"

#include <stdio.h>
#include <string.h>


static int failures = 0;


#define CHECK(condition, message)          \
    do                                     \
    {                                      \
        if (!(condition))                  \
        {                                  \
            printf("[FAIL] %s\n", message);\
            failures++;                    \
        }                                  \
        else                               \
        {                                  \
            printf("[PASS] %s\n", message);\
        }                                  \
    } while (0)


int main(void)
{
    CanFrame frame;

    CHECK(
        avatar_m_build_rpdo4(1U, 50000, &frame),
        "Build node-1 RPDO4"
    );

    CHECK(
        frame.id == 0x501U,
        "Node-1 RPDO4 COB-ID is 0x501"
    );

    CHECK(
        frame.dlc == 4U,
        "RPDO4 DLC is four bytes"
    );

    CHECK(
        frame.data[0] == 0x50 &&
        frame.data[1] == 0xC3 &&
        frame.data[2] == 0x00 &&
        frame.data[3] == 0x00,
        "50000 encodes as 50 C3 00 00"
    );


    CanFrame feedback_frame =
    {
        .id = 0x481U,
        .dlc = 6U,
        .data = {
            0x78, 0x0D, 0x00, 0x00,
            0x37, 0x04
        }
    };

    AvatarMFeedback feedback;

    CHECK(
        avatar_m_parse_tpdo4(
            1U,
            &feedback_frame,
            &feedback
        ),
        "Parse node-1 TPDO4"
    );

    CHECK(
        feedback.actual_position == 3448,
        "TPDO4 actual position decodes correctly"
    );

    CHECK(
        feedback.statusword == 0x0437U,
        "TPDO4 statusword decodes correctly"
    );


    avatar_m_build_sync(&frame);

    CHECK(
        frame.id == 0x080U,
        "SYNC COB-ID is 0x080"
    );

    CHECK(
        frame.dlc == 0U,
        "SYNC carries no payload"
    );


    printf(
        "\nTests failed: %d\n",
        failures
    );

    return failures == 0 ? 0 : 1;
}