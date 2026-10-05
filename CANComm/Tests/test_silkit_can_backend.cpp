#include "../SILKit/silkit_can_backend.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

using namespace std::chrono_literals;


int main()
{
    CanBackend receiver{};
    CanBackend sender{};

    SilKitCanBackendConfig receiver_config =
    {
        .participant_name = "AvatarMotor1",
        .controller_name = "MotorCAN",
        .network_name = "CAN1",
        .registry_uri = "silkit://localhost:8500",
        .bitrate = 1000000U
    };

    SilKitCanBackendConfig sender_config =
    {
        .participant_name = "RobotController",
        .controller_name = "RobotCAN",
        .network_name = "CAN1",
        .registry_uri = "silkit://localhost:8500",
        .bitrate = 1000000U
    };


    if (!silkit_can_backend_create(
            &receiver,
            &receiver_config))
    {
        std::printf(
            "[FAIL] Create receiver backend\n"
        );

        return 1;
    }

    std::printf(
        "[PASS] Receiver backend created\n"
    );


    if (!silkit_can_backend_create(
            &sender,
            &sender_config))
    {
        std::printf(
            "[FAIL] Create sender backend\n"
        );

        can_backend_close(&receiver);

        return 1;
    }

    std::printf(
        "[PASS] Sender backend created\n"
    );


    /*
     * AVATAR M Node 1 RPDO4 example:
     *
     * Target position = 50000
     *
     * COB-ID:
     * 0x500 + Node 1 = 0x501
     *
     * Payload:
     * 50000 = 0x0000C350
     *
     * Little endian:
     * 50 C3 00 00
     */
    CanFrame tx =
    {
        .id = 0x501,
        .dlc = 4,
        .data =
        {
            0x50,
            0xC3,
            0x00,
            0x00
        }
    };

    CanFrame rx{};

    const auto deadline =
        std::chrono::steady_clock::now()
        + 3s;


    /*
     * SIL Kit lifecycle initialization is asynchronous.
     *
     * Keep trying until both participants are ready
     * and the frame reaches the receiver.
     */
    while (
        std::chrono::steady_clock::now()
        < deadline
    )
    {
        CanBackendResult send_result =
            can_backend_send(
                &sender,
                &tx
            );

        if (
            send_result ==
            CAN_BACKEND_ERROR
        )
        {
            std::printf(
                "[FAIL] SIL Kit CAN send\n"
            );

            can_backend_close(&sender);
            can_backend_close(&receiver);

            return 1;
        }


        CanBackendResult receive_result =
            can_backend_receive(
                &receiver,
                &rx
            );

        if (
            receive_result ==
            CAN_BACKEND_ERROR
        )
        {
            std::printf(
                "[FAIL] SIL Kit CAN receive\n"
            );

            can_backend_close(&sender);
            can_backend_close(&receiver);

            return 1;
        }


        if (
            receive_result ==
            CAN_BACKEND_OK
        )
        {
            break;
        }


        std::this_thread::sleep_for(
            10ms
        );
    }


    if (
        rx.id != tx.id ||
        rx.dlc != tx.dlc ||
        std::memcmp(
            rx.data,
            tx.data,
            tx.dlc
        ) != 0
    )
    {
        std::printf(
            "[FAIL] Frame transmission\n"
        );

        std::printf(
            "Received ID=0x%03X DLC=%u\n",
            rx.id,
            rx.dlc
        );

        can_backend_close(&sender);
        can_backend_close(&receiver);

        return 1;
    }


    std::printf(
        "[PASS] SIL Kit CAN transmission\n"
    );

    std::printf(
        "[PASS] ID preserved: 0x%03X\n",
        rx.id
    );

    std::printf(
        "[PASS] DLC preserved: %u\n",
        rx.dlc
    );

    std::printf(
        "[PASS] Payload preserved: "
        "%02X %02X %02X %02X\n",
        rx.data[0],
        rx.data[1],
        rx.data[2],
        rx.data[3]
    );


    can_backend_close(&sender);
    can_backend_close(&receiver);


    std::printf(
        "\nTests failed: 0\n"
    );

    return 0;
}