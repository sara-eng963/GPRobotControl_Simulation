#include "avatar_m_silkit_node.h"

#include "../../ServoDrive/AvatarM/avatar_m_pdo.h"

#include <chrono>
#include <cstdio>
#include <thread>

using namespace std::chrono_literals;

static int failures = 0;

#define CHECK(condition, message) \
    do \
    { \
        if (condition) \
        { \
            std::printf("[PASS] %s\n", message); \
        } \
        else \
        { \
            std::printf("[FAIL] %s\n", message); \
            failures++; \
        } \
    } while (0)

static bool send_with_timeout(
    CanBackend *backend,
    const CanFrame *frame,
    std::chrono::milliseconds timeout
)
{
    const auto deadline =
        std::chrono::steady_clock::now() + timeout;

    while (std::chrono::steady_clock::now() < deadline)
    {
        const CanBackendResult result =
            can_backend_send(backend, frame);

        if (result == CAN_BACKEND_OK)
        {
            return true;
        }

        if (result == CAN_BACKEND_ERROR)
        {
            return false;
        }

        std::this_thread::sleep_for(10ms);
    }

    return false;
}

static bool pump_motor_until_activity(
    AvatarMSilKitNode *motor,
    AvatarMSilKitStepResult *result_out,
    std::chrono::milliseconds timeout
)
{
    const auto deadline =
        std::chrono::steady_clock::now() + timeout;

    while (std::chrono::steady_clock::now() < deadline)
    {
        const AvatarMSilKitStepResult result =
            avatar_m_silkit_node_pump_once(motor);

        if (result == AVATAR_M_SILKIT_ERROR)
        {
            return false;
        }

        if (result != AVATAR_M_SILKIT_IDLE)
        {
            if (result_out != nullptr)
            {
                *result_out = result;
            }

            return true;
        }

        std::this_thread::sleep_for(10ms);
    }

    return false;
}

static bool receive_id_with_timeout(
    CanBackend *backend,
    uint16_t expected_id,
    CanFrame *frame_out,
    std::chrono::milliseconds timeout
)
{
    const auto deadline =
        std::chrono::steady_clock::now() + timeout;

    while (std::chrono::steady_clock::now() < deadline)
    {
        CanFrame frame{};

        const CanBackendResult result =
            can_backend_receive(backend, &frame);

        if (result == CAN_BACKEND_ERROR)
        {
            return false;
        }

        if (result == CAN_BACKEND_OK)
        {
            if (frame.id == expected_id)
            {
                if (frame_out != nullptr)
                {
                    *frame_out = frame;
                }

                return true;
            }

            continue;
        }

        std::this_thread::sleep_for(10ms);
    }

    return false;
}

int main()
{
    AvatarMSilKitNode motor{};

    AvatarMSilKitNodeConfig motor_config{};
    motor_config.node_id = 1U;
    motor_config.participant_name = "AvatarMotor1";
    motor_config.controller_name = "MotorCAN1";
    motor_config.network_name = "CAN1";
    motor_config.registry_uri = "silkit://localhost:8500";
    motor_config.bitrate = 1000000U;

    CHECK(
        avatar_m_silkit_node_create(
            &motor,
            &motor_config
        ),
        "AVATAR M SIL Kit node created"
    );

    if (failures != 0)
    {
        return 1;
    }

    /*
     * Explicit test preconditions.
     *
     * These values are not claimed as motor power-on defaults. Mode 7 is the
     * vendor-documented CANopen interpolation mode, while 3448 / 0x0437 are
     * taken from the manual's RPDO4/TPDO4 interpolation example.
     */
    avatar_m_node_set_work_mode(
        &motor.motor,
        7U
    );

    avatar_m_node_set_actual_position(
        &motor.motor,
        3448
    );

    avatar_m_node_set_statusword(
        &motor.motor,
        0x0437U
    );

    CanBackend controller{};

    SilKitCanBackendConfig controller_config{};
    controller_config.participant_name = "RobotControllerIntegrationTest";
    controller_config.controller_name = "RobotCAN";
    controller_config.network_name = "CAN1";
    controller_config.registry_uri = "silkit://localhost:8500";
    controller_config.bitrate = 1000000U;

    CHECK(
        silkit_can_backend_create(
            &controller,
            &controller_config
        ),
        "Robot controller SIL Kit backend created"
    );

    if (failures != 0)
    {
        avatar_m_silkit_node_close(&motor);
        return 1;
    }

    /* Put node 1 into CANopen Operational state. */
    CanFrame nmt_start{};
    nmt_start.id = 0x000U;
    nmt_start.dlc = 2U;
    nmt_start.data[0] = 0x01U;
    nmt_start.data[1] = 0x01U;

    CHECK(
        send_with_timeout(
            &controller,
            &nmt_start,
            3s
        ),
        "NMT Start sent through SIL Kit"
    );

    AvatarMSilKitStepResult motor_step =
        AVATAR_M_SILKIT_IDLE;

    CHECK(
        pump_motor_until_activity(
            &motor,
            &motor_step,
            3s
        ),
        "Virtual motor received NMT Start"
    );

    CHECK(
        motor.motor.nmt_state == AVATAR_M_NMT_OPERATIONAL,
        "Virtual motor entered Operational state"
    );

    /* Send the exact node-1 RPDO4 example: target position 50000. */
    CanFrame rpdo4{};

    CHECK(
        avatar_m_build_rpdo4(
            1U,
            50000,
            &rpdo4
        ),
        "RPDO4 target 50000 built"
    );

    CHECK(
        send_with_timeout(
            &controller,
            &rpdo4,
            3s
        ),
        "RPDO4 sent through SIL Kit"
    );

    CHECK(
        pump_motor_until_activity(
            &motor,
            &motor_step,
            3s
        ),
        "Virtual motor processed RPDO4"
    );

    CHECK(
        motor_step == AVATAR_M_SILKIT_FRAME_SENT,
        "Virtual motor transmitted immediate TPDO4"
    );

    CHECK(
        motor.motor.cached_target_valid &&
        motor.motor.cached_target_position == 50000,
        "RPDO4 target cached before SYNC"
    );

    CHECK(
        !motor.motor.active_target_valid,
        "Target is not active before SYNC"
    );

    CanFrame tpdo4{};

    CHECK(
        receive_id_with_timeout(
            &controller,
            0x481U,
            &tpdo4,
            3s
        ),
        "Robot controller received TPDO4 through SIL Kit"
    );

    AvatarMFeedback feedback{};

    CHECK(
        avatar_m_parse_tpdo4(
            1U,
            &tpdo4,
            &feedback
        ),
        "TPDO4 decoded"
    );

    CHECK(
        feedback.actual_position == 3448,
        "TPDO4 actual position is 3448"
    );

    CHECK(
        feedback.statusword == 0x0437U,
        "TPDO4 statusword is 0x0437"
    );

    /* Release the cached interpolation target with SYNC. */
    CanFrame sync{};
    avatar_m_build_sync(&sync);

    CHECK(
        send_with_timeout(
            &controller,
            &sync,
            3s
        ),
        "SYNC sent through SIL Kit"
    );

    CHECK(
        pump_motor_until_activity(
            &motor,
            &motor_step,
            3s
        ),
        "Virtual motor processed SYNC"
    );

    CHECK(
        motor.motor.active_target_valid &&
        motor.motor.active_target_position == 50000,
        "SYNC released target 50000"
    );

    CHECK(
        motor.motor.actual_position == 3448,
        "SYNC does not invent physical motor motion"
    );

    /* Verify heartbeat also crosses the SIL Kit bus. */
    CHECK(
        avatar_m_silkit_node_tick_ms(
            &motor,
            1000U
        ) == AVATAR_M_SILKIT_FRAME_SENT,
        "Virtual motor transmitted 1000 ms heartbeat"
    );

    CanFrame heartbeat{};

    CHECK(
        receive_id_with_timeout(
            &controller,
            0x701U,
            &heartbeat,
            3s
        ),
        "Robot controller received heartbeat through SIL Kit"
    );

    CHECK(
        heartbeat.dlc == 1U &&
        heartbeat.data[0] == 0x05U,
        "Operational heartbeat payload is 0x05"
    );

    can_backend_close(&controller);
    avatar_m_silkit_node_close(&motor);

    std::printf(
        "\nTests failed: %d\n",
        failures
    );

    return failures != 0;
}
