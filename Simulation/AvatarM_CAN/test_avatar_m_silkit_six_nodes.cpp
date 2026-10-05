#include "avatar_m_silkit_node.h"

#include "../../ServoDrive/AvatarM/avatar_m_pdo.h"

#include <array>
#include <chrono>
#include <cstdio>
#include <thread>

using namespace std::chrono_literals;

namespace
{

constexpr std::size_t kNodeCount = 6U;
constexpr uint32_t kBitrate = 1000000U;
constexpr const char *kNetworkName = "CAN1";
constexpr const char *kRegistryUri = "silkit://localhost:8500";

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

bool send_with_timeout(
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

        std::this_thread::sleep_for(5ms);
    }

    return false;
}

bool pump_until_nmt_state(
    AvatarMSilKitNode *motor,
    AvatarMNmtState expected_state,
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

        if (motor->motor.nmt_state == expected_state)
        {
            return true;
        }

        std::this_thread::sleep_for(2ms);
    }

    return false;
}

bool pump_until_tpdo_sent(
    AvatarMSilKitNode *motor,
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

        if (result == AVATAR_M_SILKIT_FRAME_SENT)
        {
            return true;
        }

        std::this_thread::sleep_for(2ms);
    }

    return false;
}

bool pump_until_target_active(
    AvatarMSilKitNode *motor,
    int32_t expected_target,
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

        if (
            motor->motor.active_target_valid &&
            motor->motor.active_target_position == expected_target
        )
        {
            return true;
        }

        std::this_thread::sleep_for(2ms);
    }

    return false;
}

bool receive_all_tpdo4(
    CanBackend *controller,
    std::array<CanFrame, kNodeCount> *frames,
    std::chrono::milliseconds timeout
)
{
    std::array<bool, kNodeCount> received{};
    std::size_t received_count = 0U;

    const auto deadline =
        std::chrono::steady_clock::now() + timeout;

    while (
        std::chrono::steady_clock::now() < deadline &&
        received_count < kNodeCount
    )
    {
        CanFrame frame{};

        const CanBackendResult result =
            can_backend_receive(controller, &frame);

        if (result == CAN_BACKEND_ERROR)
        {
            return false;
        }

        if (result == CAN_BACKEND_WOULD_BLOCK)
        {
            std::this_thread::sleep_for(2ms);
            continue;
        }

        for (std::size_t i = 0U; i < kNodeCount; ++i)
        {
            const uint8_t node_id =
                static_cast<uint8_t>(i + 1U);

            const uint16_t expected_id =
                static_cast<uint16_t>(0x480U + node_id);

            if (
                frame.id == expected_id &&
                !received[i]
            )
            {
                (*frames)[i] = frame;
                received[i] = true;
                received_count++;
                break;
            }
        }
    }

    return received_count == kNodeCount;
}

bool receive_all_heartbeats(
    CanBackend *controller,
    std::array<CanFrame, kNodeCount> *frames,
    std::chrono::milliseconds timeout
)
{
    std::array<bool, kNodeCount> received{};
    std::size_t received_count = 0U;

    const auto deadline =
        std::chrono::steady_clock::now() + timeout;

    while (
        std::chrono::steady_clock::now() < deadline &&
        received_count < kNodeCount
    )
    {
        CanFrame frame{};

        const CanBackendResult result =
            can_backend_receive(controller, &frame);

        if (result == CAN_BACKEND_ERROR)
        {
            return false;
        }

        if (result == CAN_BACKEND_WOULD_BLOCK)
        {
            std::this_thread::sleep_for(2ms);
            continue;
        }

        for (std::size_t i = 0U; i < kNodeCount; ++i)
        {
            const uint8_t node_id =
                static_cast<uint8_t>(i + 1U);

            const uint16_t expected_id =
                static_cast<uint16_t>(0x700U + node_id);

            if (
                frame.id == expected_id &&
                !received[i]
            )
            {
                (*frames)[i] = frame;
                received[i] = true;
                received_count++;
                break;
            }
        }
    }

    return received_count == kNodeCount;
}

} /* namespace */

int main()
{
    /*
     * Test fixtures only. These are not claimed as AVATAR power-on defaults.
     * Distinct values make cross-node mix-ups visible.
     */
    constexpr std::array<int32_t, kNodeCount> kTargets =
    {
        10000,
        20000,
        30000,
        40000,
        50000,
        60000
    };

    constexpr std::array<int32_t, kNodeCount> kActualPositions =
    {
        1000,
        2000,
        3000,
        4000,
        5000,
        6000
    };

    constexpr std::array<const char *, kNodeCount> kParticipantNames =
    {
        "AvatarMotor1",
        "AvatarMotor2",
        "AvatarMotor3",
        "AvatarMotor4",
        "AvatarMotor5",
        "AvatarMotor6"
    };

    constexpr std::array<const char *, kNodeCount> kControllerNames =
    {
        "MotorCAN1",
        "MotorCAN2",
        "MotorCAN3",
        "MotorCAN4",
        "MotorCAN5",
        "MotorCAN6"
    };

    std::array<AvatarMSilKitNode, kNodeCount> motors{};

    for (std::size_t i = 0U; i < kNodeCount; ++i)
    {
        AvatarMSilKitNodeConfig config{};
        config.node_id = static_cast<uint8_t>(i + 1U);
        config.participant_name = kParticipantNames[i];
        config.controller_name = kControllerNames[i];
        config.network_name = kNetworkName;
        config.registry_uri = kRegistryUri;
        config.bitrate = kBitrate;

        const bool created =
            avatar_m_silkit_node_create(
                &motors[i],
                &config
            );

        char message[80];
        std::snprintf(
            message,
            sizeof(message),
            "AVATAR M node %zu created",
            i + 1U
        );

        CHECK(created, message);

        if (!created)
        {
            for (std::size_t j = 0U; j < i; ++j)
            {
                avatar_m_silkit_node_close(&motors[j]);
            }

            return 1;
        }

        /* Vendor-documented CANopen interpolation work mode. */
        avatar_m_node_set_work_mode(
            &motors[i].motor,
            7U
        );

        avatar_m_node_set_actual_position(
            &motors[i].motor,
            kActualPositions[i]
        );

        /* Explicit test fixture, not a claimed default statusword. */
        avatar_m_node_set_statusword(
            &motors[i].motor,
            0x0437U
        );
    }

    CanBackend controller{};

    SilKitCanBackendConfig controller_config{};
    controller_config.participant_name = "RobotControllerSixNodeTest";
    controller_config.controller_name = "RobotCAN";
    controller_config.network_name = kNetworkName;
    controller_config.registry_uri = kRegistryUri;
    controller_config.bitrate = kBitrate;

    CHECK(
        silkit_can_backend_create(
            &controller,
            &controller_config
        ),
        "Robot controller SIL Kit backend created"
    );

    if (failures != 0)
    {
        for (auto &motor : motors)
        {
            avatar_m_silkit_node_close(&motor);
        }

        return 1;
    }

    /*
     * NMT Start with node ID 0 broadcasts the command to all CANopen nodes.
     */
    CanFrame nmt_start{};
    nmt_start.id = 0x000U;
    nmt_start.dlc = 2U;
    nmt_start.data[0] = 0x01U;
    nmt_start.data[1] = 0x00U;

    CHECK(
        send_with_timeout(
            &controller,
            &nmt_start,
            3s
        ),
        "Broadcast NMT Start sent"
    );

    bool all_operational = true;

    for (auto &motor : motors)
    {
        if (!pump_until_nmt_state(
                &motor,
                AVATAR_M_NMT_OPERATIONAL,
                3s))
        {
            all_operational = false;
        }
    }

    CHECK(
        all_operational,
        "All 6 virtual motors entered Operational state"
    );

    /*
     * Send one RPDO4 to each node. Per the vendor mapping these use
     * 0x501..0x506. Each motor must cache only its own target and immediately
     * return its own TPDO4 on 0x481..0x486.
     */
    bool all_rpdo_sent = true;

    for (std::size_t i = 0U; i < kNodeCount; ++i)
    {
        CanFrame rpdo4{};

        if (!avatar_m_build_rpdo4(
                static_cast<uint8_t>(i + 1U),
                kTargets[i],
                &rpdo4))
        {
            all_rpdo_sent = false;
            continue;
        }

        const uint16_t expected_id =
            static_cast<uint16_t>(0x501U + i);

        if (rpdo4.id != expected_id)
        {
            all_rpdo_sent = false;
        }

        if (!send_with_timeout(
                &controller,
                &rpdo4,
                3s))
        {
            all_rpdo_sent = false;
        }
    }

    CHECK(
        all_rpdo_sent,
        "RPDO4 frames 0x501..0x506 sent to nodes 1..6"
    );

    bool all_tpdo_sent = true;

    for (auto &motor : motors)
    {
        if (!pump_until_tpdo_sent(
                &motor,
                3s))
        {
            all_tpdo_sent = false;
        }
    }

    CHECK(
        all_tpdo_sent,
        "All 6 motors transmitted immediate TPDO4 feedback"
    );

    bool all_cached = true;
    bool none_active = true;

    for (std::size_t i = 0U; i < kNodeCount; ++i)
    {
        if (
            !motors[i].motor.cached_target_valid ||
            motors[i].motor.cached_target_position != kTargets[i]
        )
        {
            all_cached = false;
        }

        if (motors[i].motor.active_target_valid)
        {
            none_active = false;
        }
    }

    CHECK(
        all_cached,
        "Each motor cached only its node-specific target before SYNC"
    );

    CHECK(
        none_active,
        "No motor activated its target before SYNC"
    );

    std::array<CanFrame, kNodeCount> tpdo_frames{};

    CHECK(
        receive_all_tpdo4(
            &controller,
            &tpdo_frames,
            5s
        ),
        "Controller received TPDO4 IDs 0x481..0x486"
    );

    bool all_feedback_correct = true;

    for (std::size_t i = 0U; i < kNodeCount; ++i)
    {
        AvatarMFeedback feedback{};

        if (!avatar_m_parse_tpdo4(
                static_cast<uint8_t>(i + 1U),
                &tpdo_frames[i],
                &feedback))
        {
            all_feedback_correct = false;
            continue;
        }

        if (
            feedback.actual_position != kActualPositions[i] ||
            feedback.statusword != 0x0437U
        )
        {
            all_feedback_correct = false;
        }
    }

    CHECK(
        all_feedback_correct,
        "All 6 TPDO4 frames preserve node-specific feedback"
    );

    /*
     * One global SYNC frame releases the six already-cached RPDO4 targets.
     */
    CanFrame sync{};
    avatar_m_build_sync(&sync);

    CHECK(
        sync.id == 0x080U && sync.dlc == 0U,
        "SYNC frame is 0x080 with DLC 0"
    );

    CHECK(
        send_with_timeout(
            &controller,
            &sync,
            3s
        ),
        "Single SYNC sent for all 6 motors"
    );

    bool all_targets_active = true;

    for (std::size_t i = 0U; i < kNodeCount; ++i)
    {
        if (!pump_until_target_active(
                &motors[i],
                kTargets[i],
                5s))
        {
            all_targets_active = false;
        }
    }

    CHECK(
        all_targets_active,
        "One SYNC released all 6 cached targets"
    );

    bool physical_positions_unchanged = true;

    for (std::size_t i = 0U; i < kNodeCount; ++i)
    {
        if (motors[i].motor.actual_position != kActualPositions[i])
        {
            physical_positions_unchanged = false;
        }
    }

    CHECK(
        physical_positions_unchanged,
        "SYNC still does not invent physical motor motion"
    );

    /* Verify each node's documented 1000 ms operational heartbeat path. */
    bool all_heartbeat_sent = true;

    for (auto &motor : motors)
    {
        if (
            avatar_m_silkit_node_tick_ms(
                &motor,
                1000U
            ) != AVATAR_M_SILKIT_FRAME_SENT
        )
        {
            all_heartbeat_sent = false;
        }
    }

    CHECK(
        all_heartbeat_sent,
        "All 6 motors transmitted their 1000 ms heartbeat"
    );

    std::array<CanFrame, kNodeCount> heartbeat_frames{};

    CHECK(
        receive_all_heartbeats(
            &controller,
            &heartbeat_frames,
            5s
        ),
        "Controller received heartbeat IDs 0x701..0x706"
    );

    bool all_heartbeats_correct = true;

    for (std::size_t i = 0U; i < kNodeCount; ++i)
    {
        if (
            heartbeat_frames[i].id !=
                static_cast<uint16_t>(0x701U + i) ||
            heartbeat_frames[i].dlc != 1U ||
            heartbeat_frames[i].data[0] != 0x05U
        )
        {
            all_heartbeats_correct = false;
        }
    }

    CHECK(
        all_heartbeats_correct,
        "All 6 operational heartbeat payloads are 0x05"
    );

    can_backend_close(&controller);

    for (auto &motor : motors)
    {
        avatar_m_silkit_node_close(&motor);
    }

    std::printf(
        "\nTests failed: %d\n",
        failures
    );

    return failures != 0;
}
