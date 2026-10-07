#include "avatar_m_silkit_node.h"

#include "../../ServoDrive/AvatarM/avatar_m_position.h"
#include "../../ServoDrive/CiA402/cia402.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace
{

constexpr std::size_t kNodeCount = 6U;
constexpr uint32_t kBitrate = 1000000U;
constexpr double kMotionRateUnitsPerSecond = 1000000.0;
constexpr const char *kNetworkName = "CAN1";
constexpr const char *kDefaultRegistryUri = "silkit://localhost:8500";

std::atomic<bool> g_stop{false};

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

constexpr std::array<double, kNodeCount> kInitialJointDegrees =
{
     5.0,
   -85.0,
    95.0,
     5.0,
     5.0,
     5.0
};

void on_signal(int)
{
    g_stop.store(true);
}

void close_nodes(
    std::array<AvatarMSilKitNode, kNodeCount> &nodes,
    std::size_t created_count
)
{
    for (std::size_t i = 0U; i < created_count; ++i)
    {
        avatar_m_silkit_node_close(
            &nodes[i]
        );
    }
}

bool pump_node(
    AvatarMSilKitNode *node
)
{
    if (node == nullptr)
    {
        return false;
    }

    /*
     * Drain the participant receive queue. The limit protects the runtime
     * from an accidental endless receive loop while still allowing bursts
     * such as six RPDO4 frames plus SYNC.
     */
    for (unsigned i = 0U; i < 128U; ++i)
    {
        const AvatarMSilKitStepResult result =
            avatar_m_silkit_node_pump_once(
                node
            );

        if (result == AVATAR_M_SILKIT_ERROR)
        {
            return false;
        }

        if (result == AVATAR_M_SILKIT_IDLE)
        {
            return true;
        }
    }

    return true;
}

} /* namespace */


int main()
{
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    const char *registry_uri =
        std::getenv("SILKIT_REGISTRY_URI");

    if (
        registry_uri == nullptr ||
        registry_uri[0] == '\0'
    )
    {
        registry_uri =
            kDefaultRegistryUri;
    }

    std::array<AvatarMSilKitNode, kNodeCount> nodes{};
    std::array<AvatarMPositionScale, kNodeCount> scales{};
    std::array<double, kNodeCount> simulated_positions{};

    std::size_t created_count = 0U;

    for (std::size_t i = 0U; i < kNodeCount; ++i)
    {
        AvatarMSilKitNodeConfig config{};

        config.node_id =
            static_cast<uint8_t>(i + 1U);

        config.participant_name =
            kParticipantNames[i];

        config.controller_name =
            kControllerNames[i];

        config.network_name =
            kNetworkName;

        config.registry_uri =
            registry_uri;

        config.bitrate =
            kBitrate;

        if (!avatar_m_silkit_node_create(
                &nodes[i],
                &config))
        {
            std::fprintf(
                stderr,
                "Could not create AVATAR SIL Kit node %zu.\n",
                i + 1U
            );

            close_nodes(
                nodes,
                created_count
            );

            return 1;
        }

        created_count++;

        if (!silkit_can_backend_wait_ready(
                &nodes[i].backend,
                5000U))
        {
            std::fprintf(
                stderr,
                "AVATAR SIL Kit node %zu did not become ready.\n",
                i + 1U
            );

            close_nodes(
                nodes,
                created_count
            );

            return 1;
        }

        if (!avatar_m_position_scale_default(
                &scales[i],
                50.0))
        {
            std::fprintf(
                stderr,
                "Could not create AVATAR position scale.\n"
            );

            close_nodes(
                nodes,
                created_count
            );

            return 1;
        }

        avatar_m_node_set_work_mode(
            &nodes[i].motor,
            1U
        );

        avatar_m_node_set_statusword(
            &nodes[i].motor,
            CIA402_STATE_SWITCH_ON_DISABLED
        );

        const double joint_rad =
            kInitialJointDegrees[i] *
            3.14159265358979323846 /
            180.0;

        int32_t initial_position = 0;

        if (!avatar_m_joint_rad_to_position_units(
                &scales[i],
                joint_rad,
                &initial_position))
        {
            std::fprintf(
                stderr,
                "Could not convert initial joint %zu position.\n",
                i + 1U
            );

            close_nodes(
                nodes,
                created_count
            );

            return 1;
        }

        avatar_m_node_set_actual_position(
            &nodes[i].motor,
            initial_position
        );

        simulated_positions[i] =
            static_cast<double>(
                initial_position
            );
    }

    std::printf(
        "AVATAR SIL Kit motor bank ready.\n"
        "Registry : %s\n"
        "Network  : %s\n"
        "Bitrate  : %u bit/s\n"
        "Nodes    : 1..6\n",
        registry_uri,
        kNetworkName,
        kBitrate
    );
    std::fflush(stdout);

    auto previous =
        std::chrono::steady_clock::now();

    while (!g_stop.load())
    {
        const auto now =
            std::chrono::steady_clock::now();

        auto elapsed_ms =
            std::chrono::duration_cast<
                std::chrono::milliseconds
            >(now - previous).count();

        if (elapsed_ms <= 0)
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1)
            );

            continue;
        }

        if (elapsed_ms > 100)
        {
            elapsed_ms = 100;
        }

        previous = now;

        const double max_step =
            kMotionRateUnitsPerSecond *
            static_cast<double>(elapsed_ms) /
            1000.0;

        /*
         * This is only a visualization/integration motion follower.
         * CANopen transport, node addressing, PDO/SDO/NMT/SYNC and heartbeat
         * traffic still travel through SIL Kit. No claim is made that this
         * follower models AVATAR motor dynamics.
         */
        for (std::size_t i = 0U; i < kNodeCount; ++i)
        {
            AvatarMNodeSim &motor =
                nodes[i].motor;

            if (motor.active_target_valid)
            {
                const double target =
                    static_cast<double>(
                        motor.active_target_position
                    );

                const double error =
                    target -
                    simulated_positions[i];

                if (std::fabs(error) <= max_step)
                {
                    simulated_positions[i] =
                        target;
                }
                else
                {
                    simulated_positions[i] +=
                        error > 0.0
                            ? max_step
                            : -max_step;
                }

                avatar_m_node_set_actual_position(
                    &motor,
                    static_cast<int32_t>(
                        std::llround(
                            simulated_positions[i]
                        )
                    )
                );
            }

            const AvatarMSilKitStepResult heartbeat_result =
                avatar_m_silkit_node_tick_ms(
                    &nodes[i],
                    static_cast<uint32_t>(
                        elapsed_ms
                    )
                );

            if (heartbeat_result == AVATAR_M_SILKIT_ERROR)
            {
                std::fprintf(
                    stderr,
                    "Heartbeat transport failed for node %zu.\n",
                    i + 1U
                );

                g_stop.store(true);
                break;
            }
        }

        for (std::size_t i = 0U; i < kNodeCount; ++i)
        {
            if (!pump_node(
                    &nodes[i]))
            {
                std::fprintf(
                    stderr,
                    "CAN processing failed for node %zu.\n",
                    i + 1U
                );

                g_stop.store(true);
                break;
            }
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(1)
        );
    }

    close_nodes(
        nodes,
        created_count
    );

    std::puts(
        "AVATAR SIL Kit motor bank stopped."
    );

    return 0;
}
