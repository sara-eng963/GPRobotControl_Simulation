#include "../../CANComm/SILKit/silkit_can_backend.h"
#include "../../CANComm/CANopen/canopen_ids.h"

#include "raylib.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace
{

constexpr uint32_t kBitrate = 1000000U;
constexpr uint32_t kExpectedSyncUs = 2000U;
constexpr uint32_t kBurstResetUs = 20000U;
constexpr std::size_t kNodeCount = 6U;
constexpr double kStatsWindowSeconds = 1.0;
constexpr const char *kDefaultRegistryUri = "silkit://localhost:8500";
constexpr const char *kNetworkName = "CAN1";

using Clock = std::chrono::steady_clock;

struct FrameInfo
{
    uint64_t time_us = 0U;
    uint64_t delta_us = 0U;
    uint16_t id = 0U;
    uint8_t dlc = 0U;
    uint8_t node = 0U;
    std::string type;
    std::string detail;
};

struct WireSample
{
    uint64_t time_us = 0U;
    uint32_t nominal_bits = 0U;
    uint32_t worst_case_bits = 0U;
};

struct NodeStats
{
    uint64_t rpdo4_count = 0U;
    uint64_t tpdo4_count = 0U;

    uint64_t last_rpdo_us = 0U;
    uint64_t last_tpdo_us = 0U;

    double latency_latest_ms = 0.0;
    double latency_max_ms = 0.0;
    double latency_sum_ms = 0.0;
    uint64_t latency_count = 0U;

    int32_t target_position = 0;
    int32_t actual_position = 0;
    uint16_t statusword = 0U;
};

struct MonitorStats
{
    uint64_t total_frames = 0U;
    uint64_t last_frame_us = 0U;

    uint64_t nmt_count = 0U;
    uint64_t sync_count = 0U;
    uint64_t sdo_count = 0U;
    uint64_t heartbeat_count = 0U;
    uint64_t other_count = 0U;

    uint64_t last_sync_us = 0U;
    double sync_period_latest_ms = 0.0;
    double sync_period_sum_ms = 0.0;
    uint64_t sync_period_count = 0U;
    double sync_jitter_latest_ms = 0.0;
    double sync_jitter_max_ms = 0.0;
    uint64_t missed_sync_cycles = 0U;

    std::array<NodeStats, kNodeCount> nodes{};
};

uint32_t read_u32_le(const uint8_t *data)
{
    return
        static_cast<uint32_t>(data[0]) |
        (static_cast<uint32_t>(data[1]) << 8U) |
        (static_cast<uint32_t>(data[2]) << 16U) |
        (static_cast<uint32_t>(data[3]) << 24U);
}

uint16_t read_u16_le(const uint8_t *data)
{
    return static_cast<uint16_t>(
        static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8U)
    );
}

int32_t read_i32_le(const uint8_t *data)
{
    const uint32_t raw = read_u32_le(data);
    int32_t value = 0;
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}

uint32_t nominal_can_bits(uint8_t dlc)
{
    /*
     * Standard 11-bit Classical CAN data frame, including the 3-bit
     * intermission, before bit stuffing:
     *
     *     47 + 8*DLC bits
     */
    return 47U + 8U * static_cast<uint32_t>(dlc);
}

uint32_t conservative_stuffed_bits(uint8_t dlc)
{
    /*
     * Conservative upper estimate for bit stuffing in the region from SOF
     * through the CRC sequence. This is an estimate for dashboard/load
     * planning only; the current SIL Kit setup does not model physical
     * arbitration/bit timing.
     */
    const uint32_t stuffable =
        34U + 8U * static_cast<uint32_t>(dlc);

    const uint32_t max_stuff =
        stuffable > 0U
            ? (stuffable - 1U) / 4U
            : 0U;

    return nominal_can_bits(dlc) + max_stuff;
}

std::string hex_id(uint16_t id)
{
    std::ostringstream out;
    out << "0x"
        << std::uppercase
        << std::hex
        << std::setw(3)
        << std::setfill('0')
        << static_cast<unsigned>(id);
    return out.str();
}

std::string data_hex(const CanFrame &frame)
{
    std::ostringstream out;
    out << std::uppercase << std::hex << std::setfill('0');

    for (uint8_t i = 0U; i < frame.dlc; ++i)
    {
        if (i != 0U)
        {
            out << ' ';
        }

        out << std::setw(2)
            << static_cast<unsigned>(frame.data[i]);
    }

    return out.str();
}

FrameInfo classify_frame(
    const CanFrame &frame,
    uint64_t now_us,
    uint64_t delta_us,
    MonitorStats &stats
)
{
    FrameInfo info;
    info.time_us = now_us;
    info.delta_us = delta_us;
    info.id = frame.id;
    info.dlc = frame.dlc;

    if (frame.id == CANOPEN_COBID_NMT)
    {
        info.type = "NMT";
        stats.nmt_count++;

        if (frame.dlc >= 2U)
        {
            std::ostringstream detail;
            detail << "cmd="
                   << static_cast<unsigned>(frame.data[0])
                   << " node="
                   << static_cast<unsigned>(frame.data[1]);
            info.detail = detail.str();
        }

        return info;
    }

    if (frame.id == CANOPEN_COBID_SYNC)
    {
        info.type = "SYNC";
        info.detail = "cycle trigger";
        stats.sync_count++;

        if (stats.last_sync_us != 0U)
        {
            const uint64_t period_us =
                now_us - stats.last_sync_us;

            if (period_us < kBurstResetUs)
            {
                const double period_ms =
                    static_cast<double>(period_us) / 1000.0;

                const double jitter_ms =
                    std::fabs(
                        period_ms -
                        static_cast<double>(kExpectedSyncUs) / 1000.0
                    );

                stats.sync_period_latest_ms = period_ms;
                stats.sync_period_sum_ms += period_ms;
                stats.sync_period_count++;
                stats.sync_jitter_latest_ms = jitter_ms;
                stats.sync_jitter_max_ms =
                    std::max(
                        stats.sync_jitter_max_ms,
                        jitter_ms
                    );

                if (period_us > 3000U)
                {
                    const uint64_t intervals =
                        static_cast<uint64_t>(
                            std::llround(
                                static_cast<double>(period_us) /
                                static_cast<double>(kExpectedSyncUs)
                            )
                        );

                    if (intervals > 1U)
                    {
                        stats.missed_sync_cycles +=
                            intervals - 1U;
                    }
                }
            }
            else
            {
                /*
                 * Treat a long no-SYNC interval as the boundary between
                 * motion bursts/states, not as hundreds of missed 2 ms cycles.
                 */
                stats.sync_period_latest_ms = 0.0;
                stats.sync_jitter_latest_ms = 0.0;
            }
        }

        stats.last_sync_us = now_us;
        return info;
    }

    auto classify_node_range =
        [&](uint16_t base, const char *name) -> bool
        {
            if (
                frame.id > base &&
                frame.id <= static_cast<uint16_t>(base + 127U)
            )
            {
                info.node =
                    static_cast<uint8_t>(frame.id - base);
                info.type = name;
                return true;
            }

            return false;
        };

    if (classify_node_range(CANOPEN_RPDO4_BASE, "RPDO4"))
    {
        if (
            info.node >= 1U &&
            info.node <= kNodeCount &&
            frame.dlc >= 4U
        )
        {
            NodeStats &node =
                stats.nodes[info.node - 1U];

            node.rpdo4_count++;
            node.last_rpdo_us = now_us;
            node.target_position =
                read_i32_le(frame.data);

            std::ostringstream detail;
            detail << "target="
                   << node.target_position;
            info.detail = detail.str();
        }

        return info;
    }

    if (classify_node_range(CANOPEN_TPDO4_BASE, "TPDO4"))
    {
        if (
            info.node >= 1U &&
            info.node <= kNodeCount &&
            frame.dlc >= 6U
        )
        {
            NodeStats &node =
                stats.nodes[info.node - 1U];

            node.tpdo4_count++;
            node.last_tpdo_us = now_us;
            node.actual_position =
                read_i32_le(frame.data);
            node.statusword =
                read_u16_le(&frame.data[4]);

            if (
                node.last_rpdo_us != 0U &&
                now_us >= node.last_rpdo_us
            )
            {
                const double latency_ms =
                    static_cast<double>(
                        now_us - node.last_rpdo_us
                    ) / 1000.0;

                node.latency_latest_ms =
                    latency_ms;

                node.latency_max_ms =
                    std::max(
                        node.latency_max_ms,
                        latency_ms
                    );

                node.latency_sum_ms +=
                    latency_ms;

                node.latency_count++;
            }

            std::ostringstream detail;
            detail << "actual="
                   << node.actual_position
                   << " sw=0x"
                   << std::uppercase
                   << std::hex
                   << std::setw(4)
                   << std::setfill('0')
                   << node.statusword;
            info.detail = detail.str();
        }

        return info;
    }

    if (
        classify_node_range(
            CANOPEN_SDO_RX_BASE,
            "SDO_RX"
        ) ||
        classify_node_range(
            CANOPEN_SDO_TX_BASE,
            "SDO_TX"
        )
    )
    {
        stats.sdo_count++;

        if (frame.dlc >= 4U)
        {
            const uint16_t index =
                static_cast<uint16_t>(
                    static_cast<uint16_t>(frame.data[1]) |
                    (static_cast<uint16_t>(frame.data[2]) << 8U)
                );

            std::ostringstream detail;
            detail << "0x"
                   << std::uppercase
                   << std::hex
                   << std::setw(4)
                   << std::setfill('0')
                   << index
                   << ":"
                   << std::setw(2)
                   << static_cast<unsigned>(frame.data[3]);
            info.detail = detail.str();
        }

        return info;
    }

    if (
        classify_node_range(
            CANOPEN_HEARTBEAT_BASE,
            "HEARTBEAT"
        )
    )
    {
        stats.heartbeat_count++;

        if (frame.dlc >= 1U)
        {
            std::ostringstream detail;
            detail << "NMT_state=0x"
                   << std::uppercase
                   << std::hex
                   << std::setw(2)
                   << std::setfill('0')
                   << static_cast<unsigned>(frame.data[0]);
            info.detail = detail.str();
        }

        return info;
    }

    if (
        classify_node_range(CANOPEN_RPDO1_BASE, "RPDO1") ||
        classify_node_range(CANOPEN_RPDO2_BASE, "RPDO2") ||
        classify_node_range(CANOPEN_RPDO3_BASE, "RPDO3") ||
        classify_node_range(CANOPEN_TPDO1_BASE, "TPDO1") ||
        classify_node_range(CANOPEN_TPDO2_BASE, "TPDO2") ||
        classify_node_range(CANOPEN_TPDO3_BASE, "TPDO3")
    )
    {
        return info;
    }

    info.type = "OTHER";
    stats.other_count++;
    return info;
}

Color panel_color()
{
    return Color{28, 33, 40, 255};
}

Color card_color()
{
    return Color{37, 43, 52, 255};
}

Color text_color()
{
    return Color{226, 232, 240, 255};
}

Color muted_color()
{
    return Color{148, 163, 184, 255};
}

Color good_color()
{
    return Color{74, 222, 128, 255};
}

Color warn_color()
{
    return Color{251, 191, 36, 255};
}

void draw_metric(
    int x,
    int y,
    int width,
    const char *label,
    const std::string &value,
    Color value_color
)
{
    DrawRectangle(
        x,
        y,
        width,
        72,
        card_color()
    );

    DrawText(
        label,
        x + 12,
        y + 10,
        16,
        muted_color()
    );

    DrawText(
        value.c_str(),
        x + 12,
        y + 34,
        24,
        value_color
    );
}

std::string fixed(double value, int decimals)
{
    std::ostringstream out;
    out << std::fixed
        << std::setprecision(decimals)
        << value;
    return out.str();
}

uint64_t steady_now_ns()
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<
            std::chrono::nanoseconds
        >(
            Clock::now().time_since_epoch()
        ).count()
    );
}

} /* namespace */


int main()
{
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

    const char *frame_log_path =
        std::getenv("CAN_FRAME_LOG");

    if (
        frame_log_path == nullptr ||
        frame_log_path[0] == '\0'
    )
    {
        frame_log_path =
            "can-frames.csv";
    }

    const char *metrics_log_path =
        std::getenv("CAN_METRICS_LOG");

    if (
        metrics_log_path == nullptr ||
        metrics_log_path[0] == '\0'
    )
    {
        metrics_log_path =
            "can-metrics.csv";
    }

    const uint64_t started_ns =
        steady_now_ns();

    CanBackend backend{};

    const SilKitCanBackendConfig config =
    {
        "CanLiveMonitor",
        "MonitorCAN",
        kNetworkName,
        registry_uri,
        kBitrate
    };

    if (!silkit_can_backend_create(
            &backend,
            &config))
    {
        std::fprintf(
            stderr,
            "Could not create CAN live monitor SIL Kit participant.\n"
        );
        return 1;
    }

    if (!silkit_can_backend_wait_ready(
            &backend,
            5000U))
    {
        std::fprintf(
            stderr,
            "CAN live monitor did not become ready.\n"
        );
        can_backend_close(&backend);
        return 1;
    }

    std::ofstream frame_log(
        frame_log_path,
        std::ios::out | std::ios::trunc
    );

    std::ofstream metrics_log(
        metrics_log_path,
        std::ios::out | std::ios::trunc
    );

    if (!frame_log || !metrics_log)
    {
        std::fprintf(
            stderr,
            "Could not open CAN monitor log files.\n"
        );
        can_backend_close(&backend);
        return 1;
    }

    frame_log
        << "time_us,delta_us,cob_id,type,node,dlc,data_hex,detail,"
        << "nominal_bits,worst_case_bits\n";

    metrics_log
        << "time_ms,frames_per_s,nominal_load_pct,worst_case_load_pct,"
        << "sync_period_ms,sync_jitter_ms,sync_jitter_max_ms,missed_sync_cycles,"
        << "rpdo4_per_s,tpdo4_per_s,observed_cycle_hz,"
        << "avg_rpdo_to_tpdo_ms,max_rpdo_to_tpdo_ms\n";

    SetConfigFlags(
        FLAG_WINDOW_RESIZABLE |
        FLAG_VSYNC_HINT
    );

    InitWindow(
        1040,
        690,
        "CANopen Live Monitor - Simple View"
    );

    SetTargetFPS(60);

    uint64_t last_frame_us = 0U;
    uint64_t last_metrics_log_us = 0U;
    uint64_t last_console_us = 0U;

    MonitorStats stats;
    std::deque<WireSample> wire_window;

    /*
     * Rolling 1-second timestamp windows.
     *
     * These are based on SIL Kit receive-callback timestamps, not on when the
     * GUI thread happens to drain its queue. That prevents a temporary GUI
     * stall/backlog from producing impossible rates such as 8000 Hz.
     */
    std::deque<uint64_t> rpdo4_window;
    std::deque<uint64_t> tpdo4_window;
    std::deque<uint64_t> sync_window;

    double rpdo_per_s = 0.0;
    double tpdo_per_s = 0.0;

    /*
     * Human-facing values are intentionally refreshed only 4 times/second.
     * The logger still records frames and metrics at full speed.
     */
    uint64_t last_display_update_us = 0U;
    double display_frames_per_s = 0.0;
    double display_load_pct = 0.0;
    double display_cycle_hz = 0.0;
    double display_sync_period_ms = 0.0;
    double display_sync_jitter_ms = 0.0;
    double display_latency_avg_ms = 0.0;
    double display_latency_max_ms = 0.0;
    uint64_t display_missed_sync = 0U;
    std::array<bool, kNodeCount> display_node_ok{};

    std::printf(
        "[CAN MONITOR] ready: %s / %s @ %u bit/s\n",
        registry_uri,
        kNetworkName,
        kBitrate
    );

    std::printf(
        "[CAN MONITOR] frame log: %s\n"
        "[CAN MONITOR] metrics log: %s\n",
        frame_log_path,
        metrics_log_path
    );

    std::fflush(stdout);

    while (!WindowShouldClose())
    {
        const uint64_t now_ns =
            steady_now_ns();

        const uint64_t now_us =
            now_ns >= started_ns
                ? (now_ns - started_ns) / 1000U
                : 0U;

        unsigned drained = 0U;

        for (; drained < 8192U; ++drained)
        {
            CanFrame frame{};

            uint64_t rx_time_ns = 0U;

            if (!silkit_can_backend_receive_timestamped(
                    &backend,
                    &frame,
                    &rx_time_ns))
            {
                break;
            }

            const uint64_t frame_us =
                rx_time_ns >= started_ns
                    ? (rx_time_ns - started_ns) / 1000U
                    : 0U;

            const uint64_t delta_us =
                last_frame_us == 0U
                    ? 0U
                    : frame_us - last_frame_us;

            last_frame_us = frame_us;

            FrameInfo info =
                classify_frame(
                    frame,
                    frame_us,
                    delta_us,
                    stats
                );

            if (info.type == "RPDO4")
            {
                rpdo4_window.push_back(
                    frame_us
                );
            }
            else if (info.type == "TPDO4")
            {
                tpdo4_window.push_back(
                    frame_us
                );
            }
            else if (info.type == "SYNC")
            {
                sync_window.push_back(
                    frame_us
                );
            }

            stats.total_frames++;
            stats.last_frame_us = frame_us;

            const uint32_t nominal_bits =
                nominal_can_bits(frame.dlc);

            const uint32_t worst_bits =
                conservative_stuffed_bits(frame.dlc);

            wire_window.push_back(
                WireSample{
                    frame_us,
                    nominal_bits,
                    worst_bits
                }
            );

            frame_log
                << frame_us << ','
                << delta_us << ','
                << hex_id(frame.id) << ','
                << info.type << ','
                << static_cast<unsigned>(info.node) << ','
                << static_cast<unsigned>(frame.dlc) << ','
                << '"'
                << data_hex(frame)
                << '"' << ','
                << '"'
                << info.detail
                << '"' << ','
                << nominal_bits << ','
                << worst_bits
                << '\n';
        }

        const uint64_t window_start =
            now_us > 1000000U
                ? now_us - 1000000U
                : 0U;

        while (
            !wire_window.empty() &&
            wire_window.front().time_us <
                window_start
        )
        {
            wire_window.pop_front();
        }

        auto trim_timestamp_window =
            [window_start](
                std::deque<uint64_t> &window
            )
            {
                while (
                    !window.empty() &&
                    window.front() <
                        window_start
                )
                {
                    window.pop_front();
                }
            };

        trim_timestamp_window(
            rpdo4_window
        );

        trim_timestamp_window(
            tpdo4_window
        );

        trim_timestamp_window(
            sync_window
        );

        uint64_t nominal_bits_window = 0U;
        uint64_t worst_bits_window = 0U;

        for (const WireSample &sample : wire_window)
        {
            nominal_bits_window +=
                sample.nominal_bits;

            worst_bits_window +=
                sample.worst_case_bits;
        }

        const double frames_per_s =
            static_cast<double>(
                wire_window.size()
            );

        const double nominal_load_pct =
            100.0 *
            static_cast<double>(
                nominal_bits_window
            ) /
            static_cast<double>(
                kBitrate
            );

        const double worst_load_pct =
            100.0 *
            static_cast<double>(
                worst_bits_window
            ) /
            static_cast<double>(
                kBitrate
            );

        double latency_sum = 0.0;
        uint64_t latency_count = 0U;
        double latency_max = 0.0;

        for (const NodeStats &node : stats.nodes)
        {
            latency_sum += node.latency_sum_ms;
            latency_count += node.latency_count;
            latency_max =
                std::max(
                    latency_max,
                    node.latency_max_ms
                );
        }

        rpdo_per_s =
            static_cast<double>(
                rpdo4_window.size()
            );

        tpdo_per_s =
            static_cast<double>(
                tpdo4_window.size()
            );

        /*
         * The controller emits exactly one SYNC for each six-axis target
         * cycle. Using the timestamped SYNC window is therefore the cleanest
         * achieved cyclic-rate measurement.
         */
        const double observed_cycle_hz =
            static_cast<double>(
                sync_window.size()
            );

        const double latency_avg =
            latency_count == 0U
                ? 0.0
                : latency_sum /
                    static_cast<double>(
                        latency_count
                    );

        if (
            last_metrics_log_us == 0U ||
            now_us - last_metrics_log_us >= 100000U
        )
        {
            metrics_log
                << now_us / 1000U << ','
                << frames_per_s << ','
                << nominal_load_pct << ','
                << worst_load_pct << ','
                << stats.sync_period_latest_ms << ','
                << stats.sync_jitter_latest_ms << ','
                << stats.sync_jitter_max_ms << ','
                << stats.missed_sync_cycles << ','
                << rpdo_per_s << ','
                << tpdo_per_s << ','
                << observed_cycle_hz << ','
                << latency_avg << ','
                << latency_max
                << '\n';

            frame_log.flush();
            metrics_log.flush();
            last_metrics_log_us = now_us;
        }

        if (
            last_console_us == 0U ||
            now_us - last_console_us >= 1000000U
        )
        {
            std::printf(
                "[CAN MON] fps=%.0f load=%.1f%% nominal / %.1f%% stuffed "
                "SYNC=%.3f ms jitter=%.3f ms missed=%llu "
                "RPDO4=%.0f/s TPDO4=%.0f/s cycle=%.1f Hz latency=%.3f ms avg\n",
                frames_per_s,
                nominal_load_pct,
                worst_load_pct,
                stats.sync_period_latest_ms,
                stats.sync_jitter_latest_ms,
                static_cast<unsigned long long>(
                    stats.missed_sync_cycles
                ),
                rpdo_per_s,
                tpdo_per_s,
                observed_cycle_hz,
                latency_avg
            );

            std::fflush(stdout);
            last_console_us = now_us;
        }

        if (
            last_display_update_us == 0U ||
            now_us - last_display_update_us >= 250000U
        )
        {
            display_frames_per_s = frames_per_s;
            display_load_pct = worst_load_pct;
            display_cycle_hz = observed_cycle_hz;
            display_sync_period_ms =
                stats.sync_period_latest_ms;
            display_sync_jitter_ms =
                stats.sync_jitter_latest_ms;
            display_latency_avg_ms = latency_avg;
            display_latency_max_ms = latency_max;
            display_missed_sync =
                stats.missed_sync_cycles;

            for (std::size_t i = 0U; i < kNodeCount; ++i)
            {
                const NodeStats &node =
                    stats.nodes[i];

                display_node_ok[i] =
                    node.tpdo4_count > 0U &&
                    node.last_tpdo_us != 0U &&
                    now_us >= node.last_tpdo_us &&
                    (now_us - node.last_tpdo_us) < 500000U;
            }

            last_display_update_us = now_us;
        }

        BeginDrawing();

        ClearBackground(
            panel_color()
        );

        const int width =
            GetScreenWidth();

        const bool cyclic_active =
            display_cycle_hz > 10.0 &&
            !sync_window.empty() &&
            now_us >= sync_window.back() &&
            (now_us - sync_window.back()) < 100000U;

        const bool cycle_good =
            display_cycle_hz >= 450.0;

        const bool cycle_close =
            display_cycle_hz >= 350.0;

        const Color cycle_color =
            !cyclic_active
                ? muted_color()
                : cycle_good
                    ? good_color()
                    : cycle_close
                        ? warn_color()
                        : Color{248, 113, 113, 255};

        unsigned healthy_nodes = 0U;

        for (bool ok : display_node_ok)
        {
            if (ok)
            {
                healthy_nodes++;
            }
        }

        DrawText(
            "CANopen Communication Status",
            26,
            20,
            30,
            text_color()
        );

        DrawText(
            "Simple operator view - raw CAN traffic is logged, not flashed on screen",
            26,
            56,
            16,
            muted_color()
        );

        const char *overall =
            !cyclic_active
                ? "IDLE - waiting for cyclic motion"
                : cycle_good
                    ? "RUNNING - close to 500 Hz target"
                    : cycle_close
                        ? "RUNNING - below 500 Hz target"
                        : "RUNNING - well below 500 Hz target";

        DrawRectangle(
            26,
            88,
            width - 52,
            58,
            card_color()
        );

        DrawText(
            overall,
            44,
            104,
            24,
            cycle_color
        );

        const int gap = 14;
        const int metric_width =
            (width - 52 - 3 * gap) / 4;

        draw_metric(
            26,
            166,
            metric_width,
            "CONTROL RATE",
            cyclic_active
                ? fixed(display_cycle_hz, 0) + " Hz"
                : "--",
            cycle_color
        );

        draw_metric(
            26 + metric_width + gap,
            166,
            metric_width,
            "BUS LOAD (est.)",
            fixed(display_load_pct, 1) + "%",
            display_load_pct < 80.0
                ? good_color()
                : warn_color()
        );

        draw_metric(
            26 + 2 * (metric_width + gap),
            166,
            metric_width,
            "SYNC PERIOD",
            cyclic_active
                ? fixed(display_sync_period_ms, 2) + " ms"
                : "--",
            !cyclic_active
                ? muted_color()
                : display_sync_jitter_ms < 0.5
                    ? good_color()
                    : warn_color()
        );

        draw_metric(
            26 + 3 * (metric_width + gap),
            166,
            metric_width,
            "FEEDBACK RESPONSE",
            cyclic_active
                ? fixed(display_latency_avg_ms, 2) + " ms"
                : "--",
            text_color()
        );

        DrawText(
            "Target: 500 Hz / 2.00 ms",
            26,
            258,
            18,
            muted_color()
        );

        if (cyclic_active)
        {
            const std::string timing_note =
                "Observed jitter: " +
                fixed(display_sync_jitter_ms, 3) +
                " ms   |   Max response: " +
                fixed(display_latency_max_ms, 3) +
                " ms   |   2 ms target misses: " +
                std::to_string(display_missed_sync);

            DrawText(
                timing_note.c_str(),
                26,
                286,
                17,
                text_color()
            );
        }
        else
        {
            DrawText(
                "Start Preview or Production to measure the cyclic CAN loop.",
                26,
                286,
                17,
                text_color()
            );
        }

        DrawText(
            "AVATAR nodes",
            26,
            334,
            20,
            text_color()
        );

        const int node_gap = 12;
        const int node_width =
            (width - 52 - 5 * node_gap) / 6;

        for (std::size_t i = 0U;
             i < kNodeCount;
             ++i)
        {
            const int x =
                26 +
                static_cast<int>(i) *
                (node_width + node_gap);

            DrawRectangle(
                x,
                366,
                node_width,
                92,
                card_color()
            );

            const std::string title =
                "NODE " +
                std::to_string(i + 1U);

            DrawText(
                title.c_str(),
                x + 12,
                378,
                17,
                muted_color()
            );

            DrawText(
                display_node_ok[i]
                    ? "OK"
                    : (cyclic_active ? "WAIT" : "IDLE"),
                x + 12,
                408,
                26,
                display_node_ok[i]
                    ? good_color()
                    : muted_color()
            );
        }

        const std::string nodes_summary =
            "Feedback nodes active: " +
            std::to_string(healthy_nodes) +
            "/6";

        DrawText(
            nodes_summary.c_str(),
            26,
            478,
            18,
            healthy_nodes == 6U
                ? good_color()
                : muted_color()
        );

        DrawRectangle(
            26,
            516,
            width - 52,
            88,
            card_color()
        );

        DrawText(
            "LOGGING CONTINUES AT FULL SPEED",
            42,
            530,
            17,
            good_color()
        );

        DrawText(
            "can-frames.csv  - every observed CAN frame",
            42,
            556,
            15,
            text_color()
        );

        DrawText(
            "can-metrics.csv - communication metrics for MATLAB plots",
            42,
            578,
            15,
            text_color()
        );

        DrawText(
            "Display refresh: 4 Hz. Logging/measurement is not slowed down.",
            26,
            620,
            14,
            muted_color()
        );

        DrawText(
            "SIL Kit timing is PC observer timing; bus load is an estimate, not a physical CAN measurement.",
            26,
            644,
            13,
            warn_color()
        );

        EndDrawing();
    }

    frame_log.flush();
    metrics_log.flush();

    CloseWindow();
    can_backend_close(&backend);

    return 0;
}
