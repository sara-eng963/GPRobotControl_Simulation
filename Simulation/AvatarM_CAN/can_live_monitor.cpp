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
constexpr uint32_t kBurstResetUs = 50000U;
constexpr std::size_t kNodeCount = 6U;
constexpr std::size_t kRecentRows = 18U;
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
        << "rpdo4_per_s,tpdo4_per_s,avg_rpdo_to_tpdo_ms,max_rpdo_to_tpdo_ms\n";

    SetConfigFlags(
        FLAG_WINDOW_RESIZABLE |
        FLAG_VSYNC_HINT
    );

    InitWindow(
        1180,
        760,
        "CANopen Live Monitor - SIL Kit CAN1"
    );

    SetTargetFPS(60);

    const auto started =
        Clock::now();

    uint64_t last_frame_us = 0U;
    uint64_t last_metrics_log_us = 0U;
    uint64_t last_console_us = 0U;

    MonitorStats stats;
    std::deque<FrameInfo> recent;
    std::deque<WireSample> wire_window;

    uint64_t previous_window_rpdo =
        0U;
    uint64_t previous_window_tpdo =
        0U;
    uint64_t previous_window_sample_us =
        0U;

    double rpdo_per_s = 0.0;
    double tpdo_per_s = 0.0;

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
        const auto now_clock =
            Clock::now();

        const uint64_t now_us =
            static_cast<uint64_t>(
                std::chrono::duration_cast<
                    std::chrono::microseconds
                >(now_clock - started).count()
            );

        unsigned drained = 0U;

        for (; drained < 8192U; ++drained)
        {
            CanFrame frame{};

            const CanBackendResult result =
                can_backend_receive(
                    &backend,
                    &frame
                );

            if (result == CAN_BACKEND_WOULD_BLOCK)
            {
                break;
            }

            if (result != CAN_BACKEND_OK)
            {
                std::fprintf(
                    stderr,
                    "[CAN MONITOR] receive error\n"
                );
                break;
            }

            const auto frame_clock =
                Clock::now();

            const uint64_t frame_us =
                static_cast<uint64_t>(
                    std::chrono::duration_cast<
                        std::chrono::microseconds
                    >(frame_clock - started).count()
                );

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

            recent.push_front(info);

            while (recent.size() > kRecentRows)
            {
                recent.pop_back();
            }

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

        uint64_t rpdo_total = 0U;
        uint64_t tpdo_total = 0U;
        double latency_sum = 0.0;
        uint64_t latency_count = 0U;
        double latency_max = 0.0;

        for (const NodeStats &node : stats.nodes)
        {
            rpdo_total += node.rpdo4_count;
            tpdo_total += node.tpdo4_count;
            latency_sum += node.latency_sum_ms;
            latency_count += node.latency_count;
            latency_max =
                std::max(
                    latency_max,
                    node.latency_max_ms
                );
        }

        if (
            previous_window_sample_us == 0U ||
            now_us - previous_window_sample_us >= 1000000U
        )
        {
            const double elapsed_s =
                previous_window_sample_us == 0U
                    ? 1.0
                    : static_cast<double>(
                        now_us -
                        previous_window_sample_us
                    ) / 1000000.0;

            rpdo_per_s =
                static_cast<double>(
                    rpdo_total -
                    previous_window_rpdo
                ) /
                elapsed_s;

            tpdo_per_s =
                static_cast<double>(
                    tpdo_total -
                    previous_window_tpdo
                ) /
                elapsed_s;

            previous_window_rpdo = rpdo_total;
            previous_window_tpdo = tpdo_total;
            previous_window_sample_us = now_us;
        }

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
                "RPDO4=%.0f/s TPDO4=%.0f/s latency=%.3f ms avg\n",
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
                latency_avg
            );

            std::fflush(stdout);
            last_console_us = now_us;
        }

        BeginDrawing();

        ClearBackground(
            panel_color()
        );

        const int width =
            GetScreenWidth();

        DrawText(
            "CANopen Live Monitor",
            20,
            16,
            28,
            text_color()
        );

        DrawText(
            "SIL Kit CAN1 | Classical CAN 2.0A | 1 Mbit/s | 6 AVATAR nodes",
            20,
            50,
            16,
            muted_color()
        );

        const int gap = 10;
        const int metric_width =
            (width - 40 - 3 * gap) / 4;

        draw_metric(
            20,
            82,
            metric_width,
            "Observed frames / s",
            fixed(frames_per_s, 0),
            text_color()
        );

        draw_metric(
            20 + metric_width + gap,
            82,
            metric_width,
            "Estimated bus load",
            fixed(nominal_load_pct, 1) +
                "% / " +
                fixed(worst_load_pct, 1) +
                "%",
            worst_load_pct < 80.0
                ? good_color()
                : warn_color()
        );

        draw_metric(
            20 + 2 * (metric_width + gap),
            82,
            metric_width,
            "SYNC period / jitter",
            fixed(
                stats.sync_period_latest_ms,
                3
            ) +
                " / " +
                fixed(
                    stats.sync_jitter_latest_ms,
                    3
                ) +
                " ms",
            stats.sync_jitter_latest_ms < 0.5
                ? good_color()
                : warn_color()
        );

        draw_metric(
            20 + 3 * (metric_width + gap),
            82,
            metric_width,
            "Observed RPDO->TPDO",
            fixed(latency_avg, 3) +
                " ms avg",
            text_color()
        );

        DrawText(
            "Node activity",
            20,
            170,
            20,
            text_color()
        );

        const int node_gap = 8;
        const int node_width =
            (width - 40 - 5 * node_gap) / 6;

        for (std::size_t i = 0U;
             i < kNodeCount;
             ++i)
        {
            const NodeStats &node =
                stats.nodes[i];

            const int x =
                20 +
                static_cast<int>(i) *
                (node_width + node_gap);

            DrawRectangle(
                x,
                198,
                node_width,
                112,
                card_color()
            );

            const std::string title =
                "Node " +
                std::to_string(i + 1U);

            DrawText(
                title.c_str(),
                x + 10,
                208,
                18,
                text_color()
            );

            const std::string counts =
                "RPDO " +
                std::to_string(node.rpdo4_count) +
                "  TPDO " +
                std::to_string(node.tpdo4_count);

            DrawText(
                counts.c_str(),
                x + 10,
                236,
                14,
                muted_color()
            );

            const std::string latency =
                "lat " +
                fixed(
                    node.latency_latest_ms,
                    3
                ) +
                " ms";

            DrawText(
                latency.c_str(),
                x + 10,
                258,
                14,
                text_color()
            );

            const std::string positions =
                "T " +
                std::to_string(
                    node.target_position
                ) +
                "\nA " +
                std::to_string(
                    node.actual_position
                );

            DrawText(
                positions.c_str(),
                x + 10,
                280,
                12,
                muted_color()
            );
        }

        const int table_y = 330;

        DrawText(
            "Recent CAN frames",
            20,
            table_y,
            20,
            text_color()
        );

        DrawRectangle(
            20,
            table_y + 30,
            width - 40,
            28,
            card_color()
        );

        DrawText(
            "time ms",
            30,
            table_y + 37,
            14,
            muted_color()
        );

        DrawText(
            "COB-ID",
            120,
            table_y + 37,
            14,
            muted_color()
        );

        DrawText(
            "type",
            205,
            table_y + 37,
            14,
            muted_color()
        );

        DrawText(
            "node",
            300,
            table_y + 37,
            14,
            muted_color()
        );

        DrawText(
            "DLC",
            355,
            table_y + 37,
            14,
            muted_color()
        );

        DrawText(
            "delta us",
            405,
            table_y + 37,
            14,
            muted_color()
        );

        DrawText(
            "decoded",
            500,
            table_y + 37,
            14,
            muted_color()
        );

        int row_y =
            table_y + 62;

        for (const FrameInfo &frame : recent)
        {
            DrawText(
                fixed(
                    static_cast<double>(
                        frame.time_us
                    ) / 1000.0,
                    1
                ).c_str(),
                30,
                row_y,
                13,
                text_color()
            );

            DrawText(
                hex_id(frame.id).c_str(),
                120,
                row_y,
                13,
                text_color()
            );

            DrawText(
                frame.type.c_str(),
                205,
                row_y,
                13,
                frame.type == "SYNC"
                    ? good_color()
                    : text_color()
            );

            DrawText(
                frame.node == 0U
                    ? "-"
                    : std::to_string(
                        frame.node
                    ).c_str(),
                300,
                row_y,
                13,
                text_color()
            );

            DrawText(
                std::to_string(
                    frame.dlc
                ).c_str(),
                355,
                row_y,
                13,
                text_color()
            );

            DrawText(
                std::to_string(
                    frame.delta_us
                ).c_str(),
                405,
                row_y,
                13,
                muted_color()
            );

            DrawText(
                frame.detail.c_str(),
                500,
                row_y,
                13,
                muted_color()
            );

            row_y += 20;

            if (
                row_y >
                GetScreenHeight() - 58
            )
            {
                break;
            }
        }

        const std::string footer =
            "SYNC missed (within active bursts): " +
            std::to_string(
                stats.missed_sync_cycles
            ) +
            "   |   Max observed latency: " +
            fixed(latency_max, 3) +
            " ms   |   Logs: " +
            frame_log_path;

        DrawText(
            footer.c_str(),
            20,
            GetScreenHeight() - 42,
            13,
            muted_color()
        );

        DrawText(
            "Load is estimated from observed frame sizes; this SIL Kit setup is not a physical CAN arbitration/bit-timing model.",
            20,
            GetScreenHeight() - 22,
            12,
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
