#include "sim_can_bus.h"

#include "../../CANComm/CANopen/canopen_master.h"
#include "../../ServoDrive/AvatarM/avatar_m_drive.h"

#include "raylib.h"

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NODE_COUNT 6
#define TARGET_TEXT_CAPACITY 24
#define HISTORY_CAPACITY 240

static const Color BG = {14, 18, 24, 255};
static const Color PANEL = {24, 30, 39, 255};
static const Color PANEL_2 = {31, 39, 50, 255};
static const Color BORDER = {57, 68, 82, 255};
static const Color TEXT = {229, 235, 242, 255};
static const Color MUTED = {146, 159, 174, 255};
static const Color ACCENT = {74, 163, 255, 255};
static const Color GOOD = {77, 196, 123, 255};
static const Color WARN = {240, 177, 74, 255};
static const Color BAD = {235, 91, 91, 255};
static const Color TARGET_COLOR = {238, 166, 75, 255};
static const Color ACTUAL_COLOR = {83, 191, 226, 255};

typedef struct
{
    AvatarMSimBus bus;
    CanBackend backend;
    CanopenMaster master;

    int32_t targets[NODE_COUNT];
    char target_text[NODE_COUNT][TARGET_TEXT_CAPACITY];
    int editing_target;

    bool streaming;
    double stream_accumulator_s;

    int selected_node;

    float target_history[HISTORY_CAPACITY];
    float actual_history[HISTORY_CAPACITY];
    int history_count;
    double history_accumulator_s;

    char message[160];
    double message_time_s;

} TestBench;

static void set_message(
    TestBench *app,
    const char *message
)
{
    if (
        app == NULL ||
        message == NULL
    )
    {
        return;
    }

    snprintf(
        app->message,
        sizeof(app->message),
        "%s",
        message
    );

    app->message_time_s =
        3.0;
}

static bool point_in_rect(
    Vector2 p,
    Rectangle r
)
{
    return
        p.x >= r.x &&
        p.x <= r.x + r.width &&
        p.y >= r.y &&
        p.y <= r.y + r.height;
}

static bool draw_button(
    Rectangle r,
    const char *label,
    Color base,
    bool enabled
)
{
    const Vector2 mouse =
        GetMousePosition();

    const bool hovered =
        enabled &&
        point_in_rect(mouse, r);

    Color fill =
        enabled
            ? base
            : (Color){55, 61, 70, 255};

    if (hovered)
    {
        fill = ColorBrightness(fill, 0.12f);
    }

    DrawRectangleRounded(
        r,
        0.18f,
        8,
        fill
    );

    DrawRectangleRoundedLinesEx(
        r,
        0.18f,
        8,
        1.0f,
        hovered ? RAYWHITE : ColorBrightness(fill, -0.18f)
    );

    const int font_size =
        16;

    const int width =
        MeasureText(
            label,
            font_size
        );

    DrawText(
        label,
        (int)(r.x + (r.width - (float)width) * 0.5f),
        (int)(r.y + (r.height - (float)font_size) * 0.5f),
        font_size,
        enabled ? RAYWHITE : MUTED
    );

    return
        hovered &&
        IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

static const char *nmt_name(
    AvatarMNmtState state
)
{
    switch (state)
    {
        case AVATAR_M_NMT_STOPPED:
            return "STOPPED";

        case AVATAR_M_NMT_PRE_OPERATIONAL:
            return "PRE-OP";

        case AVATAR_M_NMT_OPERATIONAL:
            return "OP";

        default:
            return "?";
    }
}

static Color nmt_color(
    AvatarMNmtState state
)
{
    switch (state)
    {
        case AVATAR_M_NMT_OPERATIONAL:
            return GOOD;

        case AVATAR_M_NMT_PRE_OPERATIONAL:
            return WARN;

        case AVATAR_M_NMT_STOPPED:
            return BAD;

        default:
            return MUTED;
    }
}

static const char *heartbeat_name(
    const AvatarMDrive *drive
)
{
    if (
        drive == NULL ||
        !drive->heartbeat_seen
    )
    {
        return "--";
    }

    switch (drive->heartbeat_state)
    {
        case AVATAR_M_HEARTBEAT_STATE_BOOTUP:
            return "BOOT";

        case AVATAR_M_HEARTBEAT_STATE_OPERATIONAL:
            return "0x05";

        case AVATAR_M_HEARTBEAT_STATE_PRE_OPERATIONAL:
            return "0x7F";

        case AVATAR_M_HEARTBEAT_STATE_ALARM:
            return "ALARM";

        default:
            return "?";
    }
}

static bool parse_target_text(
    const char *text,
    int32_t *value
)
{
    if (
        text == NULL ||
        value == NULL ||
        text[0] == '\0' ||
        strcmp(text, "-") == 0
    )
    {
        return false;
    }

    char *end = NULL;

    const long long parsed =
        strtoll(
            text,
            &end,
            10
        );

    if (
        end == text ||
        *end != '\0' ||
        parsed < INT32_MIN ||
        parsed > INT32_MAX
    )
    {
        return false;
    }

    *value =
        (int32_t)parsed;

    return true;
}

static bool commit_targets(
    TestBench *app
)
{
    if (app == NULL)
    {
        return false;
    }

    int32_t parsed[NODE_COUNT];

    for (int i = 0; i < NODE_COUNT; ++i)
    {
        if (!parse_target_text(
                app->target_text[i],
                &parsed[i]))
        {
            char msg[100];

            snprintf(
                msg,
                sizeof(msg),
                "J%d target is not a valid signed 32-bit value.",
                i + 1
            );

            set_message(
                app,
                msg
            );

            return false;
        }
    }

    memcpy(
        app->targets,
        parsed,
        sizeof(parsed)
    );

    return true;
}

static void reset_history(
    TestBench *app
)
{
    if (app == NULL)
    {
        return;
    }

    app->history_count =
        0;

    app->history_accumulator_s =
        0.0;
}

static bool initialize_system(
    TestBench *app
)
{
    if (app == NULL)
    {
        return false;
    }

    memset(
        &app->master,
        0,
        sizeof(app->master)
    );

    if (!avatar_sim_bus_init(
            &app->bus,
            &app->backend))
    {
        return false;
    }

    CanopenMasterConfig config =
    {
        .backend = &app->backend,
        .node_ids = {1U, 2U, 3U, 4U, 5U, 6U},
        .node_count = NODE_COUNT,
        .heartbeat_timeout_ms = 1500U,
        .sdo_timeout_ms = 100U
    };

    if (!canopen_master_init(
            &app->master,
            &config))
    {
        return false;
    }

    app->streaming =
        false;

    app->stream_accumulator_s =
        0.0;

    app->editing_target =
        -1;

    return true;
}

static bool send_rpdo_only(
    TestBench *app
)
{
    if (
        app == NULL ||
        !commit_targets(app)
    )
    {
        return false;
    }

    for (int i = 0; i < NODE_COUNT; ++i)
    {
        const AvatarMDrive *drive =
            canopen_master_drive(
                &app->master,
                (size_t)i
            );

        CanFrame frame;

        if (
            drive == NULL ||
            !avatar_m_drive_build_rpdo4_target(
                drive,
                app->targets[i],
                &frame) ||
            can_backend_send(
                app->master.backend,
                &frame) != CAN_BACKEND_OK
        )
        {
            set_message(
                app,
                "Failed while sending RPDO4."
            );

            return false;
        }
    }

    if (!canopen_master_poll(
            &app->master,
            app->bus.now_ms))
    {
        set_message(
            app,
            "CANopen master poll failed."
        );

        return false;
    }

    set_message(
        app,
        "RPDO4 targets sent. They are cached until SYNC."
    );

    return true;
}

static bool send_sync_only(
    TestBench *app
)
{
    if (
        app == NULL ||
        !canopen_master_send_sync(
            &app->master))
    {
        if (app != NULL)
        {
            set_message(
                app,
                "SYNC send failed."
            );
        }

        return false;
    }

    set_message(
        app,
        "SYNC 0x080 sent. Cached targets released together."
    );

    return true;
}

static bool send_full_cycle(
    TestBench *app,
    bool show_message
)
{
    if (
        app == NULL ||
        !commit_targets(app)
    )
    {
        return false;
    }

    if (!canopen_master_send_target_cycle(
            &app->master,
            app->targets,
            NODE_COUNT))
    {
        set_message(
            app,
            "Full CANopen command cycle failed."
        );

        return false;
    }

    if (!canopen_master_poll(
            &app->master,
            app->bus.now_ms))
    {
        set_message(
            app,
            "CANopen master poll failed."
        );

        return false;
    }

    if (show_message)
    {
        set_message(
            app,
            "Sent 6 RPDO4 frames + one SYNC."
        );
    }

    return true;
}

static void send_nmt(
    TestBench *app,
    CanopenNmtCommand command,
    const char *message
)
{
    if (app == NULL)
    {
        return;
    }

    if (!canopen_master_send_nmt_all(
            &app->master,
            command))
    {
        set_message(
            app,
            "NMT send failed."
        );

        return;
    }

    (void)canopen_master_poll(
        &app->master,
        app->bus.now_ms
    );

    set_message(
        app,
        message
    );
}

static void update_target_editor(
    TestBench *app,
    const Rectangle fields[NODE_COUNT]
)
{
    if (app == NULL)
    {
        return;
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
        const Vector2 mouse =
            GetMousePosition();

        int selected =
            -1;

        for (int i = 0; i < NODE_COUNT; ++i)
        {
            if (point_in_rect(
                    mouse,
                    fields[i]))
            {
                selected =
                    i;

                break;
            }
        }

        app->editing_target =
            selected;
    }

    if (
        app->editing_target < 0 ||
        app->editing_target >= NODE_COUNT
    )
    {
        return;
    }

    char *text =
        app->target_text[
            app->editing_target
        ];

    size_t length =
        strlen(text);

    int ch =
        GetCharPressed();

    while (ch > 0)
    {
        const bool digit =
            ch >= '0' &&
            ch <= '9';

        const bool minus =
            ch == '-' &&
            length == 0U;

        if (
            (digit || minus) &&
            length + 1U < TARGET_TEXT_CAPACITY
        )
        {
            text[length++] =
                (char)ch;

            text[length] =
                '\0';
        }

        ch =
            GetCharPressed();
    }

    if (
        IsKeyPressed(KEY_BACKSPACE) &&
        length > 0U
    )
    {
        text[length - 1U] =
            '\0';
    }

    if (
        IsKeyPressed(KEY_ENTER) ||
        IsKeyPressed(KEY_KP_ENTER)
    )
    {
        app->editing_target =
            -1;

        (void)commit_targets(app);
    }
}

static void sample_history(
    TestBench *app,
    double dt
)
{
    if (
        app == NULL ||
        app->selected_node < 0 ||
        app->selected_node >= NODE_COUNT
    )
    {
        return;
    }

    app->history_accumulator_s +=
        dt;

    if (app->history_accumulator_s < 0.05)
    {
        return;
    }

    app->history_accumulator_s -=
        0.05;

    const int node =
        app->selected_node;

    const float target =
        app->bus.nodes[node].active_target_valid
            ? (float)app->bus.nodes[node].active_target_position
            : 0.0f;

    const float actual =
        (float)app->bus.nodes[node].actual_position;

    if (app->history_count < HISTORY_CAPACITY)
    {
        app->target_history[
            app->history_count
        ] =
            target;

        app->actual_history[
            app->history_count
        ] =
            actual;

        app->history_count++;
    }
    else
    {
        memmove(
            &app->target_history[0],
            &app->target_history[1],
            sizeof(float) *
                (HISTORY_CAPACITY - 1)
        );

        memmove(
            &app->actual_history[0],
            &app->actual_history[1],
            sizeof(float) *
                (HISTORY_CAPACITY - 1)
        );

        app->target_history[
            HISTORY_CAPACITY - 1
        ] =
            target;

        app->actual_history[
            HISTORY_CAPACITY - 1
        ] =
            actual;
    }
}

static void decode_frame_label(
    const CanFrame *frame,
    char *buffer,
    size_t capacity
)
{
    if (
        frame == NULL ||
        buffer == NULL ||
        capacity == 0U
    )
    {
        return;
    }

    if (frame->id == 0x000U)
    {
        snprintf(
            buffer,
            capacity,
            "NMT"
        );
    }
    else if (frame->id == 0x080U)
    {
        snprintf(
            buffer,
            capacity,
            "SYNC"
        );
    }
    else if (
        frame->id >= 0x501U &&
        frame->id <= 0x506U
    )
    {
        snprintf(
            buffer,
            capacity,
            "RPDO4 J%u",
            (unsigned)(frame->id - 0x500U)
        );
    }
    else if (
        frame->id >= 0x481U &&
        frame->id <= 0x486U
    )
    {
        snprintf(
            buffer,
            capacity,
            "TPDO4 J%u",
            (unsigned)(frame->id - 0x480U)
        );
    }
    else if (
        frame->id >= 0x701U &&
        frame->id <= 0x706U
    )
    {
        snprintf(
            buffer,
            capacity,
            "Heartbeat J%u",
            (unsigned)(frame->id - 0x700U)
        );
    }
    else if (
        frame->id >= 0x601U &&
        frame->id <= 0x606U
    )
    {
        snprintf(
            buffer,
            capacity,
            "SDO req J%u",
            (unsigned)(frame->id - 0x600U)
        );
    }
    else if (
        frame->id >= 0x581U &&
        frame->id <= 0x586U
    )
    {
        snprintf(
            buffer,
            capacity,
            "SDO rsp J%u",
            (unsigned)(frame->id - 0x580U)
        );
    }
    else
    {
        snprintf(
            buffer,
            capacity,
            "CAN"
        );
    }
}

static void draw_panel(
    Rectangle r
)
{
    DrawRectangleRounded(
        r,
        0.025f,
        8,
        PANEL
    );

    DrawRectangleRoundedLinesEx(
        r,
        0.025f,
        8,
        1.0f,
        BORDER
    );
}

static void draw_badge(
    Rectangle r,
    const char *text,
    Color color
)
{
    DrawRectangleRounded(
        r,
        0.45f,
        8,
        ColorAlpha(color, 0.16f)
    );

    DrawRectangleRoundedLinesEx(
        r,
        0.45f,
        8,
        1.0f,
        ColorAlpha(color, 0.65f)
    );

    DrawText(
        text,
        (int)r.x + 10,
        (int)r.y + 5,
        14,
        color
    );
}

static void draw_table(
    TestBench *app,
    Rectangle panel,
    Rectangle fields[NODE_COUNT]
)
{
    draw_panel(panel);

    DrawText(
        "Six AVATAR M CANopen Nodes",
        (int)panel.x + 18,
        (int)panel.y + 14,
        20,
        TEXT
    );

    DrawText(
        "Target input is raw 0x607A position units",
        (int)panel.x + 325,
        (int)panel.y + 18,
        14,
        MUTED
    );

    const float header_y =
        panel.y + 54.0f;

    const float row_h =
        37.0f;

    const float col_x[] =
    {
        panel.x + 18.0f,
        panel.x + 72.0f,
        panel.x + 160.0f,
        panel.x + 292.0f,
        panel.x + 408.0f,
        panel.x + 524.0f,
        panel.x + 642.0f,
        panel.x + 760.0f
    };

    const char *headers[] =
    {
        "Node",
        "NMT",
        "Target",
        "Cached",
        "Active",
        "Sim actual",
        "Last TPDO",
        "HB"
    };

    for (int i = 0; i < 8; ++i)
    {
        DrawText(
            headers[i],
            (int)col_x[i],
            (int)header_y,
            13,
            MUTED
        );
    }

    DrawLine(
        (int)panel.x + 14,
        (int)header_y + 23,
        (int)(panel.x + panel.width - 14),
        (int)header_y + 23,
        BORDER
    );

    for (int i = 0; i < NODE_COUNT; ++i)
    {
        const float y =
            header_y + 31.0f +
            (float)i * row_h;

        Rectangle row =
        {
            panel.x + 10.0f,
            y - 5.0f,
            panel.width - 20.0f,
            row_h - 2.0f
        };

        if (i == app->selected_node)
        {
            DrawRectangleRounded(
                row,
                0.10f,
                6,
                ColorAlpha(ACCENT, 0.10f)
            );
        }

        if (
            IsMouseButtonPressed(MOUSE_BUTTON_LEFT) &&
            point_in_rect(
                GetMousePosition(),
                (Rectangle){
                    col_x[0] - 5.0f,
                    y - 5.0f,
                    45.0f,
                    28.0f
                })
        )
        {
            app->selected_node =
                i;

            reset_history(app);
        }

        DrawText(
            TextFormat("J%d", i + 1),
            (int)col_x[0],
            (int)y,
            16,
            i == app->selected_node
                ? ACCENT
                : TEXT
        );

        DrawText(
            nmt_name(
                app->bus.nodes[i].nmt_state
            ),
            (int)col_x[1],
            (int)y,
            14,
            nmt_color(
                app->bus.nodes[i].nmt_state
            )
        );

        fields[i] =
            (Rectangle)
            {
                col_x[2] - 5.0f,
                y - 6.0f,
                116.0f,
                27.0f
            };

        DrawRectangleRounded(
            fields[i],
            0.12f,
            6,
            i == app->editing_target
                ? ColorAlpha(ACCENT, 0.16f)
                : PANEL_2
        );

        DrawRectangleRoundedLinesEx(
            fields[i],
            0.12f,
            6,
            1.0f,
            i == app->editing_target
                ? ACCENT
                : BORDER
        );

        DrawText(
            app->target_text[i],
            (int)fields[i].x + 8,
            (int)fields[i].y + 5,
            15,
            TEXT
        );

        const AvatarMNodeSim *node =
            &app->bus.nodes[i];

        DrawText(
            node->cached_target_valid
                ? TextFormat(
                    "%d",
                    node->cached_target_position)
                : "--",
            (int)col_x[3],
            (int)y,
            14,
            node->cached_target_valid
                ? WARN
                : MUTED
        );

        DrawText(
            node->active_target_valid
                ? TextFormat(
                    "%d",
                    node->active_target_position)
                : "--",
            (int)col_x[4],
            (int)y,
            14,
            node->active_target_valid
                ? TARGET_COLOR
                : MUTED
        );

        DrawText(
            TextFormat(
                "%d",
                node->actual_position
            ),
            (int)col_x[5],
            (int)y,
            14,
            ACTUAL_COLOR
        );

        const AvatarMDrive *drive =
            canopen_master_drive(
                &app->master,
                (size_t)i
            );

        DrawText(
            (
                drive != NULL &&
                drive->feedback_valid
            )
                ? TextFormat(
                    "%d",
                    drive->feedback.actual_position)
                : "--",
            (int)col_x[6],
            (int)y,
            14,
            TEXT
        );

        DrawText(
            heartbeat_name(drive),
            (int)col_x[7],
            (int)y,
            14,
            (
                drive != NULL &&
                drive->heartbeat_state ==
                    AVATAR_M_HEARTBEAT_STATE_OPERATIONAL
            )
                ? GOOD
                : MUTED
        );
    }
}

static void draw_can_log(
    TestBench *app,
    Rectangle panel
)
{
    draw_panel(panel);

    DrawText(
        "CAN Monitor",
        (int)panel.x + 18,
        (int)panel.y + 14,
        20,
        TEXT
    );

    DrawText(
        "Newest frames first",
        (int)panel.x + 145,
        (int)panel.y + 18,
        13,
        MUTED
    );

    const int max_rows =
        (int)((panel.height - 62.0f) / 20.0f);

    const size_t count =
        avatar_sim_bus_log_count(
            &app->bus
        );

    const int rows =
        (int)(
            count < (size_t)max_rows
                ? count
                : (size_t)max_rows
        );

    for (int i = 0; i < rows; ++i)
    {
        const AvatarSimCanLogEntry *entry =
            avatar_sim_bus_log_newest(
                &app->bus,
                (size_t)i
            );

        if (entry == NULL)
        {
            continue;
        }

        char label[32];

        decode_frame_label(
            &entry->frame,
            label,
            sizeof(label)
        );

        char data[3 * CAN_CLASSIC_MAX_DATA_BYTES + 1];

        data[0] =
            '\0';

        size_t offset =
            0U;

        for (
            uint8_t b = 0U;
            b < entry->frame.dlc &&
            b < CAN_CLASSIC_MAX_DATA_BYTES;
            ++b
        )
        {
            const int written =
                snprintf(
                    &data[offset],
                    sizeof(data) - offset,
                    "%02X%s",
                    entry->frame.data[b],
                    b + 1U < entry->frame.dlc
                        ? " "
                        : ""
                );

            if (
                written <= 0 ||
                (size_t)written >=
                    sizeof(data) - offset
            )
            {
                break;
            }

            offset +=
                (size_t)written;
        }

        const int y =
            (int)panel.y + 48 +
            i * 20;

        DrawText(
            TextFormat(
                "%6ums",
                entry->timestamp_ms
            ),
            (int)panel.x + 18,
            y,
            13,
            MUTED
        );

        DrawText(
            entry->direction ==
                AVATAR_SIM_LOG_CONTROLLER_TX
                ? "TX"
                : "RX",
            (int)panel.x + 88,
            y,
            13,
            entry->direction ==
                AVATAR_SIM_LOG_CONTROLLER_TX
                ? TARGET_COLOR
                : ACTUAL_COLOR
        );

        DrawText(
            TextFormat(
                "0x%03X",
                entry->frame.id
            ),
            (int)panel.x + 118,
            y,
            13,
            TEXT
        );

        DrawText(
            TextFormat(
                "%u",
                entry->frame.dlc
            ),
            (int)panel.x + 178,
            y,
            13,
            MUTED
        );

        DrawText(
            data,
            (int)panel.x + 202,
            y,
            13,
            TEXT
        );

        DrawText(
            label,
            (int)panel.x + 395,
            y,
            13,
            MUTED
        );
    }

    if (rows == 0)
    {
        DrawText(
            "No frames yet. Start the nodes and send a command.",
            (int)panel.x + 18,
            (int)panel.y + 58,
            14,
            MUTED
        );
    }
}

static void draw_plot(
    TestBench *app,
    Rectangle panel
)
{
    draw_panel(panel);

    const int node =
        app->selected_node;

    DrawText(
        TextFormat(
            "J%d Position Trace",
            node + 1
        ),
        (int)panel.x + 18,
        (int)panel.y + 14,
        20,
        TEXT
    );

    DrawCircle(
        (int)panel.x + 190,
        (int)panel.y + 24,
        4.0f,
        TARGET_COLOR
    );

    DrawText(
        "active target",
        (int)panel.x + 200,
        (int)panel.y + 17,
        13,
        MUTED
    );

    DrawCircle(
        (int)panel.x + 305,
        (int)panel.y + 24,
        4.0f,
        ACTUAL_COLOR
    );

    DrawText(
        "sim actual",
        (int)panel.x + 315,
        (int)panel.y + 17,
        13,
        MUTED
    );

    Rectangle plot =
    {
        panel.x + 18.0f,
        panel.y + 52.0f,
        panel.width - 36.0f,
        panel.height - 112.0f
    };

    DrawRectangle(
        (int)plot.x,
        (int)plot.y,
        (int)plot.width,
        (int)plot.height,
        (Color){17, 22, 29, 255}
    );

    DrawRectangleLinesEx(
        plot,
        1.0f,
        BORDER
    );

    if (app->history_count >= 2)
    {
        float min_v =
            app->target_history[0];

        float max_v =
            app->target_history[0];

        for (int i = 0; i < app->history_count; ++i)
        {
            if (app->target_history[i] < min_v)
            {
                min_v =
                    app->target_history[i];
            }

            if (app->actual_history[i] < min_v)
            {
                min_v =
                    app->actual_history[i];
            }

            if (app->target_history[i] > max_v)
            {
                max_v =
                    app->target_history[i];
            }

            if (app->actual_history[i] > max_v)
            {
                max_v =
                    app->actual_history[i];
            }
        }

        if (fabsf(max_v - min_v) < 1.0f)
        {
            min_v -=
                1.0f;

            max_v +=
                1.0f;
        }

        const float margin =
            (max_v - min_v) *
            0.10f;

        min_v -=
            margin;

        max_v +=
            margin;

        for (int g = 1; g < 4; ++g)
        {
            const float gy =
                plot.y +
                plot.height *
                    (float)g /
                    4.0f;

            DrawLine(
                (int)plot.x,
                (int)gy,
                (int)(plot.x + plot.width),
                (int)gy,
                ColorAlpha(BORDER, 0.55f)
            );
        }

        for (int i = 1; i < app->history_count; ++i)
        {
            const float x0 =
                plot.x +
                plot.width *
                    (float)(i - 1) /
                    (float)(HISTORY_CAPACITY - 1);

            const float x1 =
                plot.x +
                plot.width *
                    (float)i /
                    (float)(HISTORY_CAPACITY - 1);

            const float target_y0 =
                plot.y +
                plot.height *
                    (
                        1.0f -
                        (
                            app->target_history[i - 1] -
                            min_v
                        ) /
                        (max_v - min_v)
                    );

            const float target_y1 =
                plot.y +
                plot.height *
                    (
                        1.0f -
                        (
                            app->target_history[i] -
                            min_v
                        ) /
                        (max_v - min_v)
                    );

            const float actual_y0 =
                plot.y +
                plot.height *
                    (
                        1.0f -
                        (
                            app->actual_history[i - 1] -
                            min_v
                        ) /
                        (max_v - min_v)
                    );

            const float actual_y1 =
                plot.y +
                plot.height *
                    (
                        1.0f -
                        (
                            app->actual_history[i] -
                            min_v
                        ) /
                        (max_v - min_v)
                    );

            DrawLineEx(
                (Vector2){x0, target_y0},
                (Vector2){x1, target_y1},
                2.0f,
                TARGET_COLOR
            );

            DrawLineEx(
                (Vector2){x0, actual_y0},
                (Vector2){x1, actual_y1},
                2.0f,
                ACTUAL_COLOR
            );
        }

        DrawText(
            TextFormat(
                "%.0f",
                max_v
            ),
            (int)plot.x + 5,
            (int)plot.y + 4,
            12,
            MUTED
        );

        DrawText(
            TextFormat(
                "%.0f",
                min_v
            ),
            (int)plot.x + 5,
            (int)(plot.y + plot.height - 17),
            12,
            MUTED
        );
    }
    else
    {
        DrawText(
            "Trace begins as the simulation runs.",
            (int)plot.x + 14,
            (int)plot.y + 14,
            14,
            MUTED
        );
    }

    const AvatarMNodeSim *sim =
        &app->bus.nodes[node];

    const AvatarMDrive *drive =
        canopen_master_drive(
            &app->master,
            (size_t)node
        );

    char active_text[32];
    char tpdo_text[32];

    if (sim->active_target_valid)
    {
        snprintf(
            active_text,
            sizeof(active_text),
            "%d",
            sim->active_target_position
        );
    }
    else
    {
        snprintf(
            active_text,
            sizeof(active_text),
            "--"
        );
    }

    if (
        drive != NULL &&
        drive->feedback_valid
    )
    {
        snprintf(
            tpdo_text,
            sizeof(tpdo_text),
            "%d",
            drive->feedback.actual_position
        );
    }
    else
    {
        snprintf(
            tpdo_text,
            sizeof(tpdo_text),
            "--"
        );
    }

    DrawText(
        TextFormat(
            "Active: %s    Sim actual: %d    Last TPDO: %s",
            active_text,
            sim->actual_position,
            tpdo_text
        ),
        (int)panel.x + 18,
        (int)(panel.y + panel.height - 42),
        14,
        TEXT
    );

    DrawText(
        app->bus.demo_motion_enabled
            ? "Demo motion ON: bounded visual follower only — not AVATAR dynamics."
            : "Protocol-only mode: SYNC changes active target, not physical position.",
        (int)panel.x + 18,
        (int)(panel.y + panel.height - 22),
        12,
        app->bus.demo_motion_enabled
            ? WARN
            : MUTED
    );
}

int main(void)
{
    SetConfigFlags(
        FLAG_WINDOW_RESIZABLE |
        FLAG_MSAA_4X_HINT
    );

    InitWindow(
        1380,
        820,
        "AVATAR M-Series CANopen Test Bench"
    );

    SetTargetFPS(
        60
    );

    TestBench app;

    memset(
        &app,
        0,
        sizeof(app)
    );

    app.selected_node =
        0;

    app.editing_target =
        -1;

    const int32_t initial_targets[NODE_COUNT] =
    {
        10000,
        20000,
        30000,
        40000,
        50000,
        60000
    };

    for (int i = 0; i < NODE_COUNT; ++i)
    {
        app.targets[i] =
            initial_targets[i];

        snprintf(
            app.target_text[i],
            sizeof(app.target_text[i]),
            "%d",
            initial_targets[i]
        );
    }

    if (!initialize_system(
            &app))
    {
        CloseWindow();
        return 1;
    }

    double tick_fraction_ms =
        0.0;

    while (!WindowShouldClose())
    {
        const double dt =
            (double)GetFrameTime();

        tick_fraction_ms +=
            dt * 1000.0;

        const uint32_t elapsed_ms =
            (uint32_t)tick_fraction_ms;

        tick_fraction_ms -=
            (double)elapsed_ms;

        if (elapsed_ms > 0U)
        {
            avatar_sim_bus_tick(
                &app.bus,
                elapsed_ms
            );

            (void)canopen_master_poll(
                &app.master,
                app.bus.now_ms
            );
        }

        if (app.streaming)
        {
            app.stream_accumulator_s +=
                dt;

            while (
                app.stream_accumulator_s >= 0.05
            )
            {
                (void)send_full_cycle(
                    &app,
                    false
                );

                app.stream_accumulator_s -=
                    0.05;
            }
        }

        sample_history(
            &app,
            dt
        );

        if (app.message_time_s > 0.0)
        {
            app.message_time_s -=
                dt;
        }

        const int width =
            GetScreenWidth();

        const int height =
            GetScreenHeight();

        const float margin =
            18.0f;

        BeginDrawing();

        ClearBackground(
            BG
        );

        DrawText(
            "AVATAR M-Series CANopen Test Bench",
            22,
            18,
            28,
            TEXT
        );

        DrawText(
            "Real project CANopen/AVATAR code + six protocol models",
            22,
            51,
            14,
            MUTED
        );

        draw_badge(
            (Rectangle){
                (float)width - 375.0f,
                20.0f,
                117.0f,
                26.0f
            },
            "Classic CAN",
            GOOD
        );

        draw_badge(
            (Rectangle){
                (float)width - 248.0f,
                20.0f,
                92.0f,
                26.0f
            },
            "1 Mbps",
            ACCENT
        );

        draw_badge(
            (Rectangle){
                (float)width - 146.0f,
                20.0f,
                124.0f,
                26.0f
            },
            "6 nodes",
            WARN
        );

        Rectangle controls =
        {
            margin,
            80.0f,
            (float)width - 2.0f * margin,
            70.0f
        };

        draw_panel(
            controls
        );

        const float by =
            controls.y + 15.0f;

        if (draw_button(
                (Rectangle){controls.x + 15.0f, by, 100.0f, 39.0f},
                "NMT START",
                GOOD,
                true))
        {
            send_nmt(
                &app,
                CANOPEN_NMT_START,
                "Broadcast NMT Start -> all six nodes Operational."
            );
        }

        if (draw_button(
                (Rectangle){controls.x + 123.0f, by, 92.0f, 39.0f},
                "PRE-OP",
                WARN,
                true))
        {
            send_nmt(
                &app,
                CANOPEN_NMT_PRE_OPERATIONAL,
                "Broadcast NMT Pre-operational."
            );
        }

        if (draw_button(
                (Rectangle){controls.x + 223.0f, by, 78.0f, 39.0f},
                "STOP",
                BAD,
                true))
        {
            send_nmt(
                &app,
                CANOPEN_NMT_STOP,
                "Broadcast NMT Stop."
            );
        }

        if (draw_button(
                (Rectangle){controls.x + 309.0f, by, 108.0f, 39.0f},
                "RESET COMM",
                (Color){112, 96, 167, 255},
                true))
        {
            send_nmt(
                &app,
                CANOPEN_NMT_RESET_COMMUNICATION,
                "Reset communication -> boot-up frames received."
            );
        }

        DrawLine(
            (int)controls.x + 433,
            (int)controls.y + 11,
            (int)controls.x + 433,
            (int)(controls.y + controls.height - 11),
            BORDER
        );

        if (draw_button(
                (Rectangle){controls.x + 449.0f, by, 128.0f, 39.0f},
                "SEND RPDO4",
                (Color){165, 116, 64, 255},
                true))
        {
            (void)send_rpdo_only(
                &app
            );
        }

        if (draw_button(
                (Rectangle){controls.x + 585.0f, by, 92.0f, 39.0f},
                "SYNC",
                ACCENT,
                true))
        {
            (void)send_sync_only(
                &app
            );
        }

        if (draw_button(
                (Rectangle){controls.x + 685.0f, by, 112.0f, 39.0f},
                "FULL CYCLE",
                (Color){76, 139, 169, 255},
                true))
        {
            (void)send_full_cycle(
                &app,
                true
            );
        }

        if (draw_button(
                (Rectangle){controls.x + 805.0f, by, 112.0f, 39.0f},
                app.streaming
                    ? "STOP STREAM"
                    : "STREAM 20Hz",
                app.streaming
                    ? BAD
                    : (Color){70, 132, 112, 255},
                true))
        {
            app.streaming =
                !app.streaming;

            app.stream_accumulator_s =
                0.0;

            set_message(
                &app,
                app.streaming
                    ? "20 Hz visual command stream started."
                    : "Command stream stopped."
            );
        }

        if (draw_button(
                (Rectangle){controls.x + 925.0f, by, 145.0f, 39.0f},
                app.bus.demo_motion_enabled
                    ? "DEMO MOTION: ON"
                    : "DEMO MOTION: OFF",
                app.bus.demo_motion_enabled
                    ? WARN
                    : (Color){73, 82, 94, 255},
                true))
        {
            avatar_sim_bus_set_demo_motion(
                &app.bus,
                !app.bus.demo_motion_enabled
            );

            set_message(
                &app,
                app.bus.demo_motion_enabled
                    ? "Visualization follower enabled. It is NOT vendor motor dynamics."
                    : "Returned to protocol-only motor model."
            );
        }

        if (draw_button(
                (Rectangle){controls.x + 1078.0f, by, 92.0f, 39.0f},
                "CLEAR LOG",
                (Color){73, 82, 94, 255},
                true))
        {
            avatar_sim_bus_clear_log(
                &app.bus
            );
        }

        const bool healthy =
            canopen_master_healthy(
                &app.master,
                app.bus.now_ms
            );

        const bool ready =
            canopen_master_ready_for_motion(
                &app.master,
                app.bus.now_ms
            );

        DrawText(
            healthy
                ? "COMM HEALTHY"
                : "COMM WAIT",
            (int)controls.x + 1185,
            (int)by + 1,
            13,
            healthy
                ? GOOD
                : WARN
        );

        DrawText(
            ready
                ? "MOTION READY"
                : "NOT READY",
            (int)controls.x + 1185,
            (int)by + 20,
            13,
            ready
                ? GOOD
                : MUTED
        );

        Rectangle fields[NODE_COUNT];

        const float table_y =
            164.0f;

        Rectangle table =
        {
            margin,
            table_y,
            (float)width - 2.0f * margin,
            303.0f
        };

        draw_table(
            &app,
            table,
            fields
        );

        update_target_editor(
            &app,
            fields
        );

        const float lower_y =
            480.0f;

        const float lower_h =
            (float)height -
            lower_y -
            margin;

        const float left_w =
            ((float)width - 3.0f * margin) *
            0.54f;

        Rectangle log_panel =
        {
            margin,
            lower_y,
            left_w,
            lower_h
        };

        Rectangle plot_panel =
        {
            margin * 2.0f + left_w,
            lower_y,
            (float)width -
                3.0f * margin -
                left_w,
            lower_h
        };

        draw_can_log(
            &app,
            log_panel
        );

        draw_plot(
            &app,
            plot_panel
        );

        if (app.message_time_s > 0.0)
        {
            const int msg_width =
                MeasureText(
                    app.message,
                    14
                );

            const Rectangle toast =
            {
                ((float)width - (float)msg_width) * 0.5f - 14.0f,
                444.0f,
                (float)msg_width + 28.0f,
                29.0f
            };

            DrawRectangleRounded(
                toast,
                0.28f,
                8,
                (Color){40, 48, 59, 245}
            );

            DrawRectangleRoundedLinesEx(
                toast,
                0.28f,
                8,
                1.0f,
                BORDER
            );

            DrawText(
                app.message,
                (int)toast.x + 14,
                (int)toast.y + 7,
                14,
                TEXT
            );
        }

        EndDrawing();
    }

    canopen_master_close(
        &app.master
    );

    CloseWindow();

    return 0;
}
