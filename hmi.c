/*
 * ============================================================================
 *  ROBOT MOTION CONSOLE - CARTESIAN HMI
 * ============================================================================
 *
 * Desktop-side raylib HMI for the robot control PC test.
 *
 * The UI now supports selecting the Cartesian path geometry:
 *
 *      - Straight line
 *      - Circular arc
 *      - Full circle
 *
 * IMPORTANT INTEGRATION NOTE
 * --------------------------
 * The live controller protocol in main.c currently executes only the straight
 * line command. Arc/full-circle geometry is implemented in ControlCore, but the
 * live 1 ms controller streaming bridge for those shapes has not been wired
 * into main.c yet. The HMI therefore lets the operator configure and inspect
 * those path types, but deliberately disables LIVE RUN for them instead of
 * pretending that the controller can execute them.
 *
 * This keeps the operator interface honest while the circular streaming bridge
 * is added later.
 *
 * ============================================================================
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <math.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "raylib.h"


/* ============================================================================
 * NETWORK CONFIGURATION
 * ============================================================================
 */

#define CONTROLLER_IP   "127.0.0.1"
#define CONTROLLER_PORT 5006
#define STATUS_PORT     5007

#define NUM_POSE_VALUES 6
#define PLAN_FLOAT_COUNT 15

#define PACKET_MAGIC        0x52425432U   /* ASCII "RBT2" */
#define STATUS_PACKET_MAGIC 0x53544132U   /* ASCII "STA2" */
#define STATUS_WORD_COUNT   13U


typedef enum
{
    COMMAND_PLAN_AND_RUN_LINE = 1,
    COMMAND_STOP              = 2

} CommandType;


/* ============================================================================
 * LOGICAL UI CANVAS
 * ============================================================================
 *
 * Everything is drawn in one fixed logical coordinate system. The whole HMI is
 * then scaled to the actual window size. This removes the old horizontal-scroll
 * layout and keeps the interface usable when the window is resized.
 * ============================================================================
 */

#define UI_WIDTH  1440.0f
#define UI_HEIGHT  900.0f

static float ui_scale = 1.0f;
static Vector2 ui_origin = {0.0f, 0.0f};


/* ============================================================================
 * THEME
 * ============================================================================
 */

static const Color C_BG         = { 15,  18,  24, 255};
static const Color C_PANEL      = { 22,  27,  35, 255};
static const Color C_PANEL_2    = { 27,  33,  43, 255};
static const Color C_FIELD      = { 31,  38,  49, 255};
static const Color C_FIELD_HI   = { 38,  49,  64, 255};
static const Color C_BORDER     = { 52,  63,  80, 255};
static const Color C_BORDER_HI  = { 78, 109, 153, 255};
static const Color C_TEXT       = {235, 239, 246, 255};
static const Color C_MUTED      = {150, 162, 181, 255};
static const Color C_FAINT      = {101, 113, 133, 255};
static const Color C_ACCENT     = { 88, 166, 255, 255};
static const Color C_ACCENT_2   = { 65, 128, 204, 255};
static const Color C_GOOD       = { 91, 201, 142, 255};
static const Color C_WARN       = {239, 184,  86, 255};
static const Color C_BAD        = {232, 105, 111, 255};
static const Color C_DISABLED   = { 74,  83,  98, 255};


/* ============================================================================
 * FONT
 * ============================================================================
 */

static Font ui_font;
static Font ui_font_bold;
static bool ui_font_owned = false;
static bool ui_font_bold_owned = false;


static Font load_first_font(
    const char *const *candidates,
    size_t count,
    int size,
    bool *owned
)
{
    if (owned != NULL)
    {
        *owned = false;
    }

    for (size_t i = 0; i < count; i++)
    {
        if (FileExists(candidates[i]))
        {
            Font font =
                LoadFontEx(
                    candidates[i],
                    size,
                    NULL,
                    0
                );

            SetTextureFilter(
                font.texture,
                TEXTURE_FILTER_BILINEAR
            );

            if (owned != NULL)
            {
                *owned = true;
            }

            return font;
        }
    }

    return GetFontDefault();
}


static void load_ui_fonts(void)
{
    const char *regular_candidates[] =
    {
        "/mnt/c/Windows/Fonts/segoeui.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf"
    };

    const char *bold_candidates[] =
    {
        "/mnt/c/Windows/Fonts/seguisb.ttf",
        "/mnt/c/Windows/Fonts/segoeuib.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf"
    };

    ui_font =
        load_first_font(
            regular_candidates,
            sizeof(regular_candidates) / sizeof(regular_candidates[0]),
            36,
            &ui_font_owned
        );

    ui_font_bold =
        load_first_font(
            bold_candidates,
            sizeof(bold_candidates) / sizeof(bold_candidates[0]),
            38,
            &ui_font_bold_owned
        );
}


static void unload_ui_fonts(void)
{
    if (ui_font_bold_owned)
    {
        UnloadFont(ui_font_bold);
    }

    if (ui_font_owned)
    {
        UnloadFont(ui_font);
    }
}


static void text_draw(
    const char *text,
    float x,
    float y,
    float size,
    Color color,
    bool bold
)
{
    Font font =
        bold
        ? ui_font_bold
        : ui_font;

    DrawTextEx(
        font,
        text,
        (Vector2){x, y},
        size,
        size * 0.015f,
        color
    );
}


static float text_width(
    const char *text,
    float size,
    bool bold
)
{
    Font font =
        bold
        ? ui_font_bold
        : ui_font;

    return
        MeasureTextEx(
            font,
            text,
            size,
            size * 0.015f
        ).x;
}


/* ============================================================================
 * PATH TYPE
 * ============================================================================
 */

typedef enum
{
    PATH_LINE = 0,
    PATH_ARC,
    PATH_FULL_CIRCLE

} PathType;


static const char *path_name(
    PathType type
)
{
    switch (type)
    {
        case PATH_LINE:
            return "Straight line";

        case PATH_ARC:
            return "Circular arc";

        case PATH_FULL_CIRCLE:
            return "Full circle";

        default:
            return "Unknown";
    }
}


/* ============================================================================
 * SMALL UI HELPERS
 * ============================================================================
 */

static Vector2 ui_mouse_position(void)
{
    Vector2 mouse =
        GetMousePosition();

    if (ui_scale <= 1e-6f)
    {
        return mouse;
    }

    return
        (Vector2)
        {
            (mouse.x - ui_origin.x) / ui_scale,
            (mouse.y - ui_origin.y) / ui_scale
        };
}


static void panel(
    Rectangle bounds,
    Color fill
)
{
    DrawRectangleRounded(
        bounds,
        0.045f,
        12,
        fill
    );

    DrawRectangleLinesEx(
        bounds,
        1.0f,
        C_BORDER
    );
}


static void status_dot(
    float x,
    float y,
    bool active,
    Color active_color
)
{
    DrawCircleV(
        (Vector2){x, y},
        5.0f,
        active
            ? active_color
            : C_DISABLED
    );
}


static bool button_ex(
    Rectangle bounds,
    const char *label,
    bool primary,
    bool danger,
    bool enabled
)
{
    Vector2 mouse =
        ui_mouse_position();

    bool hovered =
        enabled &&
        CheckCollisionPointRec(
            mouse,
            bounds
        );

    Color fill;
    Color border;
    Color foreground;

    if (!enabled)
    {
        fill = (Color){31, 36, 45, 255};
        border = C_BORDER;
        foreground = C_DISABLED;
    }
    else if (danger)
    {
        fill =
            hovered
            ? (Color){129, 51, 58, 255}
            : (Color){96, 42, 49, 255};

        border = C_BAD;
        foreground = C_TEXT;
    }
    else if (primary)
    {
        fill =
            hovered
            ? (Color){77, 151, 236, 255}
            : C_ACCENT_2;

        border = C_ACCENT;
        foreground = C_TEXT;
    }
    else
    {
        fill =
            hovered
            ? (Color){42, 51, 65, 255}
            : C_PANEL_2;

        border =
            hovered
            ? C_BORDER_HI
            : C_BORDER;

        foreground = C_TEXT;
    }

    DrawRectangleRounded(
        bounds,
        0.16f,
        10,
        fill
    );

    DrawRectangleLinesEx(
        bounds,
        1.0f,
        border
    );

    float font_size = 15.5f;

    float width =
        text_width(
            label,
            font_size,
            true
        );

    text_draw(
        label,
        bounds.x + bounds.width * 0.5f - width * 0.5f,
        bounds.y + bounds.height * 0.5f - font_size * 0.55f,
        font_size,
        foreground,
        true
    );

    return
        hovered &&
        IsMouseButtonPressed(
            MOUSE_BUTTON_LEFT
        );
}


static bool path_tab(
    Rectangle bounds,
    const char *title,
    const char *subtitle,
    bool selected
)
{
    Vector2 mouse =
        ui_mouse_position();

    bool hovered =
        CheckCollisionPointRec(
            mouse,
            bounds
        );

    Color fill =
        selected
        ? (Color){31, 55, 82, 255}
        : hovered
            ? (Color){31, 38, 49, 255}
            : (Color){24, 29, 38, 255};

    Color border =
        selected
        ? C_ACCENT
        : hovered
            ? C_BORDER_HI
            : C_BORDER;

    DrawRectangleRounded(
        bounds,
        0.09f,
        10,
        fill
    );

    DrawRectangleLinesEx(
        bounds,
        selected ? 2.0f : 1.0f,
        border
    );

    text_draw(
        title,
        bounds.x + 15.0f,
        bounds.y + 10.0f,
        16.5f,
        selected ? C_TEXT : (Color){210, 217, 229, 255},
        true
    );

    text_draw(
        subtitle,
        bounds.x + 15.0f,
        bounds.y + 33.0f,
        12.5f,
        C_MUTED,
        false
    );

    return
        hovered &&
        IsMouseButtonPressed(
            MOUSE_BUTTON_LEFT
        );
}


/* ============================================================================
 * NUMERIC FIELD
 * ============================================================================
 */

typedef struct
{
    char text[32];
    bool active;
    bool replace_on_type;

} NumericField;


static void numeric_field_set(
    NumericField *field,
    double value,
    int decimals
)
{
    if (field == NULL)
    {
        return;
    }

    snprintf(
        field->text,
        sizeof(field->text),
        "%.*f",
        decimals,
        value
    );

    field->active = false;
    field->replace_on_type = false;
}


static bool numeric_field_parse(
    const NumericField *field,
    double *value
)
{
    if (
        field == NULL ||
        value == NULL
    )
    {
        return false;
    }

    char *end = NULL;

    double parsed =
        strtod(
            field->text,
            &end
        );

    if (
        end == field->text ||
        *end != '\0' ||
        !isfinite(parsed)
    )
    {
        return false;
    }

    *value = parsed;

    return true;
}


static void update_numeric_field(
    NumericField *field,
    Rectangle bounds,
    bool enabled
)
{
    if (field == NULL)
    {
        return;
    }

    Vector2 mouse =
        ui_mouse_position();

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
        bool clicked =
            enabled &&
            CheckCollisionPointRec(
                mouse,
                bounds
            );

        if (clicked)
        {
            field->active = true;
            field->replace_on_type = true;
        }
        else
        {
            field->active = false;
            field->replace_on_type = false;
        }
    }

    if (
        !enabled ||
        !field->active
    )
    {
        return;
    }

    if (
        (IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)) &&
        IsKeyPressed(KEY_A)
    )
    {
        field->replace_on_type = true;
    }

    int key =
        GetCharPressed();

    while (key > 0)
    {
        bool allowed =
            (key >= '0' && key <= '9') ||
            key == '-' ||
            key == '+' ||
            key == '.' ||
            key == 'e' ||
            key == 'E';

        if (allowed)
        {
            if (field->replace_on_type)
            {
                field->text[0] = '\0';
                field->replace_on_type = false;
            }

            size_t length =
                strlen(field->text);

            if (
                length + 1 <
                sizeof(field->text)
            )
            {
                field->text[length] = (char)key;
                field->text[length + 1] = '\0';
            }
        }

        key =
            GetCharPressed();
    }

    if (
        IsKeyPressed(KEY_BACKSPACE) ||
        IsKeyPressed(KEY_DELETE)
    )
    {
        if (field->replace_on_type)
        {
            field->text[0] = '\0';
            field->replace_on_type = false;
        }
        else
        {
            size_t length =
                strlen(field->text);

            if (length > 0)
            {
                field->text[length - 1] = '\0';
            }
        }
    }

    if (
        IsKeyPressed(KEY_ENTER) ||
        IsKeyPressed(KEY_KP_ENTER) ||
        IsKeyPressed(KEY_ESCAPE)
    )
    {
        field->active = false;
        field->replace_on_type = false;
    }
}


static void draw_numeric_field(
    NumericField *field,
    Rectangle bounds,
    bool enabled
)
{
    Color fill =
        !enabled
        ? (Color){24, 29, 37, 255}
        : field->active
            ? C_FIELD_HI
            : C_FIELD;

    Color border =
        !enabled
        ? (Color){42, 49, 61, 255}
        : field->active
            ? C_ACCENT
            : C_BORDER;

    Color foreground =
        enabled
        ? C_TEXT
        : C_FAINT;

    DrawRectangleRounded(
        bounds,
        0.13f,
        8,
        fill
    );

    DrawRectangleLinesEx(
        bounds,
        field->active && enabled ? 1.7f : 1.0f,
        border
    );

    if (
        enabled &&
        field->active &&
        field->replace_on_type
    )
    {
        float selection_width =
            text_width(
                field->text,
                15.0f,
                false
            );

        DrawRectangleRounded(
            (Rectangle)
            {
                bounds.x + 8.0f,
                bounds.y + 7.0f,
                selection_width + 7.0f,
                bounds.height - 14.0f
            },
            0.15f,
            6,
            (Color){51, 88, 132, 255}
        );
    }

    text_draw(
        field->text,
        bounds.x + 10.0f,
        bounds.y + 9.0f,
        15.0f,
        foreground,
        false
    );

    if (
        enabled &&
        field->active &&
        !field->replace_on_type &&
        ((int)(GetTime() * 2.0) % 2 == 0)
    )
    {
        float caret_x =
            bounds.x +
            10.0f +
            text_width(
                field->text,
                15.0f,
                false
            ) +
            2.0f;

        DrawLineEx(
            (Vector2){caret_x, bounds.y + 8.0f},
            (Vector2){caret_x, bounds.y + bounds.height - 8.0f},
            1.5f,
            C_TEXT
        );
    }
}


/* ============================================================================
 * UDP SERIALIZATION
 * ============================================================================
 */

static uint32_t float_to_network_word(
    float value
)
{
    uint32_t bits = 0;

    memcpy(
        &bits,
        &value,
        sizeof(bits)
    );

    return htonl(bits);
}


static int send_line_plan_command(
    int socket_fd,
    const struct sockaddr_in *controller_address,
    uint32_t sequence,
    const float waypoint_a[NUM_POSE_VALUES],
    const float waypoint_b[NUM_POSE_VALUES],
    float tcp_speed,
    float tcp_accel,
    float tcp_jerk
)
{
    uint32_t packet[
        3 + PLAN_FLOAT_COUNT
    ];

    packet[0] = htonl(PACKET_MAGIC);
    packet[1] = htonl((uint32_t)COMMAND_PLAN_AND_RUN_LINE);
    packet[2] = htonl(sequence);

    int index = 3;

    for (int i = 0; i < NUM_POSE_VALUES; i++)
    {
        packet[index++] =
            float_to_network_word(
                waypoint_a[i]
            );
    }

    for (int i = 0; i < NUM_POSE_VALUES; i++)
    {
        packet[index++] =
            float_to_network_word(
                waypoint_b[i]
            );
    }

    packet[index++] = float_to_network_word(tcp_speed);
    packet[index++] = float_to_network_word(tcp_accel);
    packet[index++] = float_to_network_word(tcp_jerk);

    ssize_t sent =
        sendto(
            socket_fd,
            packet,
            sizeof(packet),
            0,
            (const struct sockaddr *)controller_address,
            sizeof(*controller_address)
        );

    return
        sent ==
        (ssize_t)sizeof(packet);
}


static int send_stop_command(
    int socket_fd,
    const struct sockaddr_in *controller_address,
    uint32_t sequence
)
{
    uint32_t packet[3];

    packet[0] = htonl(PACKET_MAGIC);
    packet[1] = htonl((uint32_t)COMMAND_STOP);
    packet[2] = htonl(sequence);

    ssize_t sent =
        sendto(
            socket_fd,
            packet,
            sizeof(packet),
            0,
            (const struct sockaddr *)controller_address,
            sizeof(*controller_address)
        );

    return
        sent ==
        (ssize_t)sizeof(packet);
}


/* ============================================================================
 * CONTROLLER LIVE STATUS
 * ============================================================================
 */

typedef struct
{
    bool valid;
    double lastReceiveTime;

    uint32_t motionState;
    uint32_t lastSequence;

    uint32_t trajectoryIndex;
    uint32_t trajectoryCount;

    uint32_t wkc;
    uint32_t expectedWkc;

    uint16_t statusword[6];

} ControllerStatus;


static const char *motion_state_name_hmi(
    uint32_t state
)
{
    switch (state)
    {
        case 0: return "WAITING";
        case 1: return "PLANNING";
        case 2: return "PREPOSITION";
        case 3: return "RUNNING";
        case 4: return "FINISHED";
        case 5: return "HOLD";
        default: return "UNKNOWN";
    }
}


static const char *cia402_state_name(
    uint16_t statusword
)
{
    uint16_t state =
        statusword &
        0x006F;

    switch (state)
    {
        case 0x0000: return "NOT READY";
        case 0x0040: return "SWITCH ON DISABLED";
        case 0x0021: return "READY TO SWITCH ON";
        case 0x0023: return "SWITCHED ON";
        case 0x0027: return "OP ENABLED";
        case 0x0007: return "QUICK STOP";
        case 0x000F: return "FAULT REACTION";
        case 0x0008: return "FAULT";
        default:     return "UNKNOWN";
    }
}


static bool cia402_operation_enabled(
    uint16_t statusword
)
{
    return
        (statusword & 0x006F) ==
        0x0027;
}


static void receive_controller_status(
    int status_socket,
    ControllerStatus *status
)
{
    if (
        status_socket < 0 ||
        status == NULL
    )
    {
        return;
    }

    for (;;)
    {
        uint32_t packet[STATUS_WORD_COUNT];

        ssize_t received =
            recvfrom(
                status_socket,
                packet,
                sizeof(packet),
                MSG_DONTWAIT,
                NULL,
                NULL
            );

        if (received < 0)
        {
            if (
                errno == EAGAIN ||
                errno == EWOULDBLOCK
            )
            {
                break;
            }

            perror("status recvfrom");
            break;
        }

        if (
            received !=
            (ssize_t)sizeof(packet)
        )
        {
            continue;
        }

        if (
            ntohl(packet[0]) !=
            STATUS_PACKET_MAGIC
        )
        {
            continue;
        }

        status->motionState = ntohl(packet[1]);
        status->lastSequence = ntohl(packet[2]);
        status->trajectoryIndex = ntohl(packet[3]);
        status->trajectoryCount = ntohl(packet[4]);
        status->wkc = ntohl(packet[5]);
        status->expectedWkc = ntohl(packet[6]);

        for (int joint = 0; joint < 6; joint++)
        {
            status->statusword[joint] =
                (uint16_t)ntohl(
                    packet[7 + joint]
                );
        }

        status->valid = true;
        status->lastReceiveTime = GetTime();
    }
}


/* ============================================================================
 * REFERENCE CASES
 * ============================================================================
 */

static void set_pose(
    NumericField fields[NUM_POSE_VALUES],
    const double values[NUM_POSE_VALUES]
)
{
    for (int i = 0; i < NUM_POSE_VALUES; i++)
    {
        numeric_field_set(
            &fields[i],
            values[i],
            i < 3 ? 6 : 2
        );
    }
}


static void load_reference_case(
    PathType path,
    NumericField a_fields[NUM_POSE_VALUES],
    NumericField b_fields[NUM_POSE_VALUES],
    NumericField c_fields[NUM_POSE_VALUES],
    NumericField *speed,
    NumericField *accel,
    NumericField *jerk
)
{
    const double A[NUM_POSE_VALUES] =
    {
        -0.421962,
        -0.451956,
         0.234228,
       104.80,
       -68.51,
        38.48
    };

    double B[NUM_POSE_VALUES] =
    {
        0.0, 0.0, 0.0,
        A[3], A[4], A[5]
    };

    double C[NUM_POSE_VALUES] =
    {
        0.0, 0.0, 0.0,
        A[3], A[4], A[5]
    };

    if (path == PATH_LINE)
    {
        const double line_b[NUM_POSE_VALUES] =
        {
             0.078038,
            -0.451956,
             0.234228,
           147.58,
           -58.31,
            12.75
        };

        memcpy(B, line_b, sizeof(B));
        memcpy(C, line_b, sizeof(C));
    }
    else if (path == PATH_ARC)
    {
        B[0] = -0.221962;
        B[1] = -0.311956;
        B[2] =  0.234228;

        C[0] = -0.021962;
        C[1] = -0.451956;
        C[2] =  0.234228;
    }
    else
    {
        B[0] = -0.271962;
        B[1] = -0.301956;
        B[2] =  0.234228;

        C[0] = -0.121962;
        C[1] = -0.451956;
        C[2] =  0.234228;
    }

    set_pose(a_fields, A);
    set_pose(b_fields, B);
    set_pose(c_fields, C);

    numeric_field_set(speed, 0.10, 3);
    numeric_field_set(accel, 0.25, 3);
    numeric_field_set(jerk, 1.00, 3);
}


/* ============================================================================
 * PATH PREVIEW
 * ============================================================================
 */

static Vector2 quadratic_bezier(
    Vector2 p0,
    Vector2 p1,
    Vector2 p2,
    float t
)
{
    float u =
        1.0f - t;

    return
        (Vector2)
        {
            u*u*p0.x + 2.0f*u*t*p1.x + t*t*p2.x,
            u*u*p0.y + 2.0f*u*t*p1.y + t*t*p2.y
        };
}


static void draw_path_preview(
    Rectangle bounds,
    PathType path
)
{
    panel(bounds, C_PANEL);

    text_draw(
        "PATH GEOMETRY",
        bounds.x + 18.0f,
        bounds.y + 16.0f,
        13.0f,
        C_MUTED,
        true
    );

    text_draw(
        path_name(path),
        bounds.x + 18.0f,
        bounds.y + 38.0f,
        18.0f,
        C_TEXT,
        true
    );

    Rectangle plot =
    {
        bounds.x + 18.0f,
        bounds.y + 76.0f,
        bounds.width - 36.0f,
        bounds.height - 96.0f
    };

    DrawRectangleRounded(
        plot,
        0.08f,
        8,
        (Color){17, 21, 28, 255}
    );

    Vector2 a =
    {
        plot.x + 34.0f,
        plot.y + plot.height * 0.68f
    };

    Vector2 b =
    {
        plot.x + plot.width * 0.50f,
        plot.y + plot.height * 0.27f
    };

    Vector2 c =
    {
        plot.x + plot.width - 34.0f,
        plot.y + plot.height * 0.68f
    };

    if (path == PATH_LINE)
    {
        DrawLineEx(a, c, 3.0f, C_ACCENT);
    }
    else if (path == PATH_ARC)
    {
        Vector2 previous = a;

        for (int i = 1; i <= 36; i++)
        {
            float t =
                (float)i / 36.0f;

            Vector2 current =
                quadratic_bezier(
                    a,
                    b,
                    c,
                    t
                );

            DrawLineEx(
                previous,
                current,
                3.0f,
                C_ACCENT
            );

            previous = current;
        }
    }
    else
    {
        Vector2 center =
        {
            plot.x + plot.width * 0.50f,
            plot.y + plot.height * 0.52f
        };

        float radius =
            fminf(
                plot.width,
                plot.height
            ) * 0.30f;

        DrawCircleLinesV(
            center,
            radius,
            C_ACCENT
        );

        a =
            (Vector2)
            {
                center.x - radius,
                center.y
            };

        b =
            (Vector2)
            {
                center.x,
                center.y - radius
            };

        c =
            (Vector2)
            {
                center.x + radius,
                center.y
            };
    }

    DrawCircleV(a, 7.0f, C_GOOD);

    text_draw(
        "A",
        a.x - 4.0f,
        a.y + 11.0f,
        12.0f,
        C_MUTED,
        true
    );

    if (path != PATH_LINE)
    {
        DrawCircleV(b, 6.0f, C_WARN);

        text_draw(
            "B",
            b.x - 4.0f,
            b.y - 25.0f,
            12.0f,
            C_MUTED,
            true
        );
    }

    DrawCircleV(c, 7.0f, C_BAD);

    text_draw(
        path == PATH_FULL_CIRCLE ? "C" : "END",
        c.x - 9.0f,
        c.y + 11.0f,
        12.0f,
        C_MUTED,
        true
    );
}


/* ============================================================================
 * WAYPOINT CARD
 * ============================================================================
 */

static void draw_pose_card(
    Rectangle bounds,
    const char *badge,
    const char *title,
    const char *subtitle,
    NumericField fields[NUM_POSE_VALUES],
    bool position_enabled,
    bool orientation_enabled
)
{
    panel(bounds, C_PANEL);

    DrawRectangleRounded(
        (Rectangle)
        {
            bounds.x + 17.0f,
            bounds.y + 16.0f,
            31.0f,
            24.0f
        },
        0.25f,
        8,
        (Color){31, 55, 82, 255}
    );

    text_draw(
        badge,
        bounds.x + 27.0f,
        bounds.y + 20.0f,
        13.0f,
        C_ACCENT,
        true
    );

    text_draw(
        title,
        bounds.x + 59.0f,
        bounds.y + 16.0f,
        17.5f,
        C_TEXT,
        true
    );

    text_draw(
        subtitle,
        bounds.x + 18.0f,
        bounds.y + 48.0f,
        12.5f,
        C_MUTED,
        false
    );

    const char *labels[NUM_POSE_VALUES] =
    {
        "X  m",
        "Y  m",
        "Z  m",
        "Yaw  deg",
        "Pitch  deg",
        "Roll  deg"
    };

    for (int i = 0; i < NUM_POSE_VALUES; i++)
    {
        int column =
            i < 3
            ? 0
            : 1;

        int row =
            i < 3
            ? i
            : i - 3;

        float column_x =
            bounds.x +
            18.0f +
            column * 150.0f;

        float row_y =
            bounds.y +
            86.0f +
            row * 69.0f;

        bool enabled =
            column == 0
            ? position_enabled
            : orientation_enabled;

        text_draw(
            labels[i],
            column_x,
            row_y,
            11.5f,
            enabled ? C_MUTED : C_FAINT,
            false
        );

        Rectangle field_bounds =
        {
            column_x,
            row_y + 19.0f,
            135.0f,
            37.0f
        };

        update_numeric_field(
            &fields[i],
            field_bounds,
            enabled
        );

        draw_numeric_field(
            &fields[i],
            field_bounds,
            enabled
        );
    }
}


/* ============================================================================
 * MAIN
 * ============================================================================
 */

int main(void)
{
    int command_socket =
        socket(
            AF_INET,
            SOCK_DGRAM,
            0
        );

    if (command_socket < 0)
    {
        perror("socket");
        return 1;
    }

    struct sockaddr_in controller_address;

    memset(
        &controller_address,
        0,
        sizeof(controller_address)
    );

    controller_address.sin_family = AF_INET;
    controller_address.sin_port = htons(CONTROLLER_PORT);

    if (
        inet_pton(
            AF_INET,
            CONTROLLER_IP,
            &controller_address.sin_addr
        ) != 1
    )
    {
        fprintf(stderr, "Invalid controller IP\n");
        close(command_socket);
        return 1;
    }

    int status_socket =
        socket(
            AF_INET,
            SOCK_DGRAM,
            0
        );

    if (status_socket < 0)
    {
        perror("status socket");
        close(command_socket);
        return 1;
    }

    struct sockaddr_in status_address;

    memset(
        &status_address,
        0,
        sizeof(status_address)
    );

    status_address.sin_family = AF_INET;
    status_address.sin_port = htons(STATUS_PORT);
    status_address.sin_addr.s_addr = htonl(INADDR_ANY);

    if (
        bind(
            status_socket,
            (struct sockaddr *)&status_address,
            sizeof(status_address)
        ) < 0
    )
    {
        perror("status bind");
        close(status_socket);
        close(command_socket);
        return 1;
    }

    SetConfigFlags(
        FLAG_WINDOW_RESIZABLE |
        FLAG_MSAA_4X_HINT
    );

    InitWindow(
        1440,
        900,
        "Robot Motion Console"
    );

    SetWindowMinSize(
        960,
        600
    );

    SetTargetFPS(60);

    load_ui_fonts();

    NumericField waypoint_a[NUM_POSE_VALUES] = {0};
    NumericField waypoint_b[NUM_POSE_VALUES] = {0};
    NumericField waypoint_c[NUM_POSE_VALUES] = {0};

    NumericField speed_field = {0};
    NumericField accel_field = {0};
    NumericField jerk_field = {0};

    PathType selected_path =
        PATH_LINE;

    load_reference_case(
        selected_path,
        waypoint_a,
        waypoint_b,
        waypoint_c,
        &speed_field,
        &accel_field,
        &jerk_field
    );

    ControllerStatus controller_status;

    memset(
        &controller_status,
        0,
        sizeof(controller_status)
    );

    uint32_t sequence = 0;

    char status_text[320];

    snprintf(
        status_text,
        sizeof(status_text),
        "Ready. Straight-line live execution is connected to the controller."
    );

    while (!WindowShouldClose())
    {
        receive_controller_status(
            status_socket,
            &controller_status
        );

        int screen_width =
            GetScreenWidth();

        int screen_height =
            GetScreenHeight();

        float scale_x =
            (float)screen_width /
            UI_WIDTH;

        float scale_y =
            (float)screen_height /
            UI_HEIGHT;

        ui_scale =
            fminf(
                scale_x,
                scale_y
            );

        if (ui_scale <= 0.0f)
        {
            ui_scale = 1.0f;
        }

        ui_origin =
            (Vector2)
            {
                ((float)screen_width - UI_WIDTH * ui_scale) * 0.5f,
                ((float)screen_height - UI_HEIGHT * ui_scale) * 0.5f
            };

        Camera2D camera =
        {
            .offset = ui_origin,
            .target = {0.0f, 0.0f},
            .rotation = 0.0f,
            .zoom = ui_scale
        };

        BeginDrawing();

        ClearBackground(C_BG);

        BeginMode2D(camera);

        /* ====================================================================
         * HEADER
         * ====================================================================
         */

        text_draw(
            "Robot Motion Console",
            32.0f,
            23.0f,
            29.0f,
            C_TEXT,
            true
        );

        text_draw(
            "Cartesian planner  /  EtherCAT CSP  /  ADLS IK",
            33.0f,
            60.0f,
            14.0f,
            C_MUTED,
            false
        );

        bool status_online =
            controller_status.valid &&
            (GetTime() - controller_status.lastReceiveTime) < 1.0;

        Rectangle header_status =
        {
            1110.0f,
            26.0f,
            296.0f,
            54.0f
        };

        DrawRectangleRounded(
            header_status,
            0.22f,
            10,
            C_PANEL
        );

        status_dot(
            1131.0f,
            53.0f,
            status_online,
            C_GOOD
        );

        text_draw(
            status_online ? "CONTROLLER ONLINE" : "CONTROLLER OFFLINE",
            1146.0f,
            37.0f,
            13.0f,
            status_online ? C_GOOD : C_BAD,
            true
        );

        text_draw(
            status_online
                ? motion_state_name_hmi(controller_status.motionState)
                : "Waiting for status packets",
            1146.0f,
            56.0f,
            11.5f,
            C_MUTED,
            false
        );

        DrawLine(
            32,
            94,
            1408,
            94,
            C_BORDER
        );

        /* ====================================================================
         * PATH GEOMETRY SELECTOR
         * ====================================================================
         */

        text_draw(
            "PATH GEOMETRY",
            32.0f,
            112.0f,
            12.5f,
            C_MUTED,
            true
        );

        Rectangle line_tab =
        {
            32.0f,
            139.0f,
            214.0f,
            60.0f
        };

        Rectangle arc_tab =
        {
            256.0f,
            139.0f,
            214.0f,
            60.0f
        };

        Rectangle circle_tab =
        {
            480.0f,
            139.0f,
            214.0f,
            60.0f
        };

        PathType previous_path =
            selected_path;

        if (
            path_tab(
                line_tab,
                "Straight line",
                "A  ->  B",
                selected_path == PATH_LINE
            )
        )
        {
            selected_path = PATH_LINE;
        }

        if (
            path_tab(
                arc_tab,
                "Circular arc",
                "A  ->  via B  ->  C",
                selected_path == PATH_ARC
            )
        )
        {
            selected_path = PATH_ARC;
        }

        if (
            path_tab(
                circle_tab,
                "Full circle",
                "A + B + C define circle",
                selected_path == PATH_FULL_CIRCLE
            )
        )
        {
            selected_path = PATH_FULL_CIRCLE;
        }

        if (selected_path != previous_path)
        {
            load_reference_case(
                selected_path,
                waypoint_a,
                waypoint_b,
                waypoint_c,
                &speed_field,
                &accel_field,
                &jerk_field
            );

            if (selected_path == PATH_LINE)
            {
                snprintf(
                    status_text,
                    sizeof(status_text),
                    "Straight line selected. Live controller execution is available."
                );
            }
            else
            {
                snprintf(
                    status_text,
                    sizeof(status_text),
                    "%s selected. ControlCore geometry exists; live circular streaming bridge is still pending.",
                    path_name(selected_path)
                );
            }
        }

        draw_path_preview(
            (Rectangle)
            {
                714.0f,
                112.0f,
                374.0f,
                87.0f
            },
            selected_path
        );

        /* ====================================================================
         * WAYPOINTS
         * ====================================================================
         */

        const float cards_y =
            219.0f;

        const float card_h =
            319.0f;

        draw_pose_card(
            (Rectangle){32.0f, cards_y, 334.0f, card_h},
            "A",
            "Start pose",
            "TCP position + start orientation",
            waypoint_a,
            true,
            true
        );

        if (selected_path == PATH_LINE)
        {
            draw_pose_card(
                (Rectangle){377.0f, cards_y, 334.0f, card_h},
                "B",
                "End pose",
                "TCP position + final orientation",
                waypoint_b,
                true,
                true
            );

            draw_pose_card(
                (Rectangle){722.0f, cards_y, 334.0f, card_h},
                "C",
                "Not used",
                "Straight line only needs A and B",
                waypoint_c,
                false,
                false
            );
        }
        else if (selected_path == PATH_ARC)
        {
            draw_pose_card(
                (Rectangle){377.0f, cards_y, 334.0f, card_h},
                "B",
                "Via point",
                "Position forces the arc through B",
                waypoint_b,
                true,
                false
            );

            draw_pose_card(
                (Rectangle){722.0f, cards_y, 334.0f, card_h},
                "C",
                "End pose",
                "Arc endpoint + final orientation",
                waypoint_c,
                true,
                true
            );
        }
        else
        {
            draw_pose_card(
                (Rectangle){377.0f, cards_y, 334.0f, card_h},
                "B",
                "Circle point 2",
                "Position defines the circle plane",
                waypoint_b,
                true,
                false
            );

            draw_pose_card(
                (Rectangle){722.0f, cards_y, 334.0f, card_h},
                "C",
                "Circle point 3",
                "Position defines circle; YPR is final",
                waypoint_c,
                true,
                true
            );
        }

        /* ====================================================================
         * MOTION PROFILE
         * ====================================================================
         */

        Rectangle profile_panel =
        {
            32.0f,
            554.0f,
            1024.0f,
            136.0f
        };

        panel(
            profile_panel,
            C_PANEL
        );

        text_draw(
            "MOTION PROFILE",
            51.0f,
            571.0f,
            13.0f,
            C_MUTED,
            true
        );

        text_draw(
            "S-curve limits applied in Cartesian path space",
            51.0f,
            592.0f,
            12.0f,
            C_FAINT,
            false
        );

        const char *profile_labels[3] =
        {
            "TCP speed   m/s",
            "TCP acceleration   m/s^2",
            "TCP jerk   m/s^3"
        };

        NumericField *profile_fields[3] =
        {
            &speed_field,
            &accel_field,
            &jerk_field
        };

        for (int i = 0; i < 3; i++)
        {
            float x =
                51.0f +
                i * 316.0f;

            text_draw(
                profile_labels[i],
                x,
                622.0f,
                11.5f,
                C_MUTED,
                false
            );

            Rectangle field_bounds =
            {
                x,
                643.0f,
                270.0f,
                35.0f
            };

            update_numeric_field(
                profile_fields[i],
                field_bounds,
                true
            );

            draw_numeric_field(
                profile_fields[i],
                field_bounds,
                true
            );
        }

        /* ====================================================================
         * ACTION BAR
         * ====================================================================
         */

        Rectangle action_panel =
        {
            32.0f,
            706.0f,
            1024.0f,
            91.0f
        };

        panel(
            action_panel,
            C_PANEL
        );

        Rectangle reference_button =
        {
            51.0f,
            727.0f,
            174.0f,
            48.0f
        };

        Rectangle run_button =
        {
            239.0f,
            727.0f,
            360.0f,
            48.0f
        };

        Rectangle stop_button =
        {
            613.0f,
            727.0f,
            174.0f,
            48.0f
        };

        if (
            button_ex(
                reference_button,
                "LOAD REFERENCE",
                false,
                false,
                true
            )
        )
        {
            load_reference_case(
                selected_path,
                waypoint_a,
                waypoint_b,
                waypoint_c,
                &speed_field,
                &accel_field,
                &jerk_field
            );

            snprintf(
                status_text,
                sizeof(status_text),
                "Loaded %s reference values.",
                path_name(selected_path)
            );
        }

        bool live_run_available =
            selected_path == PATH_LINE;

        const char *run_label =
            live_run_available
            ? "PLAN + RUN STRAIGHT LINE"
            : "LIVE RUN - BRIDGE PENDING";

        if (
            button_ex(
                run_button,
                run_label,
                true,
                false,
                live_run_available
            )
        )
        {
            float A[NUM_POSE_VALUES];
            float B[NUM_POSE_VALUES];

            bool values_valid =
                true;

            for (int i = 0; i < NUM_POSE_VALUES; i++)
            {
                double a_value = 0.0;
                double b_value = 0.0;

                if (
                    !numeric_field_parse(&waypoint_a[i], &a_value) ||
                    !numeric_field_parse(&waypoint_b[i], &b_value)
                )
                {
                    values_valid = false;
                }

                A[i] = (float)a_value;
                B[i] = (float)b_value;
            }

            double speed_value = 0.0;
            double accel_value = 0.0;
            double jerk_value = 0.0;

            if (
                !numeric_field_parse(&speed_field, &speed_value) ||
                !numeric_field_parse(&accel_field, &accel_value) ||
                !numeric_field_parse(&jerk_field, &jerk_value) ||
                speed_value <= 0.0 ||
                accel_value <= 0.0 ||
                jerk_value <= 0.0
            )
            {
                values_valid = false;
            }

            if (!values_valid)
            {
                snprintf(
                    status_text,
                    sizeof(status_text),
                    "Input error: all A/B values must be finite and speed/accel/jerk must be > 0."
                );
            }
            else
            {
                sequence++;

                int success =
                    send_line_plan_command(
                        command_socket,
                        &controller_address,
                        sequence,
                        A,
                        B,
                        (float)speed_value,
                        (float)accel_value,
                        (float)jerk_value
                    );

                if (success)
                {
                    snprintf(
                        status_text,
                        sizeof(status_text),
                        "PLAN + RUN #%u sent: straight line A -> B.",
                        sequence
                    );

                    printf(
                        "\nPLAN + RUN #%u | STRAIGHT LINE\n"
                        "A: p=[%.6f %.6f %.6f] m | YPR=[%.2f %.2f %.2f] deg\n"
                        "B: p=[%.6f %.6f %.6f] m | YPR=[%.2f %.2f %.2f] deg\n"
                        "Profile: speed=%.4f m/s accel=%.4f m/s^2 jerk=%.4f m/s^3\n",
                        sequence,
                        A[0], A[1], A[2],
                        A[3], A[4], A[5],
                        B[0], B[1], B[2],
                        B[3], B[4], B[5],
                        speed_value,
                        accel_value,
                        jerk_value
                    );

                    fflush(stdout);
                }
                else
                {
                    snprintf(
                        status_text,
                        sizeof(status_text),
                        "UDP send failed."
                    );
                }
            }
        }

        if (
            button_ex(
                stop_button,
                "STOP / HOLD",
                false,
                true,
                true
            )
        )
        {
            sequence++;

            int success =
                send_stop_command(
                    command_socket,
                    &controller_address,
                    sequence
                );

            if (success)
            {
                snprintf(
                    status_text,
                    sizeof(status_text),
                    "STOP command #%u sent - hold current position.",
                    sequence
                );
            }
            else
            {
                snprintf(
                    status_text,
                    sizeof(status_text),
                    "STOP UDP send failed."
                );
            }
        }

        if (!live_run_available)
        {
            text_draw(
                "Arc / circle can be configured here now; controller execution stays disabled until circular streaming is connected.",
                51.0f,
                782.0f,
                10.5f,
                C_WARN,
                false
            );
        }

        /* ====================================================================
         * ROBOT / ETHERCAT SIDEBAR
         * ====================================================================
         */

        Rectangle state_panel =
        {
            1080.0f,
            112.0f,
            328.0f,
            685.0f
        };

        panel(
            state_panel,
            C_PANEL
        );

        text_draw(
            "ROBOT STATE",
            1102.0f,
            131.0f,
            13.0f,
            C_MUTED,
            true
        );

        text_draw(
            status_online
                ? motion_state_name_hmi(controller_status.motionState)
                : "OFFLINE",
            1102.0f,
            155.0f,
            24.0f,
            status_online ? C_TEXT : C_BAD,
            true
        );

        char line[128];

        snprintf(
            line,
            sizeof(line),
            "Sequence  #%u",
            status_online
                ? controller_status.lastSequence
                : 0U
        );

        text_draw(
            line,
            1102.0f,
            191.0f,
            12.0f,
            C_MUTED,
            false
        );

        uint32_t display_index =
            controller_status.trajectoryCount > 0
            ? controller_status.trajectoryIndex + 1
            : 0;

        snprintf(
            line,
            sizeof(line),
            "Trajectory  %u / %u",
            display_index,
            controller_status.trajectoryCount
        );

        text_draw(
            line,
            1102.0f,
            212.0f,
            12.0f,
            C_MUTED,
            false
        );

        float progress =
            controller_status.trajectoryCount > 0
            ? (float)display_index /
              (float)controller_status.trajectoryCount
            : 0.0f;

        if (progress > 1.0f)
        {
            progress = 1.0f;
        }

        Rectangle progress_track =
        {
            1102.0f,
            238.0f,
            284.0f,
            8.0f
        };

        DrawRectangleRounded(
            progress_track,
            1.0f,
            8,
            (Color){37, 44, 56, 255}
        );

        if (progress > 0.0f)
        {
            Rectangle progress_fill =
            {
                progress_track.x,
                progress_track.y,
                progress_track.width * progress,
                progress_track.height
            };

            DrawRectangleRounded(
                progress_fill,
                1.0f,
                8,
                C_ACCENT
            );
        }

        DrawLine(
            1102,
            267,
            1386,
            267,
            C_BORDER
        );

        text_draw(
            "ETHERCAT",
            1102.0f,
            287.0f,
            12.5f,
            C_MUTED,
            true
        );

        bool wkc_good =
            status_online &&
            controller_status.expectedWkc > 0 &&
            controller_status.wkc >= controller_status.expectedWkc;

        snprintf(
            line,
            sizeof(line),
            "WKC  %u / %u",
            controller_status.wkc,
            controller_status.expectedWkc
        );

        status_dot(
            1110.0f,
            327.0f,
            wkc_good,
            C_GOOD
        );

        text_draw(
            line,
            1123.0f,
            317.0f,
            14.0f,
            wkc_good ? C_GOOD : C_MUTED,
            true
        );

        text_draw(
            "CiA-402 servo states",
            1102.0f,
            353.0f,
            11.5f,
            C_FAINT,
            false
        );

        for (int joint = 0; joint < 6; joint++)
        {
            float y =
                386.0f +
                joint * 48.0f;

            uint16_t statusword =
                controller_status.statusword[joint];

            bool enabled =
                status_online &&
                cia402_operation_enabled(statusword);

            status_dot(
                1110.0f,
                y + 11.0f,
                enabled,
                C_GOOD
            );

            snprintf(
                line,
                sizeof(line),
                "J%d",
                joint + 1
            );

            text_draw(
                line,
                1124.0f,
                y,
                13.0f,
                C_TEXT,
                true
            );

            text_draw(
                status_online
                    ? cia402_state_name(statusword)
                    : "---",
                1162.0f,
                y,
                11.5f,
                enabled ? C_GOOD : C_MUTED,
                false
            );

            snprintf(
                line,
                sizeof(line),
                "0x%04X",
                statusword
            );

            text_draw(
                line,
                1318.0f,
                y,
                10.5f,
                C_FAINT,
                false
            );
        }

        DrawLine(
            1102,
            686,
            1386,
            686,
            C_BORDER
        );

        text_draw(
            "Selected path",
            1102.0f,
            705.0f,
            11.5f,
            C_FAINT,
            false
        );

        text_draw(
            path_name(selected_path),
            1102.0f,
            727.0f,
            16.0f,
            C_TEXT,
            true
        );

        text_draw(
            selected_path == PATH_LINE
                ? "LIVE EXECUTION READY"
                : "CONFIG ONLY - BRIDGE PENDING",
            1102.0f,
            753.0f,
            10.5f,
            selected_path == PATH_LINE
                ? C_GOOD
                : C_WARN,
            true
        );

        /* ====================================================================
         * FOOTER STATUS
         * ====================================================================
         */

        Rectangle footer =
        {
            32.0f,
            817.0f,
            1376.0f,
            52.0f
        };

        DrawRectangleRounded(
            footer,
            0.12f,
            10,
            (Color){20, 25, 33, 255}
        );

        status_dot(
            51.0f,
            843.0f,
            true,
            selected_path == PATH_LINE
                ? C_ACCENT
                : C_WARN
        );

        text_draw(
            status_text,
            67.0f,
            832.0f,
            13.0f,
            C_MUTED,
            false
        );

        text_draw(
            "Click a value and type to replace  |  Enter commits",
            1069.0f,
            834.0f,
            10.5f,
            C_FAINT,
            false
        );

        EndMode2D();
        EndDrawing();
    }

    unload_ui_fonts();

    close(status_socket);
    close(command_socket);

    CloseWindow();

    return 0;
}
