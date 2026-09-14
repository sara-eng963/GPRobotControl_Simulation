/*
 * ============================================================================
 * ROBOT MOTION CONSOLE - CARTESIAN HMI
 * ============================================================================
 *
 * Live paths:
 *   - straight line A -> B
 *   - circular arc A -> via B -> C
 *   - full circle defined by A, B, C
 *
 * The interface renders at a fixed 118% scale. Resizing never shrinks it;
 * horizontal/vertical scrollbars expose the hidden area instead.
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

#define CONTROLLER_IP   "127.0.0.1"
#define CONTROLLER_PORT 5006
#define STATUS_PORT     5007

#define NUM_POSE_VALUES 6

#define PACKET_MAGIC        0x52425432U /* RBT2 */
#define STATUS_PACKET_MAGIC 0x53544132U /* STA2 */
#define STATUS_WORD_COUNT   13U

#define LINE_PLAN_WORD_COUNT     18U
#define CIRCULAR_PLAN_WORD_COUNT 24U

typedef enum
{
    COMMAND_PLAN_AND_RUN_LINE        = 1,
    COMMAND_STOP                     = 2,
    COMMAND_PLAN_AND_RUN_ARC         = 3,
    COMMAND_PLAN_AND_RUN_FULL_CIRCLE = 4
} CommandType;

typedef enum
{
    PATH_LINE = 0,
    PATH_ARC,
    PATH_FULL_CIRCLE
} PathType;

/* ============================================================================
 * FIXED-SCALE VIEWPORT / SCROLLING
 * ============================================================================
 */

#define UI_WIDTH              1280.0f
#define UI_HEIGHT              800.0f
#define UI_SCALE                 1.18f
#define UI_SCROLL_STEP          62.0f
#define UI_SCROLLBAR_MARGIN      6.0f
#define UI_SCROLLBAR_SIZE        9.0f
#define UI_SCROLLBAR_MIN_THUMB  48.0f

#define UI_CONTENT_WIDTH  (UI_WIDTH * UI_SCALE)
#define UI_CONTENT_HEIGHT (UI_HEIGHT * UI_SCALE)

static float ui_scroll_x = 0.0f;
static float ui_scroll_y = 0.0f;
static bool ui_drag_x = false;
static bool ui_drag_y = false;
static float ui_drag_offset_x = 0.0f;
static float ui_drag_offset_y = 0.0f;
static bool ui_scrollbar_consumes_pointer = false;

static float clampf_local(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static Vector2 ui_mouse_position(void)
{
    Vector2 mouse = GetMousePosition();

    return (Vector2)
    {
        (mouse.x + ui_scroll_x) / UI_SCALE,
        (mouse.y + ui_scroll_y) / UI_SCALE
    };
}

static Rectangle horizontal_scroll_track(void)
{
    float screen_width = (float)GetScreenWidth();
    float screen_height = (float)GetScreenHeight();
    bool vertical_visible = UI_CONTENT_HEIGHT > screen_height;

    float width =
        screen_width -
        2.0f * UI_SCROLLBAR_MARGIN -
        (vertical_visible ? UI_SCROLLBAR_SIZE + UI_SCROLLBAR_MARGIN : 0.0f);

    if (width < 40.0f) width = 40.0f;

    return (Rectangle)
    {
        UI_SCROLLBAR_MARGIN,
        screen_height - UI_SCROLLBAR_MARGIN - UI_SCROLLBAR_SIZE,
        width,
        UI_SCROLLBAR_SIZE
    };
}

static Rectangle vertical_scroll_track(void)
{
    float screen_width = (float)GetScreenWidth();
    float screen_height = (float)GetScreenHeight();
    bool horizontal_visible = UI_CONTENT_WIDTH > screen_width;

    float height =
        screen_height -
        2.0f * UI_SCROLLBAR_MARGIN -
        (horizontal_visible ? UI_SCROLLBAR_SIZE + UI_SCROLLBAR_MARGIN : 0.0f);

    if (height < 40.0f) height = 40.0f;

    return (Rectangle)
    {
        screen_width - UI_SCROLLBAR_MARGIN - UI_SCROLLBAR_SIZE,
        UI_SCROLLBAR_MARGIN,
        UI_SCROLLBAR_SIZE,
        height
    };
}

static Rectangle horizontal_scroll_thumb(Rectangle track, float max_scroll)
{
    float viewport = (float)GetScreenWidth();
    float width = track.width * (viewport / UI_CONTENT_WIDTH);

    width = clampf_local(width, UI_SCROLLBAR_MIN_THUMB, track.width);

    float usable = track.width - width;
    float x = track.x;

    if (max_scroll > 0.0f && usable > 0.0f)
    {
        x += (ui_scroll_x / max_scroll) * usable;
    }

    return (Rectangle){x, track.y, width, track.height};
}

static Rectangle vertical_scroll_thumb(Rectangle track, float max_scroll)
{
    float viewport = (float)GetScreenHeight();
    float height = track.height * (viewport / UI_CONTENT_HEIGHT);

    height = clampf_local(height, UI_SCROLLBAR_MIN_THUMB, track.height);

    float usable = track.height - height;
    float y = track.y;

    if (max_scroll > 0.0f && usable > 0.0f)
    {
        y += (ui_scroll_y / max_scroll) * usable;
    }

    return (Rectangle){track.x, y, track.width, height};
}

static void update_scrollbars(void)
{
    float viewport_width = (float)GetScreenWidth();
    float viewport_height = (float)GetScreenHeight();

    float max_scroll_x = UI_CONTENT_WIDTH - viewport_width;
    float max_scroll_y = UI_CONTENT_HEIGHT - viewport_height;

    if (max_scroll_x < 0.0f) max_scroll_x = 0.0f;
    if (max_scroll_y < 0.0f) max_scroll_y = 0.0f;

    if (max_scroll_x <= 0.0f)
    {
        ui_scroll_x = 0.0f;
        ui_drag_x = false;
    }

    if (max_scroll_y <= 0.0f)
    {
        ui_scroll_y = 0.0f;
        ui_drag_y = false;
    }

    float wheel = GetMouseWheelMove();

    if (wheel != 0.0f)
    {
        bool shift =
            IsKeyDown(KEY_LEFT_SHIFT) ||
            IsKeyDown(KEY_RIGHT_SHIFT);

        if (shift || max_scroll_y <= 0.0f)
            ui_scroll_x -= wheel * UI_SCROLL_STEP;
        else
            ui_scroll_y -= wheel * UI_SCROLL_STEP;
    }

    ui_scroll_x = clampf_local(ui_scroll_x, 0.0f, max_scroll_x);
    ui_scroll_y = clampf_local(ui_scroll_y, 0.0f, max_scroll_y);

    Vector2 mouse = GetMousePosition();
    bool horizontal_visible = max_scroll_x > 0.0f;
    bool vertical_visible = max_scroll_y > 0.0f;

    Rectangle h_track = {0};
    Rectangle h_thumb = {0};
    Rectangle v_track = {0};
    Rectangle v_thumb = {0};

    if (horizontal_visible)
    {
        h_track = horizontal_scroll_track();
        h_thumb = horizontal_scroll_thumb(h_track, max_scroll_x);
    }

    if (vertical_visible)
    {
        v_track = vertical_scroll_track();
        v_thumb = vertical_scroll_thumb(v_track, max_scroll_y);
    }

    ui_scrollbar_consumes_pointer =
        ui_drag_x ||
        ui_drag_y ||
        (horizontal_visible && CheckCollisionPointRec(mouse, h_track)) ||
        (vertical_visible && CheckCollisionPointRec(mouse, v_track));

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
        if (horizontal_visible && CheckCollisionPointRec(mouse, h_track))
        {
            float usable = h_track.width - h_thumb.width;

            if (CheckCollisionPointRec(mouse, h_thumb))
            {
                ui_drag_offset_x = mouse.x - h_thumb.x;
            }
            else
            {
                float new_x =
                    clampf_local(
                        mouse.x - h_thumb.width * 0.5f,
                        h_track.x,
                        h_track.x + usable
                    );

                if (usable > 0.0f)
                    ui_scroll_x = ((new_x - h_track.x) / usable) * max_scroll_x;

                ui_drag_offset_x = h_thumb.width * 0.5f;
            }

            ui_drag_x = true;
            ui_drag_y = false;
            ui_scrollbar_consumes_pointer = true;
        }
        else if (vertical_visible && CheckCollisionPointRec(mouse, v_track))
        {
            float usable = v_track.height - v_thumb.height;

            if (CheckCollisionPointRec(mouse, v_thumb))
            {
                ui_drag_offset_y = mouse.y - v_thumb.y;
            }
            else
            {
                float new_y =
                    clampf_local(
                        mouse.y - v_thumb.height * 0.5f,
                        v_track.y,
                        v_track.y + usable
                    );

                if (usable > 0.0f)
                    ui_scroll_y = ((new_y - v_track.y) / usable) * max_scroll_y;

                ui_drag_offset_y = v_thumb.height * 0.5f;
            }

            ui_drag_y = true;
            ui_drag_x = false;
            ui_scrollbar_consumes_pointer = true;
        }
    }

    if (ui_drag_x)
    {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            float usable = h_track.width - h_thumb.width;
            float new_x =
                clampf_local(
                    mouse.x - ui_drag_offset_x,
                    h_track.x,
                    h_track.x + usable
                );

            if (usable > 0.0f)
                ui_scroll_x = ((new_x - h_track.x) / usable) * max_scroll_x;
        }
        else
        {
            ui_drag_x = false;
        }
    }

    if (ui_drag_y)
    {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            float usable = v_track.height - v_thumb.height;
            float new_y =
                clampf_local(
                    mouse.y - ui_drag_offset_y,
                    v_track.y,
                    v_track.y + usable
                );

            if (usable > 0.0f)
                ui_scroll_y = ((new_y - v_track.y) / usable) * max_scroll_y;
        }
        else
        {
            ui_drag_y = false;
        }
    }

    ui_scroll_x = clampf_local(ui_scroll_x, 0.0f, max_scroll_x);
    ui_scroll_y = clampf_local(ui_scroll_y, 0.0f, max_scroll_y);
}

/* ============================================================================
 * THEME / FONT
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
static const Color C_MUTED      = {158, 170, 189, 255};
static const Color C_FAINT      = {112, 125, 146, 255};
static const Color C_ACCENT     = { 88, 166, 255, 255};
static const Color C_ACCENT_2   = { 65, 128, 204, 255};
static const Color C_GOOD       = { 91, 201, 142, 255};
static const Color C_WARN       = {239, 184,  86, 255};
static const Color C_BAD        = {232, 105, 111, 255};
static const Color C_DISABLED   = { 78,  88, 105, 255};

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
    if (owned != NULL) *owned = false;

    for (size_t i = 0; i < count; i++)
    {
        if (FileExists(candidates[i]))
        {
            Font font = LoadFontEx(candidates[i], size, NULL, 0);
            SetTextureFilter(font.texture, TEXTURE_FILTER_BILINEAR);
            if (owned != NULL) *owned = true;
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
            40,
            &ui_font_owned
        );

    ui_font_bold =
        load_first_font(
            bold_candidates,
            sizeof(bold_candidates) / sizeof(bold_candidates[0]),
            42,
            &ui_font_bold_owned
        );
}

static void unload_ui_fonts(void)
{
    if (ui_font_bold_owned) UnloadFont(ui_font_bold);
    if (ui_font_owned) UnloadFont(ui_font);
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
    Font font = bold ? ui_font_bold : ui_font;

    DrawTextEx(
        font,
        text,
        (Vector2){x, y},
        size,
        size * 0.015f,
        color
    );
}

static float text_width(const char *text, float size, bool bold)
{
    Font font = bold ? ui_font_bold : ui_font;
    return MeasureTextEx(font, text, size, size * 0.015f).x;
}

static void panel(Rectangle bounds, Color fill)
{
    DrawRectangleRounded(bounds, 0.045f, 12, fill);
    DrawRectangleLinesEx(bounds, 1.0f, C_BORDER);
}

static void status_dot(float x, float y, bool active, Color active_color)
{
    DrawCircleV(
        (Vector2){x, y},
        5.0f,
        active ? active_color : C_DISABLED
    );
}

static void draw_scrollbars(void)
{
    float max_scroll_x = UI_CONTENT_WIDTH - (float)GetScreenWidth();
    float max_scroll_y = UI_CONTENT_HEIGHT - (float)GetScreenHeight();

    if (max_scroll_x > 0.0f)
    {
        Rectangle track = horizontal_scroll_track();
        Rectangle thumb = horizontal_scroll_thumb(track, max_scroll_x);
        DrawRectangleRounded(track, 1.0f, 8, (Color){35, 41, 52, 245});
        DrawRectangleRounded(
            thumb,
            1.0f,
            8,
            ui_drag_x ? C_ACCENT : (Color){91, 105, 129, 255}
        );
    }

    if (max_scroll_y > 0.0f)
    {
        Rectangle track = vertical_scroll_track();
        Rectangle thumb = vertical_scroll_thumb(track, max_scroll_y);
        DrawRectangleRounded(track, 1.0f, 8, (Color){35, 41, 52, 245});
        DrawRectangleRounded(
            thumb,
            1.0f,
            8,
            ui_drag_y ? C_ACCENT : (Color){91, 105, 129, 255}
        );
    }
}

static bool button_ex(
    Rectangle bounds,
    const char *label,
    bool primary,
    bool danger,
    bool enabled
)
{
    Vector2 mouse = ui_mouse_position();

    bool hovered =
        enabled &&
        !ui_scrollbar_consumes_pointer &&
        CheckCollisionPointRec(mouse, bounds);

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
        fill = hovered ? (Color){129, 51, 58, 255} : (Color){96, 42, 49, 255};
        border = C_BAD;
        foreground = C_TEXT;
    }
    else if (primary)
    {
        fill = hovered ? (Color){77, 151, 236, 255} : C_ACCENT_2;
        border = C_ACCENT;
        foreground = C_TEXT;
    }
    else
    {
        fill = hovered ? (Color){42, 51, 65, 255} : C_PANEL_2;
        border = hovered ? C_BORDER_HI : C_BORDER;
        foreground = C_TEXT;
    }

    DrawRectangleRounded(bounds, 0.14f, 10, fill);
    DrawRectangleLinesEx(bounds, 1.0f, border);

    float font_size = 14.5f;
    float width = text_width(label, font_size, true);

    text_draw(
        label,
        bounds.x + bounds.width * 0.5f - width * 0.5f,
        bounds.y + bounds.height * 0.5f - font_size * 0.58f,
        font_size,
        foreground,
        true
    );

    return hovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

static const char *path_name(PathType type)
{
    switch (type)
    {
        case PATH_LINE:        return "Straight line";
        case PATH_ARC:         return "Circular arc";
        case PATH_FULL_CIRCLE: return "Full circle";
        default:               return "Unknown";
    }
}

static bool path_tab(
    Rectangle bounds,
    const char *title,
    const char *subtitle,
    bool selected
)
{
    Vector2 mouse = ui_mouse_position();
    bool hovered =
        !ui_scrollbar_consumes_pointer &&
        CheckCollisionPointRec(mouse, bounds);

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

    DrawRectangleRounded(bounds, 0.09f, 10, fill);
    DrawRectangleLinesEx(bounds, selected ? 2.0f : 1.0f, border);

    text_draw(
        title,
        bounds.x + 14.0f,
        bounds.y + 9.0f,
        16.0f,
        selected ? C_TEXT : (Color){214, 221, 232, 255},
        true
    );

    text_draw(
        subtitle,
        bounds.x + 14.0f,
        bounds.y + 31.0f,
        12.5f,
        C_MUTED,
        false
    );

    return hovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

/* ============================================================================
 * NUMERIC FIELDS
 * ============================================================================
 */

typedef struct
{
    char text[32];
    bool active;
    bool replace_on_type;
} NumericField;

static void numeric_field_set(NumericField *field, double value, int decimals)
{
    if (field == NULL) return;

    snprintf(field->text, sizeof(field->text), "%.*f", decimals, value);
    field->active = false;
    field->replace_on_type = false;
}

static bool numeric_field_parse(const NumericField *field, double *value)
{
    if (field == NULL || value == NULL) return false;

    char *end = NULL;
    double parsed = strtod(field->text, &end);

    if (end == field->text || *end != '\0' || !isfinite(parsed))
        return false;

    *value = parsed;
    return true;
}

static void update_numeric_field(
    NumericField *field,
    Rectangle bounds,
    bool enabled
)
{
    if (field == NULL) return;

    Vector2 mouse = ui_mouse_position();

    if (
        IsMouseButtonPressed(MOUSE_BUTTON_LEFT) &&
        !ui_scrollbar_consumes_pointer
    )
    {
        bool clicked = enabled && CheckCollisionPointRec(mouse, bounds);

        field->active = clicked;
        field->replace_on_type = clicked;
    }

    if (!enabled || !field->active) return;

    if (
        (IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)) &&
        IsKeyPressed(KEY_A)
    )
    {
        field->replace_on_type = true;
    }

    int key = GetCharPressed();

    while (key > 0)
    {
        bool allowed =
            (key >= '0' && key <= '9') ||
            key == '-' || key == '+' || key == '.' ||
            key == 'e' || key == 'E';

        if (allowed)
        {
            if (field->replace_on_type)
            {
                field->text[0] = '\0';
                field->replace_on_type = false;
            }

            size_t length = strlen(field->text);

            if (length + 1 < sizeof(field->text))
            {
                field->text[length] = (char)key;
                field->text[length + 1] = '\0';
            }
        }

        key = GetCharPressed();
    }

    if (IsKeyPressed(KEY_BACKSPACE) || IsKeyPressed(KEY_DELETE))
    {
        if (field->replace_on_type)
        {
            field->text[0] = '\0';
            field->replace_on_type = false;
        }
        else
        {
            size_t length = strlen(field->text);
            if (length > 0) field->text[length - 1] = '\0';
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

    Color foreground = enabled ? C_TEXT : C_FAINT;

    DrawRectangleRounded(bounds, 0.13f, 8, fill);
    DrawRectangleLinesEx(
        bounds,
        field->active && enabled ? 1.7f : 1.0f,
        border
    );

    if (enabled && field->active && field->replace_on_type)
    {
        float width = text_width(field->text, 16.0f, false);
        DrawRectangleRounded(
            (Rectangle)
            {
                bounds.x + 8.0f,
                bounds.y + 7.0f,
                width + 7.0f,
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
        bounds.y + 8.0f,
        16.0f,
        foreground,
        false
    );
}

/* ============================================================================
 * UDP PROTOCOL
 * ============================================================================
 */

static uint32_t float_to_network_word(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    return htonl(bits);
}

static CommandType command_for_path(PathType path)
{
    switch (path)
    {
        case PATH_LINE:        return COMMAND_PLAN_AND_RUN_LINE;
        case PATH_ARC:         return COMMAND_PLAN_AND_RUN_ARC;
        case PATH_FULL_CIRCLE: return COMMAND_PLAN_AND_RUN_FULL_CIRCLE;
        default:               return COMMAND_PLAN_AND_RUN_LINE;
    }
}

static int send_plan_command(
    int socket_fd,
    const struct sockaddr_in *controller_address,
    uint32_t sequence,
    PathType path,
    const float waypoint_a[NUM_POSE_VALUES],
    const float waypoint_b[NUM_POSE_VALUES],
    const float waypoint_c[NUM_POSE_VALUES],
    float tcp_speed,
    float tcp_accel,
    float tcp_jerk
)
{
    uint32_t packet[CIRCULAR_PLAN_WORD_COUNT];
    int index = 0;

    packet[index++] = htonl(PACKET_MAGIC);
    packet[index++] = htonl((uint32_t)command_for_path(path));
    packet[index++] = htonl(sequence);

    for (int i = 0; i < NUM_POSE_VALUES; i++)
        packet[index++] = float_to_network_word(waypoint_a[i]);

    for (int i = 0; i < NUM_POSE_VALUES; i++)
        packet[index++] = float_to_network_word(waypoint_b[i]);

    if (path != PATH_LINE)
    {
        for (int i = 0; i < NUM_POSE_VALUES; i++)
            packet[index++] = float_to_network_word(waypoint_c[i]);
    }

    packet[index++] = float_to_network_word(tcp_speed);
    packet[index++] = float_to_network_word(tcp_accel);
    packet[index++] = float_to_network_word(tcp_jerk);

    size_t packet_bytes =
        path == PATH_LINE
            ? LINE_PLAN_WORD_COUNT * sizeof(uint32_t)
            : CIRCULAR_PLAN_WORD_COUNT * sizeof(uint32_t);

    ssize_t sent =
        sendto(
            socket_fd,
            packet,
            packet_bytes,
            0,
            (const struct sockaddr *)controller_address,
            sizeof(*controller_address)
        );

    return sent == (ssize_t)packet_bytes;
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

    return sent == (ssize_t)sizeof(packet);
}

/* ============================================================================
 * LIVE CONTROLLER STATUS
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

static const char *motion_state_name_hmi(uint32_t state)
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

static const char *cia402_state_name(uint16_t statusword)
{
    uint16_t state = statusword & 0x006F;

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

static bool cia402_operation_enabled(uint16_t statusword)
{
    return (statusword & 0x006F) == 0x0027;
}

static void receive_controller_status(
    int status_socket,
    ControllerStatus *status
)
{
    if (status_socket < 0 || status == NULL) return;

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
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            perror("status recvfrom");
            break;
        }

        if (received != (ssize_t)sizeof(packet)) continue;
        if (ntohl(packet[0]) != STATUS_PACKET_MAGIC) continue;

        status->motionState = ntohl(packet[1]);
        status->lastSequence = ntohl(packet[2]);
        status->trajectoryIndex = ntohl(packet[3]);
        status->trajectoryCount = ntohl(packet[4]);
        status->wkc = ntohl(packet[5]);
        status->expectedWkc = ntohl(packet[6]);

        for (int joint = 0; joint < 6; joint++)
            status->statusword[joint] = (uint16_t)ntohl(packet[7 + joint]);

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
 * PATH PREVIEW / POSE CARDS
 * ============================================================================
 */

static Vector2 quadratic_bezier(Vector2 a, Vector2 b, Vector2 c, float t)
{
    float u = 1.0f - t;

    return (Vector2)
    {
        u * u * a.x + 2.0f * u * t * b.x + t * t * c.x,
        u * u * a.y + 2.0f * u * t * b.y + t * t * c.y
    };
}

static void draw_path_preview(Rectangle bounds, PathType path)
{
    panel(bounds, C_PANEL);
    text_draw("PATH PREVIEW", bounds.x + 14.0f, bounds.y + 11.0f, 11.5f, C_FAINT, true);
    text_draw(path_name(path), bounds.x + 14.0f, bounds.y + 31.0f, 15.0f, C_TEXT, true);

    Vector2 a = {bounds.x + 155.0f, bounds.y + bounds.height * 0.58f};
    Vector2 c = {bounds.x + bounds.width - 24.0f, bounds.y + bounds.height * 0.58f};

    if (path == PATH_LINE)
    {
        DrawLineEx(a, c, 3.0f, C_ACCENT);
        DrawCircleV(a, 6.0f, C_GOOD);
        DrawCircleV(c, 6.0f, C_BAD);
        text_draw("A", a.x - 4.0f, a.y + 10.0f, 11.0f, C_MUTED, true);
        text_draw("B", c.x - 4.0f, c.y + 10.0f, 11.0f, C_MUTED, true);
    }
    else if (path == PATH_ARC)
    {
        Vector2 b = {(a.x + c.x) * 0.5f, bounds.y + 24.0f};
        Vector2 previous = a;

        for (int i = 1; i <= 24; i++)
        {
            float t = (float)i / 24.0f;
            Vector2 point = quadratic_bezier(a, b, c, t);
            DrawLineEx(previous, point, 3.0f, C_ACCENT);
            previous = point;
        }

        DrawCircleV(a, 6.0f, C_GOOD);
        DrawCircleV(b, 6.0f, C_WARN);
        DrawCircleV(c, 6.0f, C_BAD);
        text_draw("A", a.x - 4.0f, a.y + 10.0f, 11.0f, C_MUTED, true);
        text_draw("B", b.x - 4.0f, b.y - 18.0f, 11.0f, C_MUTED, true);
        text_draw("C", c.x - 4.0f, c.y + 10.0f, 11.0f, C_MUTED, true);
    }
    else
    {
        Vector2 center =
        {
            bounds.x + bounds.width - 63.0f,
            bounds.y + bounds.height * 0.56f
        };

        float radius = 28.0f;
        DrawCircleLines((int)center.x, (int)center.y, radius, C_ACCENT);

        Vector2 p1 = {center.x - radius, center.y};
        Vector2 p2 = {center.x, center.y - radius};
        Vector2 p3 = {center.x + radius, center.y};

        DrawCircleV(p1, 5.0f, C_GOOD);
        DrawCircleV(p2, 5.0f, C_WARN);
        DrawCircleV(p3, 5.0f, C_BAD);
        text_draw("A", p1.x - 4.0f, p1.y + 9.0f, 10.5f, C_MUTED, true);
        text_draw("B", p2.x - 4.0f, p2.y - 16.0f, 10.5f, C_MUTED, true);
        text_draw("C", p3.x - 4.0f, p3.y + 9.0f, 10.5f, C_MUTED, true);
    }
}

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
        (Rectangle){bounds.x + 15.0f, bounds.y + 14.0f, 30.0f, 24.0f},
        0.25f,
        8,
        (Color){31, 55, 82, 255}
    );

    text_draw(badge, bounds.x + 25.0f, bounds.y + 18.0f, 13.0f, C_ACCENT, true);
    text_draw(title, bounds.x + 56.0f, bounds.y + 14.0f, 17.0f, C_TEXT, true);
    text_draw(subtitle, bounds.x + 15.0f, bounds.y + 45.0f, 12.5f, C_MUTED, false);

    const char *labels[NUM_POSE_VALUES] =
    {
        "X   m", "Y   m", "Z   m",
        "Yaw   deg", "Pitch   deg", "Roll   deg"
    };

    float column_gap = 10.0f;
    float inner_width = bounds.width - 30.0f;
    float field_width = (inner_width - column_gap) * 0.5f;

    for (int i = 0; i < NUM_POSE_VALUES; i++)
    {
        int column = i < 3 ? 0 : 1;
        int row = i < 3 ? i : i - 3;

        float x = bounds.x + 15.0f + column * (field_width + column_gap);
        float y = bounds.y + 77.0f + row * 67.0f;
        bool enabled = column == 0 ? position_enabled : orientation_enabled;

        text_draw(labels[i], x, y, 12.5f, enabled ? C_MUTED : C_FAINT, false);

        Rectangle field_bounds = {x, y + 20.0f, field_width, 37.0f};
        update_numeric_field(&fields[i], field_bounds, enabled);
        draw_numeric_field(&fields[i], field_bounds, enabled);
    }
}

/* ============================================================================
 * MAIN UI
 * ============================================================================
 */

int main(void)
{
    int command_socket = socket(AF_INET, SOCK_DGRAM, 0);

    if (command_socket < 0)
    {
        perror("socket");
        return 1;
    }

    struct sockaddr_in controller_address;
    memset(&controller_address, 0, sizeof(controller_address));
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

    int status_socket = socket(AF_INET, SOCK_DGRAM, 0);

    if (status_socket < 0)
    {
        perror("status socket");
        close(command_socket);
        return 1;
    }

    struct sockaddr_in status_address;
    memset(&status_address, 0, sizeof(status_address));
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

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow((int)UI_WIDTH, (int)UI_HEIGHT, "Robot Motion Console");
    SetWindowMinSize(720, 480);
    SetTargetFPS(60);
    load_ui_fonts();

    NumericField waypoint_a[NUM_POSE_VALUES] = {0};
    NumericField waypoint_b[NUM_POSE_VALUES] = {0};
    NumericField waypoint_c[NUM_POSE_VALUES] = {0};
    NumericField speed_field = {0};
    NumericField accel_field = {0};
    NumericField jerk_field = {0};

    PathType selected_path = PATH_LINE;

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
    memset(&controller_status, 0, sizeof(controller_status));

    uint32_t sequence = 0;
    char status_text[320];

    snprintf(
        status_text,
        sizeof(status_text),
        "Ready. Line, circular arc and full-circle live execution are connected."
    );

    while (!WindowShouldClose())
    {
        receive_controller_status(status_socket, &controller_status);
        update_scrollbars();

        bool status_online =
            controller_status.valid &&
            (GetTime() - controller_status.lastReceiveTime) < 1.0;

        Camera2D camera =
        {
            .offset = {-ui_scroll_x, -ui_scroll_y},
            .target = {0.0f, 0.0f},
            .rotation = 0.0f,
            .zoom = UI_SCALE
        };

        BeginDrawing();
        ClearBackground(C_BG);
        BeginMode2D(camera);
        DrawRectangle(0, 0, (int)UI_WIDTH, (int)UI_HEIGHT, C_BG);

        text_draw("Robot Motion Console", 24.0f, 18.0f, 27.0f, C_TEXT, true);
        text_draw(
            "Cartesian planner  /  EtherCAT CSP  /  ADLS IK",
            25.0f,
            53.0f,
            13.5f,
            C_MUTED,
            false
        );

        Rectangle header_status = {1005.0f, 18.0f, 251.0f, 48.0f};
        DrawRectangleRounded(header_status, 0.20f, 10, C_PANEL);
        DrawRectangleLinesEx(header_status, 1.0f, C_BORDER);

        status_dot(1023.0f, 42.0f, status_online, C_GOOD);
        text_draw(
            status_online ? "CONTROLLER ONLINE" : "CONTROLLER OFFLINE",
            1037.0f,
            27.0f,
            12.5f,
            status_online ? C_GOOD : C_BAD,
            true
        );
        text_draw(
            status_online
                ? motion_state_name_hmi(controller_status.motionState)
                : "Waiting for status packets",
            1037.0f,
            46.0f,
            11.5f,
            C_MUTED,
            false
        );

        DrawLine(24, 82, 1256, 82, C_BORDER);

        text_draw("PATH GEOMETRY", 24.0f, 96.0f, 12.5f, C_MUTED, true);

        Rectangle line_tab = {24.0f, 118.0f, 188.0f, 58.0f};
        Rectangle arc_tab = {220.0f, 118.0f, 188.0f, 58.0f};
        Rectangle circle_tab = {416.0f, 118.0f, 188.0f, 58.0f};

        PathType previous_path = selected_path;

        if (path_tab(line_tab, "Straight line", "A  ->  B", selected_path == PATH_LINE))
            selected_path = PATH_LINE;

        if (path_tab(arc_tab, "Circular arc", "A  ->  via B  ->  C", selected_path == PATH_ARC))
            selected_path = PATH_ARC;

        if (path_tab(circle_tab, "Full circle", "A + B + C define circle", selected_path == PATH_FULL_CIRCLE))
            selected_path = PATH_FULL_CIRCLE;

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

            snprintf(
                status_text,
                sizeof(status_text),
                "%s selected. Live controller execution is available.",
                path_name(selected_path)
            );
        }

        draw_path_preview(
            (Rectangle){622.0f, 96.0f, 334.0f, 80.0f},
            selected_path
        );

        const float cards_y = 194.0f;
        const float card_h = 298.0f;

        draw_pose_card(
            (Rectangle){24.0f, cards_y, 300.0f, card_h},
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
                (Rectangle){334.0f, cards_y, 300.0f, card_h},
                "B",
                "End pose",
                "TCP position + final orientation",
                waypoint_b,
                true,
                true
            );

            draw_pose_card(
                (Rectangle){644.0f, cards_y, 300.0f, card_h},
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
                (Rectangle){334.0f, cards_y, 300.0f, card_h},
                "B",
                "Via point",
                "Position forces the arc through B",
                waypoint_b,
                true,
                false
            );

            draw_pose_card(
                (Rectangle){644.0f, cards_y, 300.0f, card_h},
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
                (Rectangle){334.0f, cards_y, 300.0f, card_h},
                "B",
                "Circle point 2",
                "Position defines the circle plane",
                waypoint_b,
                true,
                false
            );

            draw_pose_card(
                (Rectangle){644.0f, cards_y, 300.0f, card_h},
                "C",
                "Circle point 3",
                "Position defines circle; YPR is final",
                waypoint_c,
                true,
                true
            );
        }

        Rectangle profile_panel = {24.0f, 507.0f, 920.0f, 112.0f};
        panel(profile_panel, C_PANEL);
        text_draw("MOTION PROFILE", 42.0f, 522.0f, 13.0f, C_MUTED, true);
        text_draw(
            "S-curve limits in Cartesian path space",
            42.0f,
            542.0f,
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
            float x = 42.0f + i * 292.0f;
            text_draw(profile_labels[i], x, 568.0f, 12.5f, C_MUTED, false);
            Rectangle field_bounds = {x, 588.0f, 256.0f, 35.0f};
            update_numeric_field(profile_fields[i], field_bounds, true);
            draw_numeric_field(profile_fields[i], field_bounds, true);
        }

        Rectangle action_panel = {24.0f, 633.0f, 920.0f, 78.0f};
        panel(action_panel, C_PANEL);

        Rectangle reference_button = {42.0f, 650.0f, 166.0f, 44.0f};
        Rectangle run_button = {221.0f, 650.0f, 352.0f, 44.0f};
        Rectangle stop_button = {586.0f, 650.0f, 166.0f, 44.0f};

        if (button_ex(reference_button, "LOAD REFERENCE", false, false, true))
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

        const char *run_label =
            selected_path == PATH_LINE
                ? "PLAN + RUN STRAIGHT LINE"
                : selected_path == PATH_ARC
                    ? "PLAN + RUN CIRCULAR ARC"
                    : "PLAN + RUN FULL CIRCLE";

        if (button_ex(run_button, run_label, true, false, true))
        {
            float A[NUM_POSE_VALUES];
            float B[NUM_POSE_VALUES];
            float C[NUM_POSE_VALUES];
            bool values_valid = true;

            for (int i = 0; i < NUM_POSE_VALUES; i++)
            {
                double a_value = 0.0;
                double b_value = 0.0;
                double c_value = 0.0;

                if (
                    !numeric_field_parse(&waypoint_a[i], &a_value) ||
                    !numeric_field_parse(&waypoint_b[i], &b_value) ||
                    !numeric_field_parse(&waypoint_c[i], &c_value)
                )
                {
                    values_valid = false;
                }

                A[i] = (float)a_value;
                B[i] = (float)b_value;
                C[i] = (float)c_value;
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
                    "Input error: waypoint values must be finite and speed/accel/jerk must be > 0."
                );
            }
            else
            {
                sequence++;

                int success =
                    send_plan_command(
                        command_socket,
                        &controller_address,
                        sequence,
                        selected_path,
                        A,
                        B,
                        C,
                        (float)speed_value,
                        (float)accel_value,
                        (float)jerk_value
                    );

                if (success)
                {
                    snprintf(
                        status_text,
                        sizeof(status_text),
                        "PLAN + RUN #%u sent: %s.",
                        sequence,
                        path_name(selected_path)
                    );

                    printf(
                        "\nPLAN + RUN #%u | %s\n"
                        "A: p=[%.6f %.6f %.6f] | YPR=[%.2f %.2f %.2f]\n"
                        "B: p=[%.6f %.6f %.6f] | YPR=[%.2f %.2f %.2f]\n",
                        sequence,
                        path_name(selected_path),
                        A[0], A[1], A[2], A[3], A[4], A[5],
                        B[0], B[1], B[2], B[3], B[4], B[5]
                    );

                    if (selected_path != PATH_LINE)
                    {
                        printf(
                            "C: p=[%.6f %.6f %.6f] | YPR=[%.2f %.2f %.2f]\n",
                            C[0], C[1], C[2], C[3], C[4], C[5]
                        );
                    }

                    printf(
                        "Profile: speed=%.4f accel=%.4f jerk=%.4f\n",
                        speed_value,
                        accel_value,
                        jerk_value
                    );
                    fflush(stdout);
                }
                else
                {
                    snprintf(status_text, sizeof(status_text), "UDP send failed.");
                }
            }
        }

        if (button_ex(stop_button, "STOP / HOLD", false, true, true))
        {
            sequence++;

            int success =
                send_stop_command(
                    command_socket,
                    &controller_address,
                    sequence
                );

            snprintf(
                status_text,
                sizeof(status_text),
                success
                    ? "STOP command #%u sent - hold current position."
                    : "STOP UDP send failed.",
                sequence
            );
        }

        Rectangle state_panel = {968.0f, 96.0f, 288.0f, 615.0f};
        panel(state_panel, C_PANEL);

        text_draw("ROBOT STATE", 988.0f, 114.0f, 13.0f, C_MUTED, true);
        text_draw(
            status_online
                ? motion_state_name_hmi(controller_status.motionState)
                : "OFFLINE",
            988.0f,
            138.0f,
            23.0f,
            status_online ? C_TEXT : C_BAD,
            true
        );

        char line[128];
        snprintf(
            line,
            sizeof(line),
            "Sequence  #%u",
            status_online ? controller_status.lastSequence : 0U
        );
        text_draw(line, 988.0f, 173.0f, 12.5f, C_MUTED, false);

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
        text_draw(line, 988.0f, 195.0f, 12.5f, C_MUTED, false);

        float progress =
            controller_status.trajectoryCount > 0
                ? (float)display_index / (float)controller_status.trajectoryCount
                : 0.0f;

        progress = clampf_local(progress, 0.0f, 1.0f);

        Rectangle progress_track = {988.0f, 222.0f, 248.0f, 8.0f};
        DrawRectangleRounded(progress_track, 1.0f, 8, (Color){37, 44, 56, 255});

        if (progress > 0.0f)
        {
            Rectangle fill =
            {
                progress_track.x,
                progress_track.y,
                progress_track.width * progress,
                progress_track.height
            };
            DrawRectangleRounded(fill, 1.0f, 8, C_ACCENT);
        }

        DrawLine(988, 251, 1236, 251, C_BORDER);
        text_draw("ETHERCAT", 988.0f, 270.0f, 12.5f, C_MUTED, true);

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

        status_dot(996.0f, 309.0f, wkc_good, C_GOOD);
        text_draw(line, 1009.0f, 299.0f, 14.0f, wkc_good ? C_GOOD : C_MUTED, true);
        text_draw("CiA-402 servo states", 988.0f, 330.0f, 12.0f, C_FAINT, false);

        for (int joint = 0; joint < 6; joint++)
        {
            float y = 361.0f + joint * 43.0f;
            uint16_t statusword = controller_status.statusword[joint];
            bool enabled = status_online && cia402_operation_enabled(statusword);

            status_dot(996.0f, y + 10.0f, enabled, C_GOOD);
            snprintf(line, sizeof(line), "J%d", joint + 1);
            text_draw(line, 1010.0f, y, 13.0f, C_TEXT, true);
            text_draw(
                status_online ? cia402_state_name(statusword) : "---",
                1047.0f,
                y,
                11.5f,
                enabled ? C_GOOD : C_MUTED,
                false
            );
            snprintf(line, sizeof(line), "0x%04X", statusword);
            text_draw(line, 1180.0f, y, 10.5f, C_FAINT, false);
        }

        DrawLine(988, 632, 1236, 632, C_BORDER);
        text_draw("Selected path", 988.0f, 651.0f, 11.5f, C_FAINT, false);
        text_draw(path_name(selected_path), 988.0f, 672.0f, 16.0f, C_TEXT, true);
        text_draw("LIVE EXECUTION READY", 988.0f, 694.0f, 10.5f, C_GOOD, true);

        Rectangle footer = {24.0f, 724.0f, 1232.0f, 52.0f};
        DrawRectangleRounded(footer, 0.12f, 10, (Color){20, 25, 33, 255});
        status_dot(43.0f, 750.0f, true, C_ACCENT);
        text_draw(status_text, 59.0f, 739.0f, 13.0f, C_MUTED, false);
        text_draw(
            "Wheel: vertical  |  Shift + wheel: horizontal",
            944.0f,
            740.0f,
            10.5f,
            C_FAINT,
            false
        );

        EndMode2D();
        draw_scrollbars();
        EndDrawing();
    }

    unload_ui_fonts();
    close(status_socket);
    close(command_socket);
    CloseWindow();
    return 0;
}
