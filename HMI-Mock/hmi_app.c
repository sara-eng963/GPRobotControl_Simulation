#include "hmi_app.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "raylib.h"

#include "hmi_protocol.h"
#include "hmi_theme.h"
#include "hmi_types.h"
#include "waypoint_editor_3d.h"

/* ============================================================================
 * FIXED-SCALE HMI CANVAS
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

typedef struct
{
    float x;
    float y;

    bool dragX;
    bool dragY;
    float dragOffsetX;
    float dragOffsetY;
    bool pointerBlocked;
} ScrollState;

static float clampf_local(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static Vector2 logical_mouse(const ScrollState *scroll)
{
    Vector2 mouse = GetMousePosition();

    return (Vector2)
    {
        (mouse.x + scroll->x) / UI_SCALE,
        (mouse.y + scroll->y) / UI_SCALE
    };
}

static Rectangle horizontal_scroll_track(void)
{
    float screenWidth = (float)GetScreenWidth();
    float screenHeight = (float)GetScreenHeight();
    bool verticalVisible = UI_CONTENT_HEIGHT > screenHeight;

    float width =
        screenWidth -
        2.0f * UI_SCROLLBAR_MARGIN -
        (verticalVisible ? UI_SCROLLBAR_SIZE + UI_SCROLLBAR_MARGIN : 0.0f);

    if (width < 40.0f) width = 40.0f;

    return (Rectangle)
    {
        UI_SCROLLBAR_MARGIN,
        screenHeight - UI_SCROLLBAR_MARGIN - UI_SCROLLBAR_SIZE,
        width,
        UI_SCROLLBAR_SIZE
    };
}

static Rectangle vertical_scroll_track(void)
{
    float screenWidth = (float)GetScreenWidth();
    float screenHeight = (float)GetScreenHeight();
    bool horizontalVisible = UI_CONTENT_WIDTH > screenWidth;

    float height =
        screenHeight -
        2.0f * UI_SCROLLBAR_MARGIN -
        (horizontalVisible ? UI_SCROLLBAR_SIZE + UI_SCROLLBAR_MARGIN : 0.0f);

    if (height < 40.0f) height = 40.0f;

    return (Rectangle)
    {
        screenWidth - UI_SCROLLBAR_MARGIN - UI_SCROLLBAR_SIZE,
        UI_SCROLLBAR_MARGIN,
        UI_SCROLLBAR_SIZE,
        height
    };
}

static Rectangle horizontal_scroll_thumb(
    Rectangle track,
    float maxScroll,
    const ScrollState *scroll
)
{
    float viewport = (float)GetScreenWidth();
    float width = track.width * (viewport / UI_CONTENT_WIDTH);
    width = clampf_local(width, UI_SCROLLBAR_MIN_THUMB, track.width);

    float usable = track.width - width;
    float x = track.x;

    if (maxScroll > 0.0f && usable > 0.0f)
    {
        x += (scroll->x / maxScroll) * usable;
    }

    return (Rectangle){x, track.y, width, track.height};
}

static Rectangle vertical_scroll_thumb(
    Rectangle track,
    float maxScroll,
    const ScrollState *scroll
)
{
    float viewport = (float)GetScreenHeight();
    float height = track.height * (viewport / UI_CONTENT_HEIGHT);
    height = clampf_local(height, UI_SCROLLBAR_MIN_THUMB, track.height);

    float usable = track.height - height;
    float y = track.y;

    if (maxScroll > 0.0f && usable > 0.0f)
    {
        y += (scroll->y / maxScroll) * usable;
    }

    return (Rectangle){track.x, y, track.width, height};
}

static void update_scrollbars(ScrollState *scroll)
{
    float viewportWidth = (float)GetScreenWidth();
    float viewportHeight = (float)GetScreenHeight();

    float maxScrollX = UI_CONTENT_WIDTH - viewportWidth;
    float maxScrollY = UI_CONTENT_HEIGHT - viewportHeight;

    if (maxScrollX < 0.0f) maxScrollX = 0.0f;
    if (maxScrollY < 0.0f) maxScrollY = 0.0f;

    if (maxScrollX <= 0.0f)
    {
        scroll->x = 0.0f;
        scroll->dragX = false;
    }

    if (maxScrollY <= 0.0f)
    {
        scroll->y = 0.0f;
        scroll->dragY = false;
    }

    float wheel = GetMouseWheelMove();

    if (wheel != 0.0f)
    {
        bool shift =
            IsKeyDown(KEY_LEFT_SHIFT) ||
            IsKeyDown(KEY_RIGHT_SHIFT);

        if (shift || maxScrollY <= 0.0f)
            scroll->x -= wheel * UI_SCROLL_STEP;
        else
            scroll->y -= wheel * UI_SCROLL_STEP;
    }

    scroll->x = clampf_local(scroll->x, 0.0f, maxScrollX);
    scroll->y = clampf_local(scroll->y, 0.0f, maxScrollY);

    Vector2 mouse = GetMousePosition();
    bool horizontalVisible = maxScrollX > 0.0f;
    bool verticalVisible = maxScrollY > 0.0f;

    Rectangle hTrack = {0};
    Rectangle hThumb = {0};
    Rectangle vTrack = {0};
    Rectangle vThumb = {0};

    if (horizontalVisible)
    {
        hTrack = horizontal_scroll_track();
        hThumb = horizontal_scroll_thumb(hTrack, maxScrollX, scroll);
    }

    if (verticalVisible)
    {
        vTrack = vertical_scroll_track();
        vThumb = vertical_scroll_thumb(vTrack, maxScrollY, scroll);
    }

    scroll->pointerBlocked =
        scroll->dragX ||
        scroll->dragY ||
        (horizontalVisible && CheckCollisionPointRec(mouse, hTrack)) ||
        (verticalVisible && CheckCollisionPointRec(mouse, vTrack));

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
        if (horizontalVisible && CheckCollisionPointRec(mouse, hTrack))
        {
            float usable = hTrack.width - hThumb.width;

            if (CheckCollisionPointRec(mouse, hThumb))
            {
                scroll->dragOffsetX = mouse.x - hThumb.x;
            }
            else
            {
                float newX =
                    clampf_local(
                        mouse.x - hThumb.width * 0.5f,
                        hTrack.x,
                        hTrack.x + usable
                    );

                if (usable > 0.0f)
                {
                    scroll->x =
                        ((newX - hTrack.x) / usable) * maxScrollX;
                }

                scroll->dragOffsetX = hThumb.width * 0.5f;
            }

            scroll->dragX = true;
            scroll->dragY = false;
            scroll->pointerBlocked = true;
        }
        else if (verticalVisible && CheckCollisionPointRec(mouse, vTrack))
        {
            float usable = vTrack.height - vThumb.height;

            if (CheckCollisionPointRec(mouse, vThumb))
            {
                scroll->dragOffsetY = mouse.y - vThumb.y;
            }
            else
            {
                float newY =
                    clampf_local(
                        mouse.y - vThumb.height * 0.5f,
                        vTrack.y,
                        vTrack.y + usable
                    );

                if (usable > 0.0f)
                {
                    scroll->y =
                        ((newY - vTrack.y) / usable) * maxScrollY;
                }

                scroll->dragOffsetY = vThumb.height * 0.5f;
            }

            scroll->dragY = true;
            scroll->dragX = false;
            scroll->pointerBlocked = true;
        }
    }

    if (scroll->dragX)
    {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            float usable = hTrack.width - hThumb.width;
            float newX =
                clampf_local(
                    mouse.x - scroll->dragOffsetX,
                    hTrack.x,
                    hTrack.x + usable
                );

            if (usable > 0.0f)
            {
                scroll->x =
                    ((newX - hTrack.x) / usable) * maxScrollX;
            }
        }
        else
        {
            scroll->dragX = false;
        }
    }

    if (scroll->dragY)
    {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            float usable = vTrack.height - vThumb.height;
            float newY =
                clampf_local(
                    mouse.y - scroll->dragOffsetY,
                    vTrack.y,
                    vTrack.y + usable
                );

            if (usable > 0.0f)
            {
                scroll->y =
                    ((newY - vTrack.y) / usable) * maxScrollY;
            }
        }
        else
        {
            scroll->dragY = false;
        }
    }

    scroll->x = clampf_local(scroll->x, 0.0f, maxScrollX);
    scroll->y = clampf_local(scroll->y, 0.0f, maxScrollY);
}

static void draw_scrollbars(const ScrollState *scroll)
{
    float maxScrollX = UI_CONTENT_WIDTH - (float)GetScreenWidth();
    float maxScrollY = UI_CONTENT_HEIGHT - (float)GetScreenHeight();

    if (maxScrollX > 0.0f)
    {
        Rectangle track = horizontal_scroll_track();
        Rectangle thumb = horizontal_scroll_thumb(track, maxScrollX, scroll);

        DrawRectangleRounded(track, 1.0f, 8, (Color){35, 41, 52, 245});
        DrawRectangleRounded(
            thumb,
            1.0f,
            8,
            scroll->dragX ? HMI_C_ACCENT : (Color){91, 105, 129, 255}
        );
    }

    if (maxScrollY > 0.0f)
    {
        Rectangle track = vertical_scroll_track();
        Rectangle thumb = vertical_scroll_thumb(track, maxScrollY, scroll);

        DrawRectangleRounded(track, 1.0f, 8, (Color){35, 41, 52, 245});
        DrawRectangleRounded(
            thumb,
            1.0f,
            8,
            scroll->dragY ? HMI_C_ACCENT : (Color){91, 105, 129, 255}
        );
    }
}

/* ============================================================================
 * HMI DOMAIN HELPERS
 * ============================================================================
 */

static const char *path_name(HmiPathType path)
{
    switch (path)
    {
        case HMI_PATH_LINE:        return "Straight line";
        case HMI_PATH_ARC:         return "Circular arc";
        case HMI_PATH_FULL_CIRCLE: return "Full circle";
        default:                   return "Unknown";
    }
}

static const char *motion_state_name(uint32_t state)
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

static bool path_tab(
    Rectangle bounds,
    const char *title,
    const char *subtitle,
    bool selected,
    Vector2 mouse,
    bool pointerBlocked
)
{
    bool hovered =
        !pointerBlocked &&
        CheckCollisionPointRec(mouse, bounds);

    Color fill =
        selected
            ? (Color){31, 55, 82, 255}
            : hovered
                ? (Color){31, 38, 49, 255}
                : (Color){24, 29, 38, 255};

    Color border =
        selected
            ? HMI_C_ACCENT
            : hovered
                ? HMI_C_BORDER_HI
                : HMI_C_BORDER;

    DrawRectangleRounded(bounds, 0.09f, 10, fill);
    DrawRectangleLinesEx(bounds, selected ? 2.0f : 1.0f, border);

    hmi_ui_text(
        title,
        bounds.x + 14.0f,
        bounds.y + 9.0f,
        16.0f,
        selected ? HMI_C_TEXT : (Color){214, 221, 232, 255},
        true
    );

    hmi_ui_text(
        subtitle,
        bounds.x + 14.0f,
        bounds.y + 31.0f,
        12.5f,
        HMI_C_MUTED,
        false
    );

    return
        hovered &&
        IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

static void set_pose_fields(
    HmiNumericField fields[HMI_NUM_POSE_VALUES],
    const double values[HMI_NUM_POSE_VALUES]
)
{
    for (int i = 0; i < HMI_NUM_POSE_VALUES; i++)
    {
        hmi_numeric_field_set(
            &fields[i],
            values[i],
            i < 3 ? 6 : 2
        );
    }
}

static void load_reference_case(
    HmiPathType path,
    HmiNumericField waypointFields[HMI_NUM_WAYPOINTS][HMI_NUM_POSE_VALUES],
    HmiNumericField *speed,
    HmiNumericField *accel,
    HmiNumericField *jerk
)
{
    const double A[HMI_NUM_POSE_VALUES] =
    {
        -0.421962,
        -0.451956,
         0.234228,
       104.80,
       -68.51,
        38.48
    };

    double B[HMI_NUM_POSE_VALUES] =
    {
        0.0, 0.0, 0.0,
        A[3], A[4], A[5]
    };

    double C[HMI_NUM_POSE_VALUES] =
    {
        0.0, 0.0, 0.0,
        A[3], A[4], A[5]
    };

    if (path == HMI_PATH_LINE)
    {
        const double lineB[HMI_NUM_POSE_VALUES] =
        {
             0.078038,
            -0.451956,
             0.234228,
           147.58,
           -58.31,
            12.75
        };

        memcpy(B, lineB, sizeof(B));
        memcpy(C, lineB, sizeof(C));
    }
    else if (path == HMI_PATH_ARC)
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

    set_pose_fields(waypointFields[0], A);
    set_pose_fields(waypointFields[1], B);
    set_pose_fields(waypointFields[2], C);

    hmi_numeric_field_set(speed, 0.10, 3);
    hmi_numeric_field_set(accel, 0.25, 3);
    hmi_numeric_field_set(jerk, 1.00, 3);
}

static bool parse_pose_fields(
    HmiNumericField fields[HMI_NUM_POSE_VALUES],
    HmiPose *pose
)
{
    if (pose == NULL)
    {
        return false;
    }

    for (int i = 0; i < HMI_NUM_POSE_VALUES; i++)
    {
        double value = 0.0;

        if (!hmi_numeric_field_parse(&fields[i], &value))
        {
            return false;
        }

        pose->value[i] = (float)value;
    }

    return true;
}

static bool parse_all_poses(
    HmiNumericField fields[HMI_NUM_WAYPOINTS][HMI_NUM_POSE_VALUES],
    HmiPose poses[HMI_NUM_WAYPOINTS]
)
{
    for (int waypoint = 0; waypoint < HMI_NUM_WAYPOINTS; waypoint++)
    {
        if (!parse_pose_fields(fields[waypoint], &poses[waypoint]))
        {
            return false;
        }
    }

    return true;
}

static void apply_editor_positions_to_fields(
    const HmiPose poses[HMI_NUM_WAYPOINTS],
    HmiPathType path,
    HmiNumericField fields[HMI_NUM_WAYPOINTS][HMI_NUM_POSE_VALUES]
)
{
    int count = path == HMI_PATH_LINE ? 2 : 3;

    for (int waypoint = 0; waypoint < count; waypoint++)
    {
        for (int axis = 0; axis < 3; axis++)
        {
            hmi_numeric_field_set(
                &fields[waypoint][axis],
                poses[waypoint].value[axis],
                6
            );
        }
    }
}

static void draw_pose_card(
    Rectangle bounds,
    const char *badge,
    const char *title,
    const char *subtitle,
    HmiNumericField fields[HMI_NUM_POSE_VALUES],
    bool positionEnabled,
    bool orientationEnabled,
    Vector2 mouse,
    bool pointerBlocked
)
{
    hmi_ui_panel(bounds, HMI_C_PANEL);

    DrawRectangleRounded(
        (Rectangle)
        {
            bounds.x + 15.0f,
            bounds.y + 14.0f,
            30.0f,
            24.0f
        },
        0.25f,
        8,
        (Color){31, 55, 82, 255}
    );

    hmi_ui_text(
        badge,
        bounds.x + 25.0f,
        bounds.y + 18.0f,
        13.0f,
        HMI_C_ACCENT,
        true
    );

    hmi_ui_text(
        title,
        bounds.x + 56.0f,
        bounds.y + 14.0f,
        17.0f,
        HMI_C_TEXT,
        true
    );

    hmi_ui_text(
        subtitle,
        bounds.x + 15.0f,
        bounds.y + 45.0f,
        12.5f,
        HMI_C_MUTED,
        false
    );

    const char *labels[HMI_NUM_POSE_VALUES] =
    {
        "X   m",
        "Y   m",
        "Z   m",
        "Yaw   deg",
        "Pitch   deg",
        "Roll   deg"
    };

    float columnGap = 10.0f;
    float innerWidth = bounds.width - 30.0f;
    float fieldWidth = (innerWidth - columnGap) * 0.5f;

    for (int i = 0; i < HMI_NUM_POSE_VALUES; i++)
    {
        int column = i < 3 ? 0 : 1;
        int row = i < 3 ? i : i - 3;

        float columnX =
            bounds.x +
            15.0f +
            column * (fieldWidth + columnGap);

        float rowY =
            bounds.y +
            77.0f +
            row * 67.0f;

        bool enabled =
            column == 0
                ? positionEnabled
                : orientationEnabled;

        hmi_ui_text(
            labels[i],
            columnX,
            rowY,
            12.5f,
            enabled ? HMI_C_MUTED : HMI_C_FAINT,
            false
        );

        Rectangle fieldBounds =
        {
            columnX,
            rowY + 20.0f,
            fieldWidth,
            37.0f
        };

        hmi_numeric_field_update_draw(
            &fields[i],
            fieldBounds,
            enabled,
            mouse,
            pointerBlocked
        );
    }
}

static Vector2 quadratic_bezier(
    Vector2 a,
    Vector2 b,
    Vector2 c,
    float t
)
{
    float u = 1.0f - t;

    return (Vector2)
    {
        u * u * a.x + 2.0f * u * t * b.x + t * t * c.x,
        u * u * a.y + 2.0f * u * t * b.y + t * t * c.y
    };
}

static void draw_path_preview(
    Rectangle bounds,
    HmiPathType path
)
{
    hmi_ui_panel(bounds, HMI_C_PANEL);

    hmi_ui_text(
        "PATH PREVIEW",
        bounds.x + 14.0f,
        bounds.y + 11.0f,
        11.5f,
        HMI_C_FAINT,
        true
    );

    hmi_ui_text(
        path_name(path),
        bounds.x + 14.0f,
        bounds.y + 31.0f,
        15.0f,
        HMI_C_TEXT,
        true
    );

    Vector2 a =
    {
        bounds.x + 155.0f,
        bounds.y + bounds.height * 0.58f
    };

    Vector2 c =
    {
        bounds.x + bounds.width - 24.0f,
        bounds.y + bounds.height * 0.58f
    };

    if (path == HMI_PATH_LINE)
    {
        DrawLineEx(a, c, 3.0f, HMI_C_ACCENT);
        DrawCircleV(a, 6.0f, HMI_C_GOOD);
        DrawCircleV(c, 6.0f, HMI_C_BAD);

        hmi_ui_text("A", a.x - 4.0f, a.y + 10.0f, 11.0f, HMI_C_MUTED, true);
        hmi_ui_text("B", c.x - 4.0f, c.y + 10.0f, 11.0f, HMI_C_MUTED, true);
    }
    else if (path == HMI_PATH_ARC)
    {
        Vector2 b =
        {
            (a.x + c.x) * 0.5f,
            bounds.y + 24.0f
        };

        Vector2 previous = a;

        for (int i = 1; i <= 24; i++)
        {
            float t = (float)i / 24.0f;
            Vector2 point = quadratic_bezier(a, b, c, t);
            DrawLineEx(previous, point, 3.0f, HMI_C_ACCENT);
            previous = point;
        }

        DrawCircleV(a, 6.0f, HMI_C_GOOD);
        DrawCircleV(b, 6.0f, HMI_C_WARN);
        DrawCircleV(c, 6.0f, HMI_C_BAD);

        hmi_ui_text("A", a.x - 4.0f, a.y + 10.0f, 11.0f, HMI_C_MUTED, true);
        hmi_ui_text("B", b.x - 4.0f, b.y - 18.0f, 11.0f, HMI_C_MUTED, true);
        hmi_ui_text("C", c.x - 4.0f, c.y + 10.0f, 11.0f, HMI_C_MUTED, true);
    }
    else
    {
        Vector2 center =
        {
            bounds.x + bounds.width - 63.0f,
            bounds.y + bounds.height * 0.56f
        };

        float radius = 28.0f;

        DrawCircleLines(
            (int)center.x,
            (int)center.y,
            radius,
            HMI_C_ACCENT
        );

        Vector2 p1 = {center.x - radius, center.y};
        Vector2 p2 = {center.x, center.y - radius};
        Vector2 p3 = {center.x + radius, center.y};

        DrawCircleV(p1, 5.0f, HMI_C_GOOD);
        DrawCircleV(p2, 5.0f, HMI_C_WARN);
        DrawCircleV(p3, 5.0f, HMI_C_BAD);

        hmi_ui_text("A", p1.x - 4.0f, p1.y + 9.0f, 10.5f, HMI_C_MUTED, true);
        hmi_ui_text("B", p2.x - 4.0f, p2.y - 16.0f, 10.5f, HMI_C_MUTED, true);
        hmi_ui_text("C", p3.x - 4.0f, p3.y + 9.0f, 10.5f, HMI_C_MUTED, true);
    }
}

static void draw_controller_sidebar(
    const HmiProtocol *protocol,
    bool online,
    HmiPathType selectedPath
)
{
    Rectangle statePanel = {968.0f, 96.0f, 288.0f, 615.0f};
    hmi_ui_panel(statePanel, HMI_C_PANEL);

    const HmiControllerStatus *status = &protocol->status;

    hmi_ui_text(
        "ROBOT STATE",
        988.0f,
        114.0f,
        13.0f,
        HMI_C_MUTED,
        true
    );

    hmi_ui_text(
        online ? motion_state_name(status->motionState) : "OFFLINE",
        988.0f,
        138.0f,
        23.0f,
        online ? HMI_C_TEXT : HMI_C_BAD,
        true
    );

    char line[128];

    snprintf(
        line,
        sizeof(line),
        "Sequence  #%u",
        online ? status->lastSequence : 0U
    );

    hmi_ui_text(line, 988.0f, 173.0f, 12.5f, HMI_C_MUTED, false);

    uint32_t displayIndex =
        status->trajectoryCount > 0
            ? status->trajectoryIndex + 1
            : 0;

    snprintf(
        line,
        sizeof(line),
        "Trajectory  %u / %u",
        displayIndex,
        status->trajectoryCount
    );

    hmi_ui_text(line, 988.0f, 195.0f, 12.5f, HMI_C_MUTED, false);

    float progress =
        status->trajectoryCount > 0
            ? (float)displayIndex /
              (float)status->trajectoryCount
            : 0.0f;

    progress = clampf_local(progress, 0.0f, 1.0f);

    Rectangle progressTrack = {988.0f, 222.0f, 248.0f, 8.0f};

    DrawRectangleRounded(
        progressTrack,
        1.0f,
        8,
        (Color){37, 44, 56, 255}
    );

    if (progress > 0.0f)
    {
        Rectangle progressFill =
        {
            progressTrack.x,
            progressTrack.y,
            progressTrack.width * progress,
            progressTrack.height
        };

        DrawRectangleRounded(progressFill, 1.0f, 8, HMI_C_ACCENT);
    }

    DrawLine(988, 251, 1236, 251, HMI_C_BORDER);

    hmi_ui_text(
        "ETHERCAT",
        988.0f,
        270.0f,
        12.5f,
        HMI_C_MUTED,
        true
    );

    bool wkcGood =
        online &&
        status->expectedWkc > 0 &&
        status->wkc >= status->expectedWkc;

    snprintf(
        line,
        sizeof(line),
        "WKC  %u / %u",
        status->wkc,
        status->expectedWkc
    );

    hmi_ui_status_dot(996.0f, 309.0f, wkcGood, HMI_C_GOOD);

    hmi_ui_text(
        line,
        1009.0f,
        299.0f,
        14.0f,
        wkcGood ? HMI_C_GOOD : HMI_C_MUTED,
        true
    );

    hmi_ui_text(
        "CiA-402 servo states",
        988.0f,
        330.0f,
        12.0f,
        HMI_C_FAINT,
        false
    );

    for (int joint = 0; joint < HMI_NUM_JOINTS; joint++)
    {
        float y = 361.0f + joint * 43.0f;
        uint16_t statusword = status->statusword[joint];

        bool enabled =
            online &&
            cia402_operation_enabled(statusword);

        hmi_ui_status_dot(996.0f, y + 10.0f, enabled, HMI_C_GOOD);

        snprintf(line, sizeof(line), "J%d", joint + 1);
        hmi_ui_text(line, 1010.0f, y, 13.0f, HMI_C_TEXT, true);

        hmi_ui_text(
            online ? cia402_state_name(statusword) : "---",
            1047.0f,
            y,
            11.5f,
            enabled ? HMI_C_GOOD : HMI_C_MUTED,
            false
        );

        snprintf(line, sizeof(line), "0x%04X", statusword);
        hmi_ui_text(line, 1180.0f, y, 10.5f, HMI_C_FAINT, false);
    }

    DrawLine(988, 632, 1236, 632, HMI_C_BORDER);

    hmi_ui_text(
        "Selected path",
        988.0f,
        651.0f,
        11.5f,
        HMI_C_FAINT,
        false
    );

    hmi_ui_text(
        path_name(selectedPath),
        988.0f,
        672.0f,
        16.0f,
        HMI_C_TEXT,
        true
    );

    hmi_ui_text(
        "LIVE EXECUTION READY",
        988.0f,
        694.0f,
        10.5f,
        HMI_C_GOOD,
        true
    );
}

/* ============================================================================
 * APPLICATION
 * ============================================================================
 */

int hmi_app_run(void)
{
    HmiProtocol protocol;

    if (!hmi_protocol_init(&protocol))
    {
        return 1;
    }

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);

    InitWindow(
        (int)UI_WIDTH,
        (int)UI_HEIGHT,
        "Robot Motion Console"
    );

    SetWindowMinSize(720, 480);
    SetTargetFPS(60);

    hmi_ui_init();

    HmiWaypointEditor3D editor;

    if (!hmi_waypoint_editor_init(&editor))
    {
        fprintf(stderr, "Could not initialize 3D waypoint editor\n");
        hmi_ui_shutdown();
        CloseWindow();
        hmi_protocol_shutdown(&protocol);
        return 1;
    }

    HmiNumericField waypointFields[HMI_NUM_WAYPOINTS][HMI_NUM_POSE_VALUES] = {0};
    HmiNumericField speedField = {0};
    HmiNumericField accelField = {0};
    HmiNumericField jerkField = {0};

    HmiPathType selectedPath = HMI_PATH_LINE;

    load_reference_case(
        selectedPath,
        waypointFields,
        &speedField,
        &accelField,
        &jerkField
    );

    ScrollState scroll = {0};
    uint32_t sequence = 0;

    char statusText[360];
    snprintf(
        statusText,
        sizeof(statusText),
        "Ready. Line, circular arc and full-circle live execution are connected."
    );

    while (!WindowShouldClose())
    {
        hmi_protocol_poll_status(&protocol);

        bool editorOpen =
            hmi_waypoint_editor_is_open(&editor);

        if (!editorOpen)
        {
            update_scrollbars(&scroll);
        }
        else
        {
            scroll.pointerBlocked = true;
        }

        bool statusOnline =
            hmi_protocol_controller_online(&protocol, 1.0);

        Vector2 mouse = logical_mouse(&scroll);

        Camera2D camera =
        {
            .offset = {-scroll.x, -scroll.y},
            .target = {0.0f, 0.0f},
            .rotation = 0.0f,
            .zoom = UI_SCALE
        };

        BeginDrawing();
        ClearBackground(HMI_C_BG);
        BeginMode2D(camera);

        DrawRectangle(
            0,
            0,
            (int)UI_WIDTH,
            (int)UI_HEIGHT,
            HMI_C_BG
        );

        /* Header */
        hmi_ui_text(
            "Robot Motion Console",
            24.0f,
            18.0f,
            27.0f,
            HMI_C_TEXT,
            true
        );

        hmi_ui_text(
            "Cartesian planner  /  EtherCAT CSP  /  ADLS IK  /  3D waypoint teaching",
            25.0f,
            53.0f,
            13.5f,
            HMI_C_MUTED,
            false
        );

        Rectangle headerStatus =
        {
            1005.0f,
            18.0f,
            251.0f,
            48.0f
        };

        DrawRectangleRounded(headerStatus, 0.20f, 10, HMI_C_PANEL);
        DrawRectangleLinesEx(headerStatus, 1.0f, HMI_C_BORDER);

        hmi_ui_status_dot(
            1023.0f,
            42.0f,
            statusOnline,
            HMI_C_GOOD
        );

        hmi_ui_text(
            statusOnline ? "CONTROLLER ONLINE" : "CONTROLLER OFFLINE",
            1037.0f,
            27.0f,
            12.5f,
            statusOnline ? HMI_C_GOOD : HMI_C_BAD,
            true
        );

        hmi_ui_text(
            statusOnline
                ? motion_state_name(protocol.status.motionState)
                : "Waiting for status packets",
            1037.0f,
            46.0f,
            11.5f,
            HMI_C_MUTED,
            false
        );

        DrawLine(24, 82, 1256, 82, HMI_C_BORDER);

        /* Path tabs */
        hmi_ui_text(
            "PATH GEOMETRY",
            24.0f,
            96.0f,
            12.5f,
            HMI_C_MUTED,
            true
        );

        Rectangle lineTab = {24.0f, 118.0f, 188.0f, 58.0f};
        Rectangle arcTab = {220.0f, 118.0f, 188.0f, 58.0f};
        Rectangle circleTab = {416.0f, 118.0f, 188.0f, 58.0f};

        HmiPathType previousPath = selectedPath;

        if (
            path_tab(
                lineTab,
                "Straight line",
                "A  ->  B",
                selectedPath == HMI_PATH_LINE,
                mouse,
                scroll.pointerBlocked
            )
        )
        {
            selectedPath = HMI_PATH_LINE;
        }

        if (
            path_tab(
                arcTab,
                "Circular arc",
                "A  ->  via B  ->  C",
                selectedPath == HMI_PATH_ARC,
                mouse,
                scroll.pointerBlocked
            )
        )
        {
            selectedPath = HMI_PATH_ARC;
        }

        if (
            path_tab(
                circleTab,
                "Full circle",
                "A + B + C define circle",
                selectedPath == HMI_PATH_FULL_CIRCLE,
                mouse,
                scroll.pointerBlocked
            )
        )
        {
            selectedPath = HMI_PATH_FULL_CIRCLE;
        }

        if (selectedPath != previousPath)
        {
            load_reference_case(
                selectedPath,
                waypointFields,
                &speedField,
                &accelField,
                &jerkField
            );

            snprintf(
                statusText,
                sizeof(statusText),
                "%s selected. You can type coordinates or use 3D TEACH WAYPOINTS.",
                path_name(selectedPath)
            );
        }

        draw_path_preview(
            (Rectangle){622.0f, 96.0f, 334.0f, 80.0f},
            selectedPath
        );

        /* Waypoint cards */
        const float cardsY = 194.0f;
        const float cardH = 298.0f;

        draw_pose_card(
            (Rectangle){24.0f, cardsY, 300.0f, cardH},
            "A",
            "Start pose",
            "TCP position + start orientation",
            waypointFields[0],
            true,
            true,
            mouse,
            scroll.pointerBlocked
        );

        if (selectedPath == HMI_PATH_LINE)
        {
            draw_pose_card(
                (Rectangle){334.0f, cardsY, 300.0f, cardH},
                "B",
                "End pose",
                "TCP position + final orientation",
                waypointFields[1],
                true,
                true,
                mouse,
                scroll.pointerBlocked
            );

            draw_pose_card(
                (Rectangle){644.0f, cardsY, 300.0f, cardH},
                "C",
                "Not used",
                "Straight line only needs A and B",
                waypointFields[2],
                false,
                false,
                mouse,
                scroll.pointerBlocked
            );
        }
        else if (selectedPath == HMI_PATH_ARC)
        {
            draw_pose_card(
                (Rectangle){334.0f, cardsY, 300.0f, cardH},
                "B",
                "Via point",
                "Position forces the arc through B",
                waypointFields[1],
                true,
                false,
                mouse,
                scroll.pointerBlocked
            );

            draw_pose_card(
                (Rectangle){644.0f, cardsY, 300.0f, cardH},
                "C",
                "End pose",
                "Arc endpoint + final orientation",
                waypointFields[2],
                true,
                true,
                mouse,
                scroll.pointerBlocked
            );
        }
        else
        {
            draw_pose_card(
                (Rectangle){334.0f, cardsY, 300.0f, cardH},
                "B",
                "Circle point 2",
                "Position defines the circle plane",
                waypointFields[1],
                true,
                false,
                mouse,
                scroll.pointerBlocked
            );

            draw_pose_card(
                (Rectangle){644.0f, cardsY, 300.0f, cardH},
                "C",
                "Circle point 3",
                "Position defines circle; YPR is final",
                waypointFields[2],
                true,
                true,
                mouse,
                scroll.pointerBlocked
            );
        }

        /* Motion profile */
        Rectangle profilePanel = {24.0f, 507.0f, 920.0f, 112.0f};
        hmi_ui_panel(profilePanel, HMI_C_PANEL);

        hmi_ui_text(
            "MOTION PROFILE",
            42.0f,
            522.0f,
            13.0f,
            HMI_C_MUTED,
            true
        );

        hmi_ui_text(
            "S-curve limits in Cartesian path space",
            42.0f,
            542.0f,
            12.0f,
            HMI_C_FAINT,
            false
        );

        const char *profileLabels[3] =
        {
            "TCP speed   m/s",
            "TCP acceleration   m/s^2",
            "TCP jerk   m/s^3"
        };

        HmiNumericField *profileFields[3] =
        {
            &speedField,
            &accelField,
            &jerkField
        };

        for (int i = 0; i < 3; i++)
        {
            float x = 42.0f + i * 292.0f;

            hmi_ui_text(
                profileLabels[i],
                x,
                568.0f,
                12.5f,
                HMI_C_MUTED,
                false
            );

            Rectangle fieldBounds = {x, 588.0f, 256.0f, 35.0f};

            hmi_numeric_field_update_draw(
                profileFields[i],
                fieldBounds,
                true,
                mouse,
                scroll.pointerBlocked
            );
        }

        /* Action bar */
        Rectangle actionPanel = {24.0f, 633.0f, 920.0f, 78.0f};
        hmi_ui_panel(actionPanel, HMI_C_PANEL);

        Rectangle referenceButton = {42.0f, 650.0f, 145.0f, 44.0f};
        Rectangle teachButton = {198.0f, 650.0f, 170.0f, 44.0f};
        Rectangle runButton = {379.0f, 650.0f, 348.0f, 44.0f};
        Rectangle stopButton = {738.0f, 650.0f, 188.0f, 44.0f};

        if (
            hmi_ui_button(
                referenceButton,
                "LOAD REFERENCE",
                HMI_BUTTON_NORMAL,
                true,
                mouse,
                scroll.pointerBlocked
            )
        )
        {
            load_reference_case(
                selectedPath,
                waypointFields,
                &speedField,
                &accelField,
                &jerkField
            );

            snprintf(
                statusText,
                sizeof(statusText),
                "Loaded %s reference values.",
                path_name(selectedPath)
            );
        }

        if (
            hmi_ui_button(
                teachButton,
                "3D TEACH WAYPOINTS",
                HMI_BUTTON_NORMAL,
                true,
                mouse,
                scroll.pointerBlocked
            )
        )
        {
            HmiPose poses[HMI_NUM_WAYPOINTS];

            if (!parse_all_poses(waypointFields, poses))
            {
                snprintf(
                    statusText,
                    sizeof(statusText),
                    "Input error: fix waypoint numeric fields before opening 3D teaching."
                );
            }
            else
            {
                hmi_waypoint_editor_open(
                    &editor,
                    selectedPath,
                    poses
                );

                snprintf(
                    statusText,
                    sizeof(statusText),
                    "3D teaching opened for %s.",
                    path_name(selectedPath)
                );
            }
        }

        const char *runLabel =
            selectedPath == HMI_PATH_LINE
                ? "PLAN + RUN STRAIGHT LINE"
                : selectedPath == HMI_PATH_ARC
                    ? "PLAN + RUN CIRCULAR ARC"
                    : "PLAN + RUN FULL CIRCLE";

        if (
            hmi_ui_button(
                runButton,
                runLabel,
                HMI_BUTTON_PRIMARY,
                statusOnline,
                mouse,
                scroll.pointerBlocked
            )
        )
        {
            HmiPose poses[HMI_NUM_WAYPOINTS];
            double speed = 0.0;
            double accel = 0.0;
            double jerk = 0.0;

            bool valid =
                parse_all_poses(waypointFields, poses) &&
                hmi_numeric_field_parse(&speedField, &speed) &&
                hmi_numeric_field_parse(&accelField, &accel) &&
                hmi_numeric_field_parse(&jerkField, &jerk) &&
                speed > 0.0 &&
                accel > 0.0 &&
                jerk > 0.0;

            if (!valid)
            {
                snprintf(
                    statusText,
                    sizeof(statusText),
                    "Input error: waypoint values must be finite and speed/accel/jerk must be > 0."
                );
            }
            else
            {
                sequence++;

                bool sent =
                    hmi_protocol_send_plan(
                        &protocol,
                        selectedPath,
                        sequence,
                        &poses[0],
                        &poses[1],
                        &poses[2],
                        (float)speed,
                        (float)accel,
                        (float)jerk
                    );

                if (sent)
                {
                    snprintf(
                        statusText,
                        sizeof(statusText),
                        "PLAN + RUN #%u sent: %s.",
                        sequence,
                        path_name(selectedPath)
                    );

                    printf(
                        "\nPLAN + RUN #%u | %s\n",
                        sequence,
                        path_name(selectedPath)
                    );

                    int waypointCount =
                        selectedPath == HMI_PATH_LINE ? 2 : 3;

                    for (int i = 0; i < waypointCount; i++)
                    {
                        printf(
                            "%c: p=[%.6f %.6f %.6f] m | YPR=[%.2f %.2f %.2f] deg\n",
                            'A' + i,
                            poses[i].value[0],
                            poses[i].value[1],
                            poses[i].value[2],
                            poses[i].value[3],
                            poses[i].value[4],
                            poses[i].value[5]
                        );
                    }

                    printf(
                        "Profile: speed=%.4f m/s accel=%.4f m/s^2 jerk=%.4f m/s^3\n",
                        speed,
                        accel,
                        jerk
                    );

                    fflush(stdout);
                }
                else
                {
                    snprintf(
                        statusText,
                        sizeof(statusText),
                        "UDP plan send failed."
                    );
                }
            }
        }

        if (
            hmi_ui_button(
                stopButton,
                "STOP / HOLD",
                HMI_BUTTON_DANGER,
                statusOnline,
                mouse,
                scroll.pointerBlocked
            )
        )
        {
            sequence++;

            if (hmi_protocol_send_stop(&protocol, sequence))
            {
                snprintf(
                    statusText,
                    sizeof(statusText),
                    "STOP command #%u sent - hold current position.",
                    sequence
                );
            }
            else
            {
                snprintf(
                    statusText,
                    sizeof(statusText),
                    "STOP UDP send failed."
                );
            }
        }

        draw_controller_sidebar(
            &protocol,
            statusOnline,
            selectedPath
        );

        /* Footer */
        Rectangle footer = {24.0f, 724.0f, 1232.0f, 52.0f};

        DrawRectangleRounded(
            footer,
            0.12f,
            10,
            (Color){20, 25, 33, 255}
        );

        hmi_ui_status_dot(
            43.0f,
            750.0f,
            true,
            HMI_C_ACCENT
        );

        hmi_ui_text(
            statusText,
            59.0f,
            739.0f,
            13.0f,
            HMI_C_MUTED,
            false
        );

        hmi_ui_text(
            "Wheel: vertical  |  Shift + wheel: horizontal",
            944.0f,
            740.0f,
            10.5f,
            HMI_C_FAINT,
            false
        );

        EndMode2D();

        if (!editorOpen)
        {
            draw_scrollbars(&scroll);
        }

        if (hmi_waypoint_editor_is_open(&editor))
        {
            float margin = 18.0f;

            Rectangle editorBounds =
            {
                margin,
                margin,
                (float)GetScreenWidth() - 2.0f * margin,
                (float)GetScreenHeight() - 2.0f * margin
            };

            if (editorBounds.width < 680.0f)
            {
                editorBounds.x = 5.0f;
                editorBounds.width = (float)GetScreenWidth() - 10.0f;
            }

            if (editorBounds.height < 430.0f)
            {
                editorBounds.y = 5.0f;
                editorBounds.height = (float)GetScreenHeight() - 10.0f;
            }

            HmiPose edited[HMI_NUM_WAYPOINTS];

            HmiWaypointEditorResult result =
                hmi_waypoint_editor_frame(
                    &editor,
                    editorBounds,
                    GetMousePosition(),
                    edited
                );

            if (result == HMI_EDITOR_RESULT_APPLIED)
            {
                apply_editor_positions_to_fields(
                    edited,
                    selectedPath,
                    waypointFields
                );

                snprintf(
                    statusText,
                    sizeof(statusText),
                    "Applied 3D-taught XYZ coordinates to %s waypoints.",
                    path_name(selectedPath)
                );
            }
            else if (result == HMI_EDITOR_RESULT_CANCELLED)
            {
                snprintf(
                    statusText,
                    sizeof(statusText),
                    "3D waypoint teaching cancelled; original coordinates kept."
                );
            }
        }

        EndDrawing();
    }

    hmi_waypoint_editor_shutdown(&editor);
    hmi_ui_shutdown();
    CloseWindow();
    hmi_protocol_shutdown(&protocol);

    return 0;
}
