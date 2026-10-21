/* Stage 3E: six-axis host-only virtual F/T keyboard GUI.
 * This is an input device, not a motion controller or safety function. */
#include "hmi_protocol.h"
#include "hmi_theme.h"
#include "../Simulation/Guidance/virtual_ft_client.h"
#include "../Simulation/Guidance/virtual_ft_controls.h"
#include "../StateMachine/state_machine_types.h"

#include "raylib.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

enum { WINDOW_W = 1000, WINDOW_H = 750, POINTER_NONE = -1,
       POINTER_FORCE = 12, POINTER_TORQUE = 13, POINTER_ZERO = 14 };

static const Rectangle FORCE_TRACK = {72, 608, 384, 15};
static const Rectangle TORQUE_TRACK = {542, 608, 384, 15};
static const Rectangle ZERO_BUTTON = {743, 670, 203, 48};

static const int negative_key[6] = {
    KEY_LEFT, KEY_DOWN, KEY_PAGE_DOWN, KEY_Q, KEY_S, KEY_A
};
static const int positive_key[6] = {
    KEY_RIGHT, KEY_UP, KEY_PAGE_UP, KEY_E, KEY_W, KEY_D
};
static const char *const negative_label[6] = {
    "Left  -Fx", "Down  -Fy", "PageDown  -Fz", "Q  -Tx", "S  -Ty", "A  -Tz"
};
static const char *const positive_label[6] = {
    "Right  +Fx", "Up  +Fy", "PageUp  +Fz", "E  +Tx", "W  +Ty", "D  +Tz"
};
static const char *const axis_label[6] = {
    "X / Fx (N)", "Y / Fy (N)", "Z / Fz (N)",
    "Roll / Tx (N.m)", "Pitch / Ty (N.m)", "Yaw / Tz (N.m)"
};

static Rectangle key_rect(unsigned axis, bool positive)
{
    return (Rectangle){positive ? 516.0f : 251.0f,
                       218.0f + 52.0f * axis, 235.0f, 40.0f};
}

static float clampf(float value, float lo, float hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static void centered(const char *message, Rectangle area, float size,
                     Color color, bool bold)
{
    const float width = hmi_ui_text_width(message, size, bold);
    hmi_ui_text(message, area.x + (area.width - width)*0.5f,
                area.y + (area.height - size)*0.5f - 2.0f,
                size, color, bold);
}

static void draw_hold(Rectangle bounds, const char *label, bool held,
                      bool enabled, Vector2 mouse)
{
    const bool hover = enabled && CheckCollisionPointRec(mouse, bounds);
    const Color fill = !enabled ? HMI_C_FIELD :
        held ? (Color){46,108,172,255} :
        hover ? HMI_C_FIELD_HI : HMI_C_PANEL_2;
    DrawRectangleRounded(bounds, 0.13f, 8, fill);
    DrawRectangleLinesEx(bounds, held ? 2.0f : 1.0f,
                         held ? HMI_C_ACCENT : HMI_C_BORDER);
    centered(label, bounds, 16.0f,
             enabled ? HMI_C_TEXT : HMI_C_FAINT, true);
}

static void draw_slider(Rectangle track, float value, float maximum,
                        const char *caption, const char *unit, bool dragging)
{
    char text[96];
    snprintf(text, sizeof(text), "%s  %.3f %s", caption, value, unit);
    hmi_ui_text(text, track.x, track.y - 49.0f, 19, HMI_C_TEXT, true);
    DrawRectangleRounded(track, 0.5f, 8, HMI_C_BORDER);
    const float filled_width = track.width * value / maximum;
    DrawRectangleRounded((Rectangle){track.x, track.y, filled_width,
                                   track.height}, 0.5f, 8, HMI_C_ACCENT);
    DrawCircleV((Vector2){track.x + filled_width,
                          track.y + track.height*0.5f}, 11.0f,
                dragging ? HMI_C_TEXT : HMI_C_ACCENT);
    hmi_ui_text("0", track.x, track.y + 21, 14, HMI_C_MUTED, false);
    snprintf(text, sizeof(text), "%.2f %s", maximum, unit);
    hmi_ui_text(text, track.x + track.width -
                hmi_ui_text_width(text, 14, false),
                track.y + 21, 14, HMI_C_MUTED, false);
}

static bool any_nonzero(const double wrench[6])
{
    for (unsigned i = 0; i < 6U; ++i)
        if (wrench[i] != 0.0) return true;
    return false;
}

int main(void)
{
    GpVirtualFtClient client;
    if (!gp_virtual_ft_client_open(&client)) {
        fprintf(stderr, "[VFT GUI] UDP sender socket could not open\n");
        return 1;
    }
    HmiProtocol status;
    if (!hmi_protocol_init(&status, HMI_VFT_STATUS_PORT)) {
        fprintf(stderr, "[VFT GUI] status UDP port 5014 unavailable\n");
        gp_virtual_ft_client_close(&client);
        return 1;
    }
    InitWindow(WINDOW_W, WINDOW_H, "Virtual F/T Sensor | Six-Axis Hand Guiding");
    if (!IsWindowReady()) {
        hmi_protocol_shutdown(&status);
        gp_virtual_ft_client_close(&client);
        return 1;
    }
    SetTargetFPS(120);
    hmi_ui_init();

    /* Must remain within the existing protocol receiver's per-axis limits.
     * Synthetic simulation settings, NOT real sensor/robot ratings. */
    const float force_max_N = 5.0f;
    const float torque_max_Nm = 1.0f;
    float force_N = 0.5f;
    float torque_Nm = 0.05f;
    int pointer = POINTER_NONE;
    GpVirtualFtInputGate input_gate = {0};
    bool transport_error = false;
    double next_send = 0.0;
    double last_sent[6] = {0};

    while (!WindowShouldClose()) {
        hmi_protocol_poll_status(&status);
        const bool online = hmi_protocol_controller_online(&status, 0.5);
        const HmiStatus *s = &status.status;
        const bool fault = online && (s->estop_active ||
            s->robot_state == ROBOT_STATE_FAULT ||
            s->robot_state == ROBOT_STATE_EMERGENCY_STOP);
        const bool guidance = online && !fault &&
            s->robot_state == ROBOT_STATE_TEACHING &&
            s->can_expected_nodes > 0U &&
            s->can_ready_nodes == s->can_expected_nodes;
        const bool focus = IsWindowFocused();
        const bool allowed = guidance && focus;
        const Vector2 mouse = GetMousePosition();
        const bool mouse_down = focus && IsMouseButtonDown(MOUSE_BUTTON_LEFT);
        const bool mouse_pressed = focus && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
        bool clear_pressed = false;

        if (!focus || !mouse_down) pointer = POINTER_NONE;
        if (mouse_pressed) {
            pointer = POINTER_NONE;
            if (CheckCollisionPointRec(mouse, FORCE_TRACK))
                pointer = POINTER_FORCE;
            else if (CheckCollisionPointRec(mouse, TORQUE_TRACK))
                pointer = POINTER_TORQUE;
            else if (CheckCollisionPointRec(mouse, ZERO_BUTTON)) {
                pointer = POINTER_ZERO;
                clear_pressed = true;
            } else if (allowed) {
                for (unsigned i = 0U; i < 6U; ++i) {
                    if (CheckCollisionPointRec(mouse, key_rect(i, false)))
                        pointer = (int)(2U*i);
                    else if (CheckCollisionPointRec(mouse, key_rect(i, true)))
                        pointer = (int)(2U*i + 1U);
                }
            }
        }
        if (mouse_down && pointer == POINTER_FORCE) {
            force_N = force_max_N * clampf((mouse.x - FORCE_TRACK.x) /
                                           FORCE_TRACK.width, 0.0f, 1.0f);
        }
        if (mouse_down && pointer == POINTER_TORQUE) {
            torque_Nm = torque_max_Nm * clampf((mouse.x - TORQUE_TRACK.x) /
                                               TORQUE_TRACK.width, 0.0f, 1.0f);
        }

        GpVirtualFtKeys keys = {0};
        for (unsigned i = 0U; i < 6U; ++i) {
            const bool mouse_neg = mouse_down && pointer == (int)(2U*i) &&
                CheckCollisionPointRec(mouse, key_rect(i, false));
            const bool mouse_pos = mouse_down && pointer == (int)(2U*i + 1U) &&
                CheckCollisionPointRec(mouse, key_rect(i, true));
            keys.negative[i] = focus && (IsKeyDown(negative_key[i]) || mouse_neg);
            keys.positive[i] = focus && (IsKeyDown(positive_key[i]) || mouse_pos);
        }
        gp_virtual_ft_input_gate_update(&input_gate, focus, clear_pressed, &keys);
        const bool enabled = allowed && !input_gate.suppressed_until_release;
        double wrench[6];
        gp_virtual_ft_wrench_request(&keys, enabled, force_N, torque_Nm, wrench);

        /* The GUI streams 50 Hz, not at the 500-Hz controller rate.
         * Send a zero immediately on release, focus loss, or inhibition;
         * keep sending zeros afterward for the existing watchdog. */
        const double now = GetTime();
        if (now >= next_send || (!any_nonzero(wrench) && any_nonzero(last_sent))) {
            transport_error = !gp_virtual_ft_client_send(&client, wrench);
            if (!transport_error) memcpy(last_sent, wrench, sizeof(last_sent));
            next_send = now + 0.020;
        }

        const char *status_text = transport_error ? "Transport error" :
            !online ? "Disconnected" : fault ? "Supervisor fault / E-stop" :
            guidance ? "TEACHING - guidance input enabled" :
            "Waiting for TEACHING / CAN readiness";
        const Color status_color = transport_error || fault ? HMI_C_BAD :
            !online ? HMI_C_FAINT : guidance ? HMI_C_GOOD : HMI_C_WARN;

        BeginDrawing();
        ClearBackground(HMI_C_BG);
        hmi_ui_text("VIRTUAL FORCE / TORQUE", 48, 25, 30, HMI_C_TEXT, true);
        hmi_ui_text("HOST-ONLY TEST INPUT  |  BASE FRAME B, ABOUT TCP",
                    49, 68, 15, HMI_C_MUTED, false);
        hmi_ui_panel((Rectangle){48, 101, 898, 67}, HMI_C_PANEL);
        DrawCircle(77, 129, 7, status_color);
        hmi_ui_text(status_text, 96, 114, 21, HMI_C_TEXT, true);
        char detail[150];
        snprintf(detail, sizeof(detail),
                 "State %u  |  guidance active %s  |  CAN %u/%u  |  error 0x%X",
                 online ? s->robot_state : 0U,
                 online && s->guidance_active ? "yes" : "no",
                 online ? s->can_ready_nodes : 0U,
                 online ? s->can_expected_nodes : 0U,
                 online ? s->guidance_error : 0U);
        hmi_ui_text(detail, 96, 142, 14, HMI_C_MUTED, false);
        hmi_ui_text("AXIS / INPUT", 55, 185, 15, HMI_C_MUTED, true);
        hmi_ui_text("NEGATIVE KEY", 256, 185, 15, HMI_C_MUTED, true);
        hmi_ui_text("POSITIVE KEY", 521, 185, 15, HMI_C_MUTED, true);
        hmi_ui_text("LAST UDP SEND", 770, 185, 15, HMI_C_MUTED, true);

        for (unsigned i = 0U; i < 6U; ++i) {
            const float y = 227.0f + 52.0f*i;
            hmi_ui_text(axis_label[i], 56, y, 16, HMI_C_TEXT, false);
            draw_hold(key_rect(i, false), negative_label[i],
                      keys.negative[i] && !keys.positive[i] && enabled,
                      allowed, mouse);
            draw_hold(key_rect(i, true), positive_label[i],
                      keys.positive[i] && !keys.negative[i] && enabled,
                      allowed, mouse);
            char value[40];
            snprintf(value, sizeof(value), "%+.3f %s", last_sent[i],
                     i < 3U ? "N" : "N.m");
            hmi_ui_text(value, 776, y, 16, HMI_C_TEXT, true);
        }
        draw_slider(FORCE_TRACK, force_N, force_max_N,
                    "Force magnitude", "N",
                    mouse_down && pointer == POINTER_FORCE);
        draw_slider(TORQUE_TRACK, torque_Nm, torque_max_Nm,
                    "Torque magnitude", "N.m",
                    mouse_down && pointer == POINTER_TORQUE);
        hmi_ui_text("Last successful UDP send; receiver acceptance is not guaranteed.",
                    54, 671, 14, HMI_C_MUTED, false);
        hmi_ui_text("Release keys for damped decay. Focus loss / ZERO clears input.",
                    54, 699, 14, HMI_C_MUTED, false);
        hmi_ui_panel(ZERO_BUTTON, HMI_C_PANEL_2);
        centered("ZERO WRENCH", ZERO_BUTTON, 17, HMI_C_TEXT, true);
        EndDrawing();
    }

    /* Best-effort valid zero packets; loss after exit still faults watchdog. */
    gp_virtual_ft_client_close(&client);
    hmi_protocol_shutdown(&status);
    hmi_ui_shutdown();
    CloseWindow();
    return 0;
}
