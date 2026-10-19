/* Stage 4F2B standalone virtual F/T input. Host simulation ONLY.
 * This GUI produces sensor-equivalent wrench samples. It never sends
 * Cartesian targets, joint references, safety resets, or CANopen frames. */
#include "hmi_protocol.h"
#include "hmi_theme.h"
#include "../Simulation/Guidance/virtual_ft_client.h"
#include "../Simulation/Guidance/virtual_ft_controls.h"
#include "../StateMachine/state_machine_types.h"

#include "raylib.h"
#include <stdbool.h>
#include <stdio.h>

enum { WINDOW_W = 850, WINDOW_H = 570 };
enum {
    POINTER_NONE = 0, POINTER_POS_X, POINTER_NEG_X,
    POINTER_SLIDER, POINTER_ZERO
};

static const Rectangle BUTTON_NEG = {74, 220, 296, 92};
static const Rectangle BUTTON_POS = {480, 220, 296, 92};
static const Rectangle FORCE_TRACK = {90, 392, 662, 16};
static const Rectangle ZERO_BUTTON = {308, 468, 234, 54};

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
    hmi_ui_text(message, area.x + (area.width-width)*0.5f,
                area.y + (area.height-size)*0.5f - 2.0f,
                size, color, bold);
}

static void draw_hold(Rectangle bounds, const char *text, bool held,
                      bool enabled, Vector2 mouse)
{
    const bool hover=enabled && CheckCollisionPointRec(mouse,bounds);
    const Color fill = !enabled ? HMI_C_FIELD :
        held ? (Color){46,108,172,255} :
        hover ? HMI_C_FIELD_HI : HMI_C_PANEL_2;
    DrawRectangleRounded(bounds,0.10f,12,fill);
    DrawRectangleLinesEx(bounds,held ? 2.0f : 1.0f,
                         held ? HMI_C_ACCENT : HMI_C_BORDER);
    centered(text,bounds,25.0f,enabled ? HMI_C_TEXT:HMI_C_FAINT,true);
}

static void draw_slider(float value, float max_value, bool dragging)
{
    Rectangle track=FORCE_TRACK;
    DrawRectangleRounded(track,0.5f,8,HMI_C_BORDER);
    Rectangle filled={track.x,track.y,track.width*value/max_value,track.height};
    DrawRectangleRounded(filled,0.5f,8,HMI_C_ACCENT);
    const float x=track.x+filled.width;
    DrawCircleV((Vector2){x,track.y+track.height*0.5f},12.0f,
                dragging ? HMI_C_TEXT : HMI_C_ACCENT);
    char label[64];
    snprintf(label,sizeof(label),"Force magnitude     %.2f N",value);
    hmi_ui_text(label,90,348,22,HMI_C_TEXT,true);
    hmi_ui_text("0 N",90,423,15,HMI_C_MUTED,false);
    snprintf(label,sizeof(label),"%.1f N",max_value);
    hmi_ui_text(label,702,423,15,HMI_C_MUTED,false);
}

int main(void)
{
    GpVirtualFtClient client;
    if (!gp_virtual_ft_client_open(&client)) {
        fprintf(stderr,"[VFT GUI] UDP sender socket could not open\n");
        return 1;
    }
    HmiProtocol status;
    if (!hmi_protocol_init(&status, HMI_VFT_STATUS_PORT)) {
        fprintf(stderr,"[VFT GUI] status UDP port 5014 unavailable\n");
        gp_virtual_ft_client_close(&client);
        return 1;
    }
    InitWindow(WINDOW_W,WINDOW_H,"Virtual F/T Sensor | Hand Guiding");
    if (!IsWindowReady()) {
        hmi_protocol_shutdown(&status);
        gp_virtual_ft_client_close(&client);
        return 1;
    }
    SetTargetFPS(120);
    hmi_ui_init();

    const float force_max_N=5.0f; /* software demonstration limit */
    float force_N=0.5f;
    int pointer=POINTER_NONE;
    bool suppress_until_release=false;
    bool transport_error=false;
    double next_send=0.0;

    while (!WindowShouldClose()) {
        hmi_protocol_poll_status(&status);
        const bool online=hmi_protocol_controller_online(&status,0.5);
        const HmiStatus *s=&status.status;
        const bool fault=online && (s->estop_active ||
            s->robot_state==ROBOT_STATE_FAULT ||
            s->robot_state==ROBOT_STATE_EMERGENCY_STOP);
        const bool guidance=online && !fault &&
            s->robot_state==ROBOT_STATE_TEACHING && s->guidance_active;
        const bool focus=IsWindowFocused();
        const bool allowed=guidance && focus;
        const Vector2 mouse=GetMousePosition();

        if (!focus || IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
            pointer=POINTER_NONE;
        if (focus && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            pointer=POINTER_NONE;
            if (CheckCollisionPointRec(mouse,FORCE_TRACK))
                pointer=POINTER_SLIDER;
            else if (CheckCollisionPointRec(mouse,ZERO_BUTTON))
                pointer=POINTER_ZERO;
            else if (allowed && CheckCollisionPointRec(mouse,BUTTON_POS))
                pointer=POINTER_POS_X;
            else if (allowed && CheckCollisionPointRec(mouse,BUTTON_NEG))
                pointer=POINTER_NEG_X;
            if (pointer==POINTER_ZERO) suppress_until_release=true;
        }
        const bool mouse_down=focus && IsMouseButtonDown(MOUSE_BUTTON_LEFT);
        if (pointer==POINTER_SLIDER && mouse_down) {
            const float fraction=clampf((mouse.x-FORCE_TRACK.x)/
                                      FORCE_TRACK.width,0.0f,1.0f);
            force_N=force_max_N*fraction;
        }
        const bool plus=focus && (IsKeyDown(KEY_RIGHT) ||
            (mouse_down && pointer==POINTER_POS_X &&
             CheckCollisionPointRec(mouse,BUTTON_POS)));
        const bool minus=focus && (IsKeyDown(KEY_LEFT) ||
            (mouse_down && pointer==POINTER_NEG_X &&
             CheckCollisionPointRec(mouse,BUTTON_NEG)));
        if (!plus && !minus && pointer!=POINTER_ZERO)
            suppress_until_release=false;
        const bool enabled=allowed && !suppress_until_release;
        const double requested_fx=gp_virtual_ft_x_request(
            plus,minus,enabled,force_N);
        const double wrench[6]={requested_fx,0,0,0,0,0};

        /* Status loss, focus loss, faults, and release transmit valid ZERO
         * samples; the actual C admittance damping governs velocity decay.
         * Stream rate ~50 Hz; never wait for network traffic in a frame. */
        const double now=GetTime();
        if (now>=next_send) {
            if (!gp_virtual_ft_client_send(&client,wrench))
                transport_error=true;
            else transport_error=false;
            next_send+=0.020;
            if (next_send<=now) next_send=now+0.020;
        }

        const char *status_text=transport_error ? "Transport error" :
            !online ? "Disconnected" : fault ? "Fault" :
            guidance ? "Guidance active" : "Ready / awaiting TEACHING";
        const Color status_color=transport_error || fault ? HMI_C_BAD :
            !online ? HMI_C_FAINT : guidance ? HMI_C_GOOD : HMI_C_WARN;

        BeginDrawing();
        ClearBackground(HMI_C_BG);
        hmi_ui_text("VIRTUAL FORCE / TORQUE",48,30,30,HMI_C_TEXT,true);
        hmi_ui_text("STAGE 4F2B  |  HOST SIMULATION ONLY",
                    49,76,15,HMI_C_MUTED,false);
        hmi_ui_panel((Rectangle){48,114,754,72},HMI_C_PANEL);
        DrawCircle(77,150,7,status_color);
        hmi_ui_text(status_text,96,135,22,HMI_C_TEXT,true);
        hmi_ui_text(online ? "Supervisor status via UDP 5014" :
                    "No recent Supervisor status on UDP 5014",
                    97,163,14,HMI_C_MUTED,false);

        draw_hold(BUTTON_NEG,"LEFT ARROW     -X",plus ? false : minus && enabled,
                  allowed,mouse);
        draw_hold(BUTTON_POS,"RIGHT ARROW     +X",plus && enabled,
                  allowed,mouse);
        hmi_ui_text("Hold an arrow key or press and hold a direction button",
                    95,324,15,HMI_C_MUTED,false);
        draw_slider(force_N,force_max_N,pointer==POINTER_SLIDER && mouse_down);
        hmi_ui_panel(ZERO_BUTTON,HMI_C_PANEL_2);
        centered("ZERO WRENCH",ZERO_BUTTON,17,HMI_C_TEXT,true);
        char wrench_label[140];
        snprintf(wrench_label,sizeof(wrench_label),
                "Requested:  Fx %+.2f N   Fy 0.00 N   Fz 0.00 N",
                requested_fx);
        hmi_ui_text(wrench_label,70,540,17,HMI_C_TEXT,false);
        hmi_ui_text("Base frame B  |  about TCP  |  Tx=Ty=Tz=0 N.m",
                    70,514,14,HMI_C_MUTED,false);
        EndDrawing();
    }

    /* Do not leave a held, nonzero input on a normal GUI exit. Lost GUI
     * communication after exit still drives the host watchdog to fault. */
    gp_virtual_ft_client_close(&client);
    hmi_protocol_shutdown(&status);
    hmi_ui_shutdown();
    CloseWindow();
    return 0;
}
