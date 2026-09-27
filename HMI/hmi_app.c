#include "hmi_app.h"

#include "hmi_protocol.h"
#include "hmi_theme.h"

#include "raylib.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define HMI_WINDOW_WIDTH   920
#define HMI_WINDOW_HEIGHT  720

static const char *robot_state_name(
    uint32_t state
)
{
    switch (state)
    {
        case 0U: return "BOOT";
        case 1U: return "HOMING";
        case 2U: return "IDLE";
        case 3U: return "TEACHING";
        case 4U: return "PATH VALIDATION";
        case 5U: return "APPROACH";
        default: return "UNKNOWN";
    }
}

static const char *program_name(
    uint32_t program
)
{
    switch (program)
    {
        case HMI_PROGRAM_LINE:
            return "LINE";

        case HMI_PROGRAM_ARC:
            return "CIRCULAR ARC";

        case HMI_PROGRAM_CIRCLE:
            return "CIRCLE";

        default:
            return "NONE";
    }
}

static void make_status_message(
    const HmiProtocol *protocol,
    bool online,
    char *buffer,
    size_t capacity
)
{
    if (
        buffer == NULL ||
        capacity == 0U
    )
    {
        return;
    }

    if (!online)
    {
        snprintf(
            buffer,
            capacity,
            "Controller offline - start the HMI state simulator."
        );

        return;
    }

    const HmiRobotStatus *status =
        &protocol->status;

    if (status->estop_active)
    {
        snprintf(
            buffer,
            capacity,
            "E-STOP ACTIVE - motion commands are inhibited."
        );

        return;
    }

    switch (status->robot_state)
    {
        case 0U:
            snprintf(
                buffer,
                capacity,
                "Booting: phase %u",
                status->boot_phase
            );
            break;

        case 1U:
            snprintf(
                buffer,
                capacity,
                "Homing robot: phase %u",
                status->homing_phase
            );
            break;

        case 2U:
            snprintf(
                buffer,
                capacity,
                "Ready. Select Line / Arc / Circle, then press START."
            );
            break;

        case 3U:
            if (status->guidance_active)
            {
                snprintf(
                    buffer,
                    capacity,
                    "Teaching: simulated hand-guiding move in progress..."
                );
            }
            else if (status->teaching_error != 0U)
            {
                snprintf(
                    buffer,
                    capacity,
                    "Teaching action rejected. Error code %u",
                    status->teaching_error
                );
            }
            else if (status->teaching_next_point > 0U)
            {
                snprintf(
                    buffer,
                    capacity,
                    "Teaching %s: move to P%u in the second window, then RECORD.",
                    program_name(status->selected_program),
                    status->teaching_next_point
                );
            }
            else
            {
                snprintf(
                    buffer,
                    capacity,
                    "Teaching active. Select the next segment type."
                );
            }
            break;

        case 4U:
            if (status->path_validation_result == 1U)
            {
                snprintf(
                    buffer,
                    capacity,
                    "Validating path... %.0f%%",
                    100.0F * status->path_validation_progress
                );
            }
            else if (status->path_validation_result == 2U)
            {
                snprintf(
                    buffer,
                    capacity,
                    "Path VALID. Press START / REPLAY to enter Approach."
                );
            }
            else
            {
                snprintf(
                    buffer,
                    capacity,
                    "Path validation finished. Result=%u Error=%u",
                    status->path_validation_result,
                    status->path_validation_error
                );
            }
            break;

        case 5U:
            if (status->approach_result == 1U)
            {
                snprintf(
                    buffer,
                    capacity,
                    "Approach in progress... %.0f%%",
                    100.0F * status->approach_progress
                );
            }
            else if (status->approach_result == 2U)
            {
                snprintf(
                    buffer,
                    capacity,
                    "Approach complete. Preview/Welding state is the next integration."
                );
            }
            else
            {
                snprintf(
                    buffer,
                    capacity,
                    "Approach result=%u Error=%u",
                    status->approach_result,
                    status->approach_error
                );
            }
            break;

        default:
            snprintf(
                buffer,
                capacity,
                "Controller state %u",
                status->robot_state
            );
            break;
    }
}

static bool panel_button(
    Rectangle bounds,
    const char *label,
    HmiButtonStyle style,
    bool enabled,
    Vector2 mouse
)
{
    return
        hmi_ui_button(
            bounds,
            label,
            style,
            enabled,
            mouse,
            false
        );
}

static void draw_section_title(
    const char *text,
    float x,
    float y
)
{
    hmi_ui_text(
        text,
        x,
        y,
        13.0F,
        HMI_C_MUTED,
        true
    );
}

int hmi_app_run(void)
{
    HmiProtocol protocol;

    if (
        !hmi_protocol_init(
            &protocol,
            HMI_PANEL_STATUS_PORT
        )
    )
    {
        return 1;
    }

    SetConfigFlags(
        FLAG_WINDOW_RESIZABLE |
        FLAG_MSAA_4X_HINT
    );

    InitWindow(
        HMI_WINDOW_WIDTH,
        HMI_WINDOW_HEIGHT,
        "Industrial Robot HMI Panel"
    );

    SetWindowMinSize(
        820,
        650
    );

    SetTargetFPS(60);

    hmi_ui_init();

    char local_message[192] =
        "Waiting for controller...";

    while (!WindowShouldClose())
    {
        hmi_protocol_poll_status(
            &protocol
        );

        const bool online =
            hmi_protocol_controller_online(
                &protocol,
                1.0
            );

        const HmiRobotStatus *status =
            &protocol.status;

        Vector2 mouse =
            GetMousePosition();

        BeginDrawing();

        ClearBackground(
            HMI_C_BG
        );

        /*
         * Outer bezel: this desktop window is intentionally laid out like
         * the actual operator panel rather than the old planner/test console.
         */
        Rectangle bezel =
        {
            18.0F,
            18.0F,
            (float)GetScreenWidth() - 36.0F,
            (float)GetScreenHeight() - 36.0F
        };

        DrawRectangleRounded(
            bezel,
            0.025F,
            12,
            (Color){33, 38, 46, 255}
        );

        DrawRectangleLinesEx(
            bezel,
            2.0F,
            HMI_C_BORDER_HI
        );

        hmi_ui_text(
            "6-DOF INDUSTRIAL ROBOT",
            42.0F,
            34.0F,
            24.0F,
            HMI_C_TEXT,
            true
        );

        hmi_ui_text(
            "Operator HMI",
            42.0F,
            62.0F,
            13.0F,
            HMI_C_MUTED,
            false
        );

        hmi_ui_status_dot(
            748.0F,
            52.0F,
            online,
            HMI_C_GOOD
        );

        hmi_ui_text(
            online ? "CONTROLLER ONLINE" : "CONTROLLER OFFLINE",
            766.0F,
            42.0F,
            12.0F,
            online ? HMI_C_GOOD : HMI_C_BAD,
            true
        );

        /* ---------------------------------------------------------------
         * ROBOT STATUS SCREEN
         * --------------------------------------------------------------- */

        Rectangle screen =
        {
            42.0F,
            94.0F,
            836.0F,
            112.0F
        };

        DrawRectangleRounded(
            screen,
            0.06F,
            12,
            (Color){8, 13, 17, 255}
        );

        DrawRectangleLinesEx(
            screen,
            2.0F,
            (Color){63, 93, 102, 255}
        );

        hmi_ui_text(
            "ROBOT STATUS  |  EN",
            58.0F,
            108.0F,
            12.0F,
            HMI_C_ACCENT,
            true
        );

        char automatic_message[256];

        make_status_message(
            &protocol,
            online,
            automatic_message,
            sizeof(automatic_message)
        );

        const char *screen_message =
            local_message[0] != '\0'
            ? local_message
            : automatic_message;

        /*
         * Once the controller is online, state-derived status takes priority
         * unless a button was just pressed this frame.
         */
        if (online)
        {
            screen_message =
                automatic_message;
        }

        hmi_ui_text(
            screen_message,
            58.0F,
            137.0F,
            18.0F,
            HMI_C_TEXT,
            true
        );

        char detail[256];

        snprintf(
            detail,
            sizeof(detail),
            "State: %s   Program: %s   WKC: %u/%u   TCP: [%.3f %.3f %.3f] m",
            online
                ? robot_state_name(status->robot_state)
                : "---",
            online
                ? program_name(status->selected_program)
                : "---",
            status->wkc,
            status->expected_wkc,
            status->actual_tcp_m[0],
            status->actual_tcp_m[1],
            status->actual_tcp_m[2]
        );

        hmi_ui_text(
            detail,
            58.0F,
            172.0F,
            12.0F,
            HMI_C_MUTED,
            false
        );

        /* ---------------------------------------------------------------
         * LEFT: PROGRAM / TEACHING
         * --------------------------------------------------------------- */

        Rectangle left =
        {
            42.0F,
            226.0F,
            500.0F,
            438.0F
        };

        hmi_ui_panel(
            left,
            HMI_C_PANEL
        );

        draw_section_title(
            "PROGRAM SELECTION",
            60.0F,
            244.0F
        );

        Rectangle lineButton =
            {60.0F, 270.0F, 138.0F, 48.0F};

        Rectangle arcButton =
            {210.0F, 270.0F, 138.0F, 48.0F};

        Rectangle circleButton =
            {360.0F, 270.0F, 138.0F, 48.0F};

        if (
            panel_button(
                lineButton,
                "LINE",
                status->selected_program == HMI_PROGRAM_LINE
                    ? HMI_BUTTON_PRIMARY
                    : HMI_BUTTON_NORMAL,
                online,
                mouse
            )
        )
        {
            (void)hmi_protocol_send_command(
                &protocol,
                HMI_CMD_SELECT_LINE
            );

            snprintf(
                local_message,
                sizeof(local_message),
                "Line selected."
            );
        }

        if (
            panel_button(
                arcButton,
                "CIRCULAR ARC",
                status->selected_program == HMI_PROGRAM_ARC
                    ? HMI_BUTTON_PRIMARY
                    : HMI_BUTTON_NORMAL,
                online,
                mouse
            )
        )
        {
            (void)hmi_protocol_send_command(
                &protocol,
                HMI_CMD_SELECT_ARC
            );

            snprintf(
                local_message,
                sizeof(local_message),
                "Circular arc selected."
            );
        }

        if (
            panel_button(
                circleButton,
                "CIRCLE",
                status->selected_program == HMI_PROGRAM_CIRCLE
                    ? HMI_BUTTON_PRIMARY
                    : HMI_BUTTON_NORMAL,
                online,
                mouse
            )
        )
        {
            (void)hmi_protocol_send_command(
                &protocol,
                HMI_CMD_SELECT_CIRCLE
            );

            snprintf(
                local_message,
                sizeof(local_message),
                "Circle selected."
            );
        }

        draw_section_title(
            "TEACHING",
            60.0F,
            342.0F
        );

        const bool record_enabled =
            online &&
            status->record_allowed &&
            !status->guidance_active;

        if (
            panel_button(
                (Rectangle){60.0F, 368.0F, 205.0F, 56.0F},
                status->teaching_next_point > 0U
                    ? "RECORD POINT"
                    : "RECORD",
                HMI_BUTTON_PRIMARY,
                record_enabled,
                mouse
            )
        )
        {
            (void)hmi_protocol_send_command(
                &protocol,
                HMI_CMD_RECORD
            );

            snprintf(
                local_message,
                sizeof(local_message),
                "Record command sent."
            );
        }

        if (
            panel_button(
                (Rectangle){277.0F, 368.0F, 221.0F, 56.0F},
                "PREVIEW / VALIDATE",
                HMI_BUTTON_NORMAL,
                online && status->validate_allowed,
                mouse
            )
        )
        {
            (void)hmi_protocol_send_command(
                &protocol,
                HMI_CMD_VALIDATE_PREVIEW
            );

            snprintf(
                local_message,
                sizeof(local_message),
                "Validation requested."
            );
        }

        draw_section_title(
            "SET SPEED",
            60.0F,
            450.0F
        );

        if (
            panel_button(
                (Rectangle){60.0F, 476.0F, 92.0F, 48.0F},
                "-",
                HMI_BUTTON_NORMAL,
                online && status->robot_state == 3U,
                mouse
            )
        )
        {
            (void)hmi_protocol_send_command(
                &protocol,
                HMI_CMD_SPEED_DOWN
            );
        }

        if (
            panel_button(
                (Rectangle){162.0F, 476.0F, 92.0F, 48.0F},
                "+",
                HMI_BUTTON_NORMAL,
                online && status->robot_state == 3U,
                mouse
            )
        )
        {
            (void)hmi_protocol_send_command(
                &protocol,
                HMI_CMD_SPEED_UP
            );
        }

        if (
            panel_button(
                (Rectangle){266.0F, 476.0F, 232.0F, 48.0F},
                "DEFAULT MODE",
                HMI_BUTTON_NORMAL,
                online && status->robot_state == 3U,
                mouse
            )
        )
        {
            (void)hmi_protocol_send_command(
                &protocol,
                HMI_CMD_SPEED_DEFAULT
            );
        }

        char speedText[96];

        snprintf(
            speedText,
            sizeof(speedText),
            "Teaching speed: %.3f m/s",
            status->teaching_speed_mps
        );

        hmi_ui_text(
            speedText,
            60.0F,
            536.0F,
            13.0F,
            HMI_C_MUTED,
            false
        );

        char pointText[128];

        snprintf(
            pointText,
            sizeof(pointText),
            "Recorded: %u   Next point: %s",
            status->recorded_count,
            status->teaching_next_point > 0U
                ? "shown on screen"
                : "-"
        );

        hmi_ui_text(
            pointText,
            60.0F,
            562.0F,
            13.0F,
            HMI_C_MUTED,
            false
        );

        hmi_ui_text(
            "Use the second window to simulate hand-guiding.",
            60.0F,
            606.0F,
            12.0F,
            HMI_C_FAINT,
            false
        );

        /* ---------------------------------------------------------------
         * RIGHT: OPERATIONAL / E-STOP
         * --------------------------------------------------------------- */

        Rectangle right =
        {
            560.0F,
            226.0F,
            318.0F,
            438.0F
        };

        hmi_ui_panel(
            right,
            HMI_C_PANEL
        );

        draw_section_title(
            "EMERGENCY",
            580.0F,
            244.0F
        );

        if (
            panel_button(
                (Rectangle){580.0F, 270.0F, 278.0F, 82.0F},
                status->estop_active
                    ? "E-STOP ACTIVE"
                    : "E-STOP",
                HMI_BUTTON_DANGER,
                online,
                mouse
            )
        )
        {
            (void)hmi_protocol_send_command(
                &protocol,
                HMI_CMD_ESTOP_TOGGLE
            );
        }

        draw_section_title(
            "OPERATIONAL",
            580.0F,
            380.0F
        );

        if (
            panel_button(
                (Rectangle){580.0F, 406.0F, 278.0F, 54.0F},
                "START / REPLAY",
                HMI_BUTTON_PRIMARY,
                online && !status->estop_active,
                mouse
            )
        )
        {
            (void)hmi_protocol_send_command(
                &protocol,
                HMI_CMD_START_REPLAY
            );
        }

        if (
            panel_button(
                (Rectangle){580.0F, 472.0F, 134.0F, 50.0F},
                status->paused ? "RESUME" : "PAUSE",
                HMI_BUTTON_NORMAL,
                online,
                mouse
            )
        )
        {
            (void)hmi_protocol_send_command(
                &protocol,
                HMI_CMD_PAUSE_TOGGLE
            );
        }

        if (
            panel_button(
                (Rectangle){724.0F, 472.0F, 134.0F, 50.0F},
                "RESET",
                HMI_BUTTON_NORMAL,
                online,
                mouse
            )
        )
        {
            (void)hmi_protocol_send_command(
                &protocol,
                HMI_CMD_RESET
            );
        }

        if (
            panel_button(
                (Rectangle){580.0F, 534.0F, 278.0F, 50.0F},
                "HOME",
                HMI_BUTTON_NORMAL,
                online,
                mouse
            )
        )
        {
            (void)hmi_protocol_send_command(
                &protocol,
                HMI_CMD_HOME
            );
        }

        hmi_ui_text(
            "Main HMI only records.",
            580.0F,
            608.0F,
            12.0F,
            HMI_C_FAINT,
            false
        );

        hmi_ui_text(
            "No A/B/C batch entry exists here.",
            580.0F,
            628.0F,
            12.0F,
            HMI_C_FAINT,
            false
        );

        EndDrawing();
    }

    hmi_ui_shutdown();

    CloseWindow();

    hmi_protocol_shutdown(
        &protocol
    );

    return 0;
}
