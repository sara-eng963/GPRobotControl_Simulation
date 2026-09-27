#include "teaching_sim_app.h"

#include "hmi_protocol.h"
#include "hmi_theme.h"

#include "raylib.h"
#include "raymath.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define TEACH_WINDOW_WIDTH   980
#define TEACH_WINDOW_HEIGHT  720

typedef enum
{
    TEACH_VIEW_PERSPECTIVE = 0,
    TEACH_VIEW_TOP,
    TEACH_VIEW_FRONT,
    TEACH_VIEW_SIDE

} TeachView;

static Vector3 robot_to_render(
    float x,
    float y,
    float z
)
{
    /*
     * Same convention used by the old HMI-Mock editor:
     *
     * robot X -> render +X
     * robot Y -> render -Z
     * robot Z -> render +Y
     */
    return
        (Vector3)
        {
            x,
            z,
            -y
        };
}

static void render_to_robot(
    Vector3 p,
    float *x,
    float *y,
    float *z
)
{
    if (x != NULL) *x = p.x;
    if (y != NULL) *y = -p.z;
    if (z != NULL) *z = p.y;
}

static Camera3D camera_for_view(
    TeachView view
)
{
    Camera3D camera = {0};

    camera.target =
        (Vector3){0.0F, 0.35F, 0.0F};

    camera.up =
        (Vector3){0.0F, 1.0F, 0.0F};

    camera.fovy =
        48.0F;

    camera.projection =
        CAMERA_PERSPECTIVE;

    switch (view)
    {
        case TEACH_VIEW_TOP:
            camera.position =
                (Vector3){0.0F, 2.2F, 0.0F};

            camera.up =
                (Vector3){0.0F, 0.0F, -1.0F};

            camera.fovy =
                2.0F;

            camera.projection =
                CAMERA_ORTHOGRAPHIC;

            break;

        case TEACH_VIEW_FRONT:
            camera.position =
                (Vector3){0.0F, 0.35F, 2.2F};

            camera.fovy =
                2.0F;

            camera.projection =
                CAMERA_ORTHOGRAPHIC;

            break;

        case TEACH_VIEW_SIDE:
            camera.position =
                (Vector3){2.2F, 0.35F, 0.0F};

            camera.fovy =
                2.0F;

            camera.projection =
                CAMERA_ORTHOGRAPHIC;

            break;

        case TEACH_VIEW_PERSPECTIVE:
        default:
            camera.position =
                (Vector3){1.55F, 1.15F, 1.55F};

            break;
    }

    return camera;
}

static bool ray_plane_intersection(
    Ray ray,
    Vector3 normal,
    float distance,
    Vector3 *point
)
{
    const float denominator =
        Vector3DotProduct(
            normal,
            ray.direction
        );

    if (fabsf(denominator) < 1.0e-6F)
    {
        return false;
    }

    const float t =
        (
            distance -
            Vector3DotProduct(
                normal,
                ray.position
            )
        )
        /
        denominator;

    if (t < 0.0F)
    {
        return false;
    }

    if (point != NULL)
    {
        *point =
            Vector3Add(
                ray.position,
                Vector3Scale(
                    ray.direction,
                    t
                )
            );
    }

    return true;
}

static float clamp_local(
    float value,
    float minimum,
    float maximum
)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void send_target(
    HmiProtocol *protocol,
    float target[3]
)
{
    target[0] =
        clamp_local(
            target[0],
            -0.90F,
            0.90F
        );

    target[1] =
        clamp_local(
            target[1],
            -0.90F,
            0.90F
        );

    target[2] =
        clamp_local(
            target[2],
            -0.20F,
            1.20F
        );

    (void)hmi_protocol_send_guidance_pose(
        protocol,
        target[0],
        target[1],
        target[2]
    );
}

static void draw_scene(
    const HmiRobotStatus *status,
    const float target[3],
    Camera3D camera
)
{
    DrawGrid(
        20,
        0.10F
    );

    DrawLine3D(
        (Vector3){0.0F, 0.0F, 0.0F},
        (Vector3){0.65F, 0.0F, 0.0F},
        RED
    );

    DrawLine3D(
        (Vector3){0.0F, 0.0F, 0.0F},
        (Vector3){0.0F, 0.65F, 0.0F},
        GREEN
    );

    DrawLine3D(
        (Vector3){0.0F, 0.0F, 0.0F},
        (Vector3){0.0F, 0.0F, -0.65F},
        BLUE
    );

    const Vector3 actual =
        robot_to_render(
            status->actual_tcp_m[0],
            status->actual_tcp_m[1],
            status->actual_tcp_m[2]
        );

    const Vector3 desired =
        robot_to_render(
            target[0],
            target[1],
            target[2]
        );

    DrawSphere(
        actual,
        0.035F,
        HMI_C_GOOD
    );

    DrawSphereWires(
        desired,
        0.050F,
        12,
        12,
        HMI_C_ACCENT
    );

    DrawLine3D(
        actual,
        desired,
        HMI_C_MUTED
    );

    for (
        uint32_t i = 0U;
        i < status->recorded_count &&
        i < HMI_MAX_RECORDED_POINTS;
        ++i
    )
    {
        const Vector3 recorded =
            robot_to_render(
                status->recorded_tcp_m[i][0],
                status->recorded_tcp_m[i][1],
                status->recorded_tcp_m[i][2]
            );

        DrawSphere(
            recorded,
            0.042F,
            HMI_C_WARN
        );

        if (i > 0U)
        {
            const Vector3 previous =
                robot_to_render(
                    status->recorded_tcp_m[i - 1U][0],
                    status->recorded_tcp_m[i - 1U][1],
                    status->recorded_tcp_m[i - 1U][2]
                );

            DrawLine3D(
                previous,
                recorded,
                HMI_C_WARN
            );
        }
    }

    (void)camera;
}

int teaching_sim_app_run(void)
{
    HmiProtocol protocol;

    if (
        !hmi_protocol_init(
            &protocol,
            HMI_TEACH_STATUS_PORT
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
        TEACH_WINDOW_WIDTH,
        TEACH_WINDOW_HEIGHT,
        "Teaching Simulation - Virtual Hand Guiding"
    );

    SetWindowMinSize(
        820,
        620
    );

    SetTargetFPS(60);

    hmi_ui_init();

    TeachView view =
        TEACH_VIEW_PERSPECTIVE;

    float target[3] =
    {
        0.35F,
        0.0F,
        0.35F
    };

    bool target_initialized =
        false;

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

        if (
            online &&
            !target_initialized
        )
        {
            memcpy(
                target,
                status->actual_tcp_m,
                sizeof(target)
            );

            target_initialized =
                true;
        }

        Vector2 mouse =
            GetMousePosition();

        BeginDrawing();

        ClearBackground(
            HMI_C_BG
        );

        Rectangle viewport =
        {
            28.0F,
            118.0F,
            680.0F,
            540.0F
        };

        Rectangle perspectiveButton =
            {28.0F, 62.0F, 128.0F, 38.0F};

        Rectangle topButton =
            {166.0F, 62.0F, 92.0F, 38.0F};

        Rectangle frontButton =
            {268.0F, 62.0F, 92.0F, 38.0F};

        Rectangle sideButton =
            {370.0F, 62.0F, 92.0F, 38.0F};

        if (
            hmi_ui_button(
                perspectiveButton,
                "PERSPECTIVE",
                view == TEACH_VIEW_PERSPECTIVE
                    ? HMI_BUTTON_PRIMARY
                    : HMI_BUTTON_NORMAL,
                true,
                mouse,
                false
            )
        )
        {
            view =
                TEACH_VIEW_PERSPECTIVE;
        }

        if (
            hmi_ui_button(
                topButton,
                "TOP",
                view == TEACH_VIEW_TOP
                    ? HMI_BUTTON_PRIMARY
                    : HMI_BUTTON_NORMAL,
                true,
                mouse,
                false
            )
        )
        {
            view =
                TEACH_VIEW_TOP;
        }

        if (
            hmi_ui_button(
                frontButton,
                "FRONT",
                view == TEACH_VIEW_FRONT
                    ? HMI_BUTTON_PRIMARY
                    : HMI_BUTTON_NORMAL,
                true,
                mouse,
                false
            )
        )
        {
            view =
                TEACH_VIEW_FRONT;
        }

        if (
            hmi_ui_button(
                sideButton,
                "SIDE",
                view == TEACH_VIEW_SIDE
                    ? HMI_BUTTON_PRIMARY
                    : HMI_BUTTON_NORMAL,
                true,
                mouse,
                false
            )
        )
        {
            view =
                TEACH_VIEW_SIDE;
        }

        Camera3D camera =
            camera_for_view(
                view
            );

        const bool teaching_active =
            online &&
            status->robot_state == 3U &&
            !status->estop_active &&
            !status->paused;

        /*
         * In orthographic views a click chooses ONE current hand-guided pose.
         * There are deliberately no A/B/C tabs. The main HMI's RECORD button
         * decides when the current real/simulated robot pose is stored.
         */
        if (
            teaching_active &&
            view != TEACH_VIEW_PERSPECTIVE &&
            IsMouseButtonPressed(MOUSE_BUTTON_LEFT) &&
            CheckCollisionPointRec(
                mouse,
                viewport
            )
        )
        {
            Ray ray =
                GetScreenToWorldRay(
                    mouse,
                    camera
                );

            Vector3 current =
                robot_to_render(
                    target[0],
                    target[1],
                    target[2]
                );

            Vector3 hit =
                current;

            bool hit_ok =
                false;

            if (view == TEACH_VIEW_TOP)
            {
                hit_ok =
                    ray_plane_intersection(
                        ray,
                        (Vector3){0.0F, 1.0F, 0.0F},
                        current.y,
                        &hit
                    );
            }
            else if (view == TEACH_VIEW_FRONT)
            {
                hit_ok =
                    ray_plane_intersection(
                        ray,
                        (Vector3){0.0F, 0.0F, 1.0F},
                        current.z,
                        &hit
                    );
            }
            else if (view == TEACH_VIEW_SIDE)
            {
                hit_ok =
                    ray_plane_intersection(
                        ray,
                        (Vector3){1.0F, 0.0F, 0.0F},
                        current.x,
                        &hit
                    );
            }

            if (hit_ok)
            {
                render_to_robot(
                    hit,
                    &target[0],
                    &target[1],
                    &target[2]
                );

                send_target(
                    &protocol,
                    target
                );
            }
        }

        hmi_ui_text(
            "SIMULATED HAND GUIDING",
            28.0F,
            22.0F,
            24.0F,
            HMI_C_TEXT,
            true
        );

        hmi_ui_text(
            "Move ONE current TCP pose here. RECORD remains on the real HMI window.",
            28.0F,
            48.0F,
            12.5F,
            HMI_C_MUTED,
            false
        );

        DrawRectangleRounded(
            viewport,
            0.02F,
            8,
            (Color){12, 16, 22, 255}
        );

        BeginScissorMode(
            (int)viewport.x,
            (int)viewport.y,
            (int)viewport.width,
            (int)viewport.height
        );

        BeginMode3D(
            camera
        );

        draw_scene(
            status,
            target,
            camera
        );

        EndMode3D();

        EndScissorMode();

        Rectangle monitor =
        {
            728.0F,
            118.0F,
            224.0F,
            540.0F
        };

        hmi_ui_panel(
            monitor,
            HMI_C_PANEL
        );

        hmi_ui_text(
            "LIVE MONITOR",
            746.0F,
            136.0F,
            13.0F,
            HMI_C_MUTED,
            true
        );

        hmi_ui_status_dot(
            748.0F,
            170.0F,
            online,
            HMI_C_GOOD
        );

        hmi_ui_text(
            online ? "Controller online" : "Controller offline",
            764.0F,
            160.0F,
            12.0F,
            online ? HMI_C_GOOD : HMI_C_BAD,
            true
        );

        char line[160];

        snprintf(
            line,
            sizeof(line),
            "Global state: %u",
            status->robot_state
        );

        hmi_ui_text(
            line,
            746.0F,
            196.0F,
            12.0F,
            HMI_C_TEXT,
            false
        );

        snprintf(
            line,
            sizeof(line),
            "Next point: P%u",
            status->teaching_next_point
        );

        hmi_ui_text(
            line,
            746.0F,
            222.0F,
            12.0F,
            HMI_C_TEXT,
            false
        );

        snprintf(
            line,
            sizeof(line),
            "Recorded: %u",
            status->recorded_count
        );

        hmi_ui_text(
            line,
            746.0F,
            248.0F,
            12.0F,
            HMI_C_TEXT,
            false
        );

        snprintf(
            line,
            sizeof(line),
            "Target X %.3f",
            target[0]
        );

        hmi_ui_text(
            line,
            746.0F,
            294.0F,
            12.0F,
            HMI_C_ACCENT,
            true
        );

        snprintf(
            line,
            sizeof(line),
            "Target Y %.3f",
            target[1]
        );

        hmi_ui_text(
            line,
            746.0F,
            320.0F,
            12.0F,
            HMI_C_ACCENT,
            true
        );

        snprintf(
            line,
            sizeof(line),
            "Target Z %.3f",
            target[2]
        );

        hmi_ui_text(
            line,
            746.0F,
            346.0F,
            12.0F,
            HMI_C_ACCENT,
            true
        );

        snprintf(
            line,
            sizeof(line),
            "Actual X %.3f",
            status->actual_tcp_m[0]
        );

        hmi_ui_text(
            line,
            746.0F,
            390.0F,
            12.0F,
            HMI_C_GOOD,
            false
        );

        snprintf(
            line,
            sizeof(line),
            "Actual Y %.3f",
            status->actual_tcp_m[1]
        );

        hmi_ui_text(
            line,
            746.0F,
            416.0F,
            12.0F,
            HMI_C_GOOD,
            false
        );

        snprintf(
            line,
            sizeof(line),
            "Actual Z %.3f",
            status->actual_tcp_m[2]
        );

        hmi_ui_text(
            line,
            746.0F,
            442.0F,
            12.0F,
            HMI_C_GOOD,
            false
        );

        if (status->guidance_active)
        {
            hmi_ui_text(
                "MOVING...",
                746.0F,
                482.0F,
                14.0F,
                HMI_C_WARN,
                true
            );
        }
        else if (status->guidance_error != 0U)
        {
            snprintf(
                line,
                sizeof(line),
                "Guidance error: %u",
                status->guidance_error
            );

            hmi_ui_text(
                line,
                746.0F,
                482.0F,
                12.0F,
                HMI_C_BAD,
                true
            );
        }
        else
        {
            hmi_ui_text(
                "SETTLED",
                746.0F,
                482.0F,
                14.0F,
                HMI_C_GOOD,
                true
            );
        }

        hmi_ui_text(
            teaching_active
                ? "TOP/FRONT/SIDE: click to guide"
                : "Press START on main HMI",
            746.0F,
            532.0F,
            11.5F,
            HMI_C_MUTED,
            false
        );

        hmi_ui_text(
            "Yellow = recorded point",
            746.0F,
            558.0F,
            11.0F,
            HMI_C_FAINT,
            false
        );

        hmi_ui_text(
            "Green = actual TCP",
            746.0F,
            580.0F,
            11.0F,
            HMI_C_FAINT,
            false
        );

        hmi_ui_text(
            "Blue wire = guide target",
            746.0F,
            602.0F,
            11.0F,
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
