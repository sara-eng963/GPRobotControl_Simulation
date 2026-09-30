#include "teaching_sim_app.h"

#include "hmi_protocol.h"
#include "hmi_theme.h"

#include "raylib.h"
#include "raymath.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEACH_WINDOW_WIDTH   980
#define TEACH_WINDOW_HEIGHT  720

#define GIZMO_SHAFT_START_M  0.060F
#define GIZMO_SHAFT_END_M    0.180F
#define GIZMO_TIP_END_M      0.240F
#define GIZMO_HIT_RADIUS_PX  16.0F
#define TARGET_HIT_RADIUS_PX 48.0F


typedef enum
{
    TEACH_VIEW_PERSPECTIVE = 0,
    TEACH_VIEW_TOP,
    TEACH_VIEW_FRONT,
    TEACH_VIEW_SIDE

} TeachView;


typedef enum
{
    TEACH_DRAG_NONE = 0,
    TEACH_DRAG_FREE,
    TEACH_DRAG_X,
    TEACH_DRAG_Y,
    TEACH_DRAG_Z

} TeachDragMode;


static Vector3 robot_to_render(
    float x,
    float y,
    float z
)
{
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


static Vector3 drag_axis_render(
    TeachDragMode mode
)
{
    switch (mode)
    {
        case TEACH_DRAG_X:
            return (Vector3){1.0F, 0.0F, 0.0F};

        case TEACH_DRAG_Y:
            return (Vector3){0.0F, 0.0F, -1.0F};

        case TEACH_DRAG_Z:
            return (Vector3){0.0F, 1.0F, 0.0F};

        default:
            return (Vector3){0.0F, 0.0F, 0.0F};
    }
}


static int drag_axis_robot_index(
    TeachDragMode mode
)
{
    switch (mode)
    {
        case TEACH_DRAG_X: return 0;
        case TEACH_DRAG_Y: return 1;
        case TEACH_DRAG_Z: return 2;
        default: return -1;
    }
}


static Color drag_axis_color(
    TeachDragMode mode
)
{
    switch (mode)
    {
        case TEACH_DRAG_X: return RED;
        case TEACH_DRAG_Y: return GREEN;
        case TEACH_DRAG_Z: return BLUE;
        default: return HMI_C_MUTED;
    }
}


static Color brighten_color(
    Color color
)
{
    const int amount = 70;

    return
        (Color)
        {
            (unsigned char)((color.r + amount > 255) ? 255 : color.r + amount),
            (unsigned char)((color.g + amount > 255) ? 255 : color.g + amount),
            (unsigned char)((color.b + amount > 255) ? 255 : color.b + amount),
            color.a
        };
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
        ) /
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


static Ray viewport_mouse_ray(
    Vector2 mouse,
    Rectangle viewport,
    Camera3D camera
)
{
    const Vector2 local =
    {
        mouse.x - viewport.x,
        mouse.y - viewport.y
    };

    return
        GetScreenToWorldRayEx(
            local,
            camera,
            (int)viewport.width,
            (int)viewport.height
        );
}


static Vector2 viewport_world_to_screen(
    Vector3 point,
    Rectangle viewport,
    Camera3D camera
)
{
    Vector2 local =
        GetWorldToScreenEx(
            point,
            camera,
            (int)viewport.width,
            (int)viewport.height
        );

    local.x += viewport.x;
    local.y += viewport.y;

    return local;
}


static Vector3 drag_plane_normal_for_view(
    TeachView view,
    Camera3D camera
)
{
    if (view == TEACH_VIEW_TOP)
    {
        return (Vector3){0.0F, 1.0F, 0.0F};
    }

    if (view == TEACH_VIEW_FRONT)
    {
        return (Vector3){0.0F, 0.0F, 1.0F};
    }

    if (view == TEACH_VIEW_SIDE)
    {
        return (Vector3){1.0F, 0.0F, 0.0F};
    }

    return
        Vector3Normalize(
            Vector3Subtract(
                camera.target,
                camera.position
            )
        );
}


static float point_segment_distance(
    Vector2 point,
    Vector2 start,
    Vector2 end
)
{
    const float dx =
        end.x - start.x;

    const float dy =
        end.y - start.y;

    const float length_squared =
        dx * dx + dy * dy;

    if (length_squared < 1.0e-6F)
    {
        const float px =
            point.x - start.x;

        const float py =
            point.y - start.y;

        return sqrtf(px * px + py * py);
    }

    float t =
        (
            (point.x - start.x) * dx +
            (point.y - start.y) * dy
        ) /
        length_squared;

    t = clamp_local(t, 0.0F, 1.0F);

    const float closest_x =
        start.x + t * dx;

    const float closest_y =
        start.y + t * dy;

    const float px =
        point.x - closest_x;

    const float py =
        point.y - closest_y;

    return sqrtf(px * px + py * py);
}


static TeachDragMode pick_gizmo_axis(
    Vector2 mouse,
    Rectangle viewport,
    Camera3D camera,
    Vector3 origin
)
{
    const TeachDragMode axes[] =
    {
        TEACH_DRAG_X,
        TEACH_DRAG_Y,
        TEACH_DRAG_Z
    };

    TeachDragMode best_axis =
        TEACH_DRAG_NONE;

    float best_distance =
        GIZMO_HIT_RADIUS_PX;

    for (size_t i = 0U;
         i < sizeof(axes) / sizeof(axes[0]);
         ++i)
    {
        const Vector3 direction =
            drag_axis_render(
                axes[i]
            );

        const Vector3 shaft_start =
            Vector3Add(
                origin,
                Vector3Scale(
                    direction,
                    GIZMO_SHAFT_START_M
                )
            );

        const Vector3 tip_end =
            Vector3Add(
                origin,
                Vector3Scale(
                    direction,
                    GIZMO_TIP_END_M
                )
            );

        const Vector2 start_screen =
            viewport_world_to_screen(
                shaft_start,
                viewport,
                camera
            );

        const Vector2 end_screen =
            viewport_world_to_screen(
                tip_end,
                viewport,
                camera
            );

        const float screen_length =
            Vector2Distance(
                start_screen,
                end_screen
            );

        if (screen_length < 18.0F)
        {
            continue;
        }

        const float distance =
            point_segment_distance(
                mouse,
                start_screen,
                end_screen
            );

        if (distance < best_distance)
        {
            best_distance = distance;
            best_axis = axes[i];
        }
    }

    return best_axis;
}


static bool begin_axis_drag(
    TeachDragMode mode,
    Rectangle viewport,
    Camera3D camera,
    Vector3 origin,
    Vector2 *screen_direction,
    float *robot_units_per_pixel
)
{
    const Vector3 direction =
        drag_axis_render(
            mode
        );

    const Vector3 end =
        Vector3Add(
            origin,
            Vector3Scale(
                direction,
                GIZMO_TIP_END_M
            )
        );

    const Vector2 start_screen =
        viewport_world_to_screen(
            origin,
            viewport,
            camera
        );

    const Vector2 end_screen =
        viewport_world_to_screen(
            end,
            viewport,
            camera
        );

    const float dx =
        end_screen.x - start_screen.x;

    const float dy =
        end_screen.y - start_screen.y;

    const float screen_length =
        sqrtf(dx * dx + dy * dy);

    if (
        screen_length < 18.0F ||
        screen_direction == NULL ||
        robot_units_per_pixel == NULL
    )
    {
        return false;
    }

    screen_direction->x =
        dx / screen_length;

    screen_direction->y =
        dy / screen_length;

    *robot_units_per_pixel =
        GIZMO_TIP_END_M /
        screen_length;

    return true;
}


static void send_target(
    HmiProtocol *protocol,
    float target[3]
)
{
    target[0] =
        clamp_local(target[0], -0.90F, 0.90F);

    target[1] =
        clamp_local(target[1], -0.90F, 0.90F);

    target[2] =
        clamp_local(target[2], -0.20F, 1.20F);

    (void)hmi_protocol_send_guidance_pose(
        protocol,
        target[0],
        target[1],
        target[2]
    );
}


static void draw_gizmo_arrow(
    Vector3 origin,
    TeachDragMode mode,
    bool highlighted
)
{
    const Vector3 direction =
        drag_axis_render(
            mode
        );

    Color color =
        drag_axis_color(
            mode
        );

    if (highlighted)
    {
        color =
            brighten_color(
                color
            );
    }

    const float shaft_radius =
        highlighted ? 0.009F : 0.006F;

    const Vector3 shaft_start =
        Vector3Add(
            origin,
            Vector3Scale(
                direction,
                GIZMO_SHAFT_START_M
            )
        );

    const Vector3 shaft_end =
        Vector3Add(
            origin,
            Vector3Scale(
                direction,
                GIZMO_SHAFT_END_M
            )
        );

    const Vector3 tip_end =
        Vector3Add(
            origin,
            Vector3Scale(
                direction,
                GIZMO_TIP_END_M
            )
        );

    DrawCylinderEx(
        shaft_start,
        shaft_end,
        shaft_radius,
        shaft_radius,
        12,
        color
    );

    DrawCylinderEx(
        shaft_end,
        tip_end,
        highlighted ? 0.026F : 0.022F,
        0.0F,
        12,
        color
    );
}


static void draw_scene(
    const HmiStatus *status,
    const float target[3],
    TeachDragMode hovered_axis,
    TeachDragMode active_drag
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
        BLUE
    );

    DrawLine3D(
        (Vector3){0.0F, 0.0F, 0.0F},
        (Vector3){0.0F, 0.0F, -0.65F},
        GREEN
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

    draw_gizmo_arrow(
        desired,
        TEACH_DRAG_X,
        hovered_axis == TEACH_DRAG_X ||
        active_drag == TEACH_DRAG_X
    );

    draw_gizmo_arrow(
        desired,
        TEACH_DRAG_Y,
        hovered_axis == TEACH_DRAG_Y ||
        active_drag == TEACH_DRAG_Y
    );

    draw_gizmo_arrow(
        desired,
        TEACH_DRAG_Z,
        hovered_axis == TEACH_DRAG_Z ||
        active_drag == TEACH_DRAG_Z
    );

    for (uint32_t i = 0U;
         i < status->recorded_count &&
         i < HMI_MAX_RECORDED_POINTS;
         ++i)
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
}


static void draw_gizmo_labels(
    Vector3 target_render,
    Rectangle viewport,
    Camera3D camera,
    TeachDragMode hovered_axis,
    TeachDragMode active_drag
)
{
    const TeachDragMode axes[] =
    {
        TEACH_DRAG_X,
        TEACH_DRAG_Y,
        TEACH_DRAG_Z
    };

    const char *labels[] =
    {
        "X",
        "Y",
        "Z"
    };

    for (size_t i = 0U;
         i < sizeof(axes) / sizeof(axes[0]);
         ++i)
    {
        const Vector3 endpoint =
            Vector3Add(
                target_render,
                Vector3Scale(
                    drag_axis_render(axes[i]),
                    GIZMO_TIP_END_M + 0.025F
                )
            );

        const Vector2 screen =
            viewport_world_to_screen(
                endpoint,
                viewport,
                camera
            );

        Color color =
            drag_axis_color(
                axes[i]
            );

        if (
            hovered_axis == axes[i] ||
            active_drag == axes[i]
        )
        {
            color = brighten_color(color);
        }

        DrawText(
            labels[i],
            (int)screen.x - 5,
            (int)screen.y - 9,
            18,
            color
        );
    }
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

    TeachDragMode active_drag =
        TEACH_DRAG_NONE;

    Vector3 drag_plane_normal =
        (Vector3){0.0F, 1.0F, 0.0F};

    float drag_plane_distance =
        0.0F;

    Vector2 drag_start_mouse =
        {0.0F, 0.0F};

    Vector2 drag_axis_screen_direction =
        {0.0F, 0.0F};

    float drag_robot_units_per_pixel =
        0.0F;

    float drag_start_target[3] =
        {0.0F, 0.0F, 0.0F};

    double last_guidance_send_time =
        -1.0;

    const double guidance_send_period_s =
        0.05;

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

        const HmiStatus *status =
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

        const Vector2 mouse =
            GetMousePosition();

        BeginDrawing();

        ClearBackground(
            HMI_C_BG
        );

        const Rectangle viewport =
        {
            28.0F,
            118.0F,
            680.0F,
            540.0F
        };

        const Rectangle perspectiveButton =
            {28.0F, 62.0F, 128.0F, 38.0F};

        const Rectangle topButton =
            {166.0F, 62.0F, 92.0F, 38.0F};

        const Rectangle frontButton =
            {268.0F, 62.0F, 92.0F, 38.0F};

        const Rectangle sideButton =
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
            view = TEACH_VIEW_PERSPECTIVE;
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
            view = TEACH_VIEW_TOP;
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
            view = TEACH_VIEW_FRONT;
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
            view = TEACH_VIEW_SIDE;
        }

        const Camera3D camera =
            camera_for_view(
                view
            );

        const bool teaching_active =
            online &&
            status->robot_state == 3U &&
            !status->estop_active &&
            !status->paused;

        const Vector3 target_render =
            robot_to_render(
                target[0],
                target[1],
                target[2]
            );

        const Vector2 target_screen =
            viewport_world_to_screen(
                target_render,
                viewport,
                camera
            );

        TeachDragMode hovered_axis =
            TEACH_DRAG_NONE;

        if (
            teaching_active &&
            active_drag == TEACH_DRAG_NONE &&
            CheckCollisionPointRec(mouse, viewport)
        )
        {
            hovered_axis =
                pick_gizmo_axis(
                    mouse,
                    viewport,
                    camera,
                    target_render
                );
        }

        const bool mouse_near_target =
            Vector2Distance(
                mouse,
                target_screen
            ) <= TARGET_HIT_RADIUS_PX;

        if (
            IsMouseButtonReleased(
                MOUSE_BUTTON_LEFT
            )
        )
        {
            if (
                teaching_active &&
                active_drag != TEACH_DRAG_NONE
            )
            {
                send_target(
                    &protocol,
                    target
                );

                last_guidance_send_time =
                    GetTime();
            }

            active_drag =
                TEACH_DRAG_NONE;
        }

        if (
            teaching_active &&
            active_drag == TEACH_DRAG_NONE &&
            IsMouseButtonPressed(
                MOUSE_BUTTON_LEFT
            ) &&
            CheckCollisionPointRec(
                mouse,
                viewport
            )
        )
        {
            if (hovered_axis != TEACH_DRAG_NONE)
            {
                Vector2 screen_direction;
                float robot_units_per_pixel;

                if (
                    begin_axis_drag(
                        hovered_axis,
                        viewport,
                        camera,
                        target_render,
                        &screen_direction,
                        &robot_units_per_pixel
                    )
                )
                {
                    active_drag = hovered_axis;
                    drag_start_mouse = mouse;
                    drag_axis_screen_direction = screen_direction;
                    drag_robot_units_per_pixel = robot_units_per_pixel;

                    memcpy(
                        drag_start_target,
                        target,
                        sizeof(drag_start_target)
                    );
                }
            }
            else if (mouse_near_target)
            {
                active_drag =
                    TEACH_DRAG_FREE;

                drag_plane_normal =
                    drag_plane_normal_for_view(
                        view,
                        camera
                    );

                drag_plane_distance =
                    Vector3DotProduct(
                        drag_plane_normal,
                        target_render
                    );
            }
        }

        if (
            teaching_active &&
            active_drag != TEACH_DRAG_NONE &&
            IsMouseButtonDown(
                MOUSE_BUTTON_LEFT
            )
        )
        {
            bool target_changed =
                false;

            const int axis_index =
                drag_axis_robot_index(
                    active_drag
                );

            if (axis_index >= 0)
            {
                const float mouse_delta_x =
                    mouse.x - drag_start_mouse.x;

                const float mouse_delta_y =
                    mouse.y - drag_start_mouse.y;

                const float projected_pixels =
                    mouse_delta_x * drag_axis_screen_direction.x +
                    mouse_delta_y * drag_axis_screen_direction.y;

                target[axis_index] =
                    drag_start_target[axis_index] +
                    projected_pixels * drag_robot_units_per_pixel;

                target_changed =
                    true;
            }
            else if (active_drag == TEACH_DRAG_FREE)
            {
                const Ray ray =
                    viewport_mouse_ray(
                        mouse,
                        viewport,
                        camera
                    );

                Vector3 hit =
                    target_render;

                if (
                    ray_plane_intersection(
                        ray,
                        drag_plane_normal,
                        drag_plane_distance,
                        &hit
                    )
                )
                {
                    render_to_robot(
                        hit,
                        &target[0],
                        &target[1],
                        &target[2]
                    );

                    target_changed =
                        true;
                }
            }

            if (target_changed)
            {
                const double now =
                    GetTime();

                if (
                    last_guidance_send_time < 0.0 ||
                    now - last_guidance_send_time >=
                        guidance_send_period_s
                )
                {
                    send_target(
                        &protocol,
                        target
                    );

                    last_guidance_send_time =
                        now;
                }
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
            "Drag X / Y / Z arrows for constrained motion; drag the sphere for free motion.",
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
            hovered_axis,
            active_drag
        );

        EndMode3D();

        draw_gizmo_labels(
            robot_to_render(
                target[0],
                target[1],
                target[2]
            ),
            viewport,
            camera,
            hovered_axis,
            active_drag
        );

        EndScissorMode();

        const Rectangle monitor =
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
                ? "Drag an axis arrow or sphere"
                : "Press START on main HMI",
            746.0F,
            532.0F,
            11.5F,
            HMI_C_MUTED,
            false
        );

        hmi_ui_text(
            "Red X / Green Y / Blue Z",
            746.0F,
            558.0F,
            11.0F,
            HMI_C_FAINT,
            false
        );

        hmi_ui_text(
            "Yellow = recorded point",
            746.0F,
            580.0F,
            11.0F,
            HMI_C_FAINT,
            false
        );

        hmi_ui_text(
            "Green sphere = actual TCP",
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
