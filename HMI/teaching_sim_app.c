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

/*
 * The teaching manipulator intentionally reuses the proven interaction model
 * from HMI-Mock/waypoint_editor_3d.c:
 *
 *   - fixed-length X/Y/Z axis handles
 *   - screen-space hit testing for easy selection
 *   - ray-to-axis closest-point dragging for true 3D constrained motion
 *   - cylinder shaft + spherical end handle
 */
#define GIZMO_AXIS_LENGTH_M   0.18F
#define GIZMO_AXIS_RADIUS_M   0.006F
#define GIZMO_AXIS_HIT_PX     14.0F
#define GIZMO_ENDPOINT_RADIUS 0.014F
#define TARGET_HIT_RADIUS_PX  42.0F

static const Color GIZMO_COLOR_X = {228, 82, 82, 255};
static const Color GIZMO_COLOR_Y = {83, 201, 115, 255};
static const Color GIZMO_COLOR_Z = {78, 145, 245, 255};


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
    /*
     * Robot frame:  X, Y horizontal, Z vertical.
     * raylib frame: X, Z horizontal, Y vertical.
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


static Vector3 axis_direction_render(
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


static Color axis_color(
    TeachDragMode mode
)
{
    switch (mode)
    {
        case TEACH_DRAG_X: return GIZMO_COLOR_X;
        case TEACH_DRAG_Y: return GIZMO_COLOR_Y;
        case TEACH_DRAG_Z: return GIZMO_COLOR_Z;
        default: return WHITE;
    }
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


static bool intersect_ray_plane(
    Ray ray,
    Vector3 plane_point,
    Vector3 plane_normal,
    Vector3 *hit
)
{
    if (hit == NULL)
    {
        return false;
    }

    const Vector3 normal =
        Vector3Normalize(
            plane_normal
        );

    const float denominator =
        Vector3DotProduct(
            ray.direction,
            normal
        );

    if (fabsf(denominator) < 1.0e-6F)
    {
        return false;
    }

    const float distance =
        Vector3DotProduct(
            Vector3Subtract(
                plane_point,
                ray.position
            ),
            normal
        ) /
        denominator;

    if (distance < 0.0F)
    {
        return false;
    }

    *hit =
        Vector3Add(
            ray.position,
            Vector3Scale(
                ray.direction,
                distance
            )
        );

    return true;
}


/*
 * Exact axis-drag primitive used by the old HMI-Mock editor.
 *
 * The mouse ray and selected axis are treated as two 3D lines. The closest
 * point on the axis supplies a scalar parameter. Dragging changes only that
 * parameter, so the selected robot coordinate moves while the other two remain
 * fixed regardless of perspective projection.
 */
static bool closest_axis_parameter(
    Vector3 axis_origin,
    Vector3 axis_direction,
    Ray ray,
    float *parameter
)
{
    if (parameter == NULL)
    {
        return false;
    }

    const Vector3 u =
        Vector3Normalize(
            axis_direction
        );

    const Vector3 v =
        Vector3Normalize(
            ray.direction
        );

    const Vector3 w0 =
        Vector3Subtract(
            axis_origin,
            ray.position
        );

    const float a = Vector3DotProduct(u, u);
    const float b = Vector3DotProduct(u, v);
    const float c = Vector3DotProduct(v, v);
    const float d = Vector3DotProduct(u, w0);
    const float e = Vector3DotProduct(v, w0);

    const float denominator =
        a * c - b * b;

    if (fabsf(denominator) < 1.0e-6F)
    {
        return false;
    }

    *parameter =
        (b * e - c * d) /
        denominator;

    return true;
}


static float point_segment_distance(
    Vector2 point,
    Vector2 a,
    Vector2 b
)
{
    const Vector2 ab =
        Vector2Subtract(
            b,
            a
        );

    const float length_squared =
        Vector2DotProduct(
            ab,
            ab
        );

    if (length_squared <= 1.0e-8F)
    {
        return
            Vector2Distance(
                point,
                a
            );
    }

    float t =
        Vector2DotProduct(
            Vector2Subtract(
                point,
                a
            ),
            ab
        ) /
        length_squared;

    t =
        Clamp(
            t,
            0.0F,
            1.0F
        );

    const Vector2 closest =
        Vector2Add(
            a,
            Vector2Scale(
                ab,
                t
            )
        );

    return
        Vector2Distance(
            point,
            closest
        );
}


static TeachDragMode hit_test_gizmo_axis(
    Vector2 mouse,
    Rectangle viewport,
    Camera3D camera,
    Vector3 origin
)
{
    const TeachDragMode axes[3] =
    {
        TEACH_DRAG_X,
        TEACH_DRAG_Y,
        TEACH_DRAG_Z
    };

    TeachDragMode best_axis =
        TEACH_DRAG_NONE;

    float best_distance =
        GIZMO_AXIS_HIT_PX;

    const Vector2 origin_screen =
        viewport_world_to_screen(
            origin,
            viewport,
            camera
        );

    for (int index = 0;
         index < 3;
         ++index)
    {
        const Vector3 end =
            Vector3Add(
                origin,
                Vector3Scale(
                    axis_direction_render(
                        axes[index]
                    ),
                    GIZMO_AXIS_LENGTH_M
                )
            );

        const Vector2 end_screen =
            viewport_world_to_screen(
                end,
                viewport,
                camera
            );

        const float distance =
            point_segment_distance(
                mouse,
                origin_screen,
                end_screen
            );

        if (distance < best_distance)
        {
            best_distance =
                distance;

            best_axis =
                axes[index];
        }
    }

    return best_axis;
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


static void draw_gizmo_axis(
    Vector3 origin,
    TeachDragMode mode,
    bool active
)
{
    const Vector3 end =
        Vector3Add(
            origin,
            Vector3Scale(
                axis_direction_render(
                    mode
                ),
                GIZMO_AXIS_LENGTH_M
            )
        );

    Color color =
        axis_color(
            mode
        );

    const float radius =
        active
        ? GIZMO_AXIS_RADIUS_M * 1.8F
        : GIZMO_AXIS_RADIUS_M;

    if (active)
    {
        color =
            (Color)
            {
                (unsigned char)((color.r + 55U > 255U) ? 255U : color.r + 55U),
                (unsigned char)((color.g + 55U > 255U) ? 255U : color.g + 55U),
                (unsigned char)((color.b + 55U > 255U) ? 255U : color.b + 55U),
                255U
            };
    }

    DrawCylinderEx(
        origin,
        end,
        radius,
        radius,
        10,
        color
    );

    DrawSphere(
        end,
        active
            ? GIZMO_ENDPOINT_RADIUS * 1.35F
            : GIZMO_ENDPOINT_RADIUS,
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

    /* World-frame reference axes. */
    DrawLine3D(
        (Vector3){0.0F, 0.0F, 0.0F},
        (Vector3){0.65F, 0.0F, 0.0F},
        GIZMO_COLOR_X
    );

    DrawLine3D(
        (Vector3){0.0F, 0.0F, 0.0F},
        (Vector3){0.0F, 0.0F, -0.65F},
        GIZMO_COLOR_Y
    );

    DrawLine3D(
        (Vector3){0.0F, 0.0F, 0.0F},
        (Vector3){0.0F, 0.65F, 0.0F},
        GIZMO_COLOR_Z
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

    draw_gizmo_axis(
        desired,
        TEACH_DRAG_X,
        hovered_axis == TEACH_DRAG_X ||
        active_drag == TEACH_DRAG_X
    );

    draw_gizmo_axis(
        desired,
        TEACH_DRAG_Y,
        hovered_axis == TEACH_DRAG_Y ||
        active_drag == TEACH_DRAG_Y
    );

    draw_gizmo_axis(
        desired,
        TEACH_DRAG_Z,
        hovered_axis == TEACH_DRAG_Z ||
        active_drag == TEACH_DRAG_Z
    );

    for (uint32_t index = 0U;
         index < status->recorded_count &&
         index < HMI_MAX_RECORDED_POINTS;
         ++index)
    {
        const Vector3 recorded =
            robot_to_render(
                status->recorded_tcp_m[index][0],
                status->recorded_tcp_m[index][1],
                status->recorded_tcp_m[index][2]
            );

        DrawSphere(
            recorded,
            0.042F,
            HMI_C_WARN
        );

        if (index > 0U)
        {
            const Vector3 previous =
                robot_to_render(
                    status->recorded_tcp_m[index - 1U][0],
                    status->recorded_tcp_m[index - 1U][1],
                    status->recorded_tcp_m[index - 1U][2]
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
    Vector3 origin,
    Rectangle viewport,
    Camera3D camera
)
{
    const TeachDragMode axes[3] =
    {
        TEACH_DRAG_X,
        TEACH_DRAG_Y,
        TEACH_DRAG_Z
    };

    const char *labels[3] =
    {
        "X",
        "Y",
        "Z"
    };

    for (int index = 0;
         index < 3;
         ++index)
    {
        const Vector3 end =
            Vector3Add(
                origin,
                Vector3Scale(
                    axis_direction_render(
                        axes[index]
                    ),
                    GIZMO_AXIS_LENGTH_M
                )
            );

        const Vector2 screen =
            viewport_world_to_screen(
                end,
                viewport,
                camera
            );

        DrawText(
            labels[index],
            (int)screen.x + 5,
            (int)screen.y - 8,
            16,
            axis_color(
                axes[index]
            )
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

    Vector3 drag_origin_render =
        {0.0F, 0.0F, 0.0F};

    float drag_start_parameter =
        0.0F;

    Vector3 free_drag_plane_normal =
        {0.0F, 1.0F, 0.0F};

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

        const Rectangle perspective_button =
            {28.0F, 62.0F, 128.0F, 38.0F};

        const Rectangle top_button =
            {166.0F, 62.0F, 92.0F, 38.0F};

        const Rectangle front_button =
            {268.0F, 62.0F, 92.0F, 38.0F};

        const Rectangle side_button =
            {370.0F, 62.0F, 92.0F, 38.0F};

        if (
            hmi_ui_button(
                perspective_button,
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
            active_drag = TEACH_DRAG_NONE;
        }

        if (
            hmi_ui_button(
                top_button,
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
            active_drag = TEACH_DRAG_NONE;
        }

        if (
            hmi_ui_button(
                front_button,
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
            active_drag = TEACH_DRAG_NONE;
        }

        if (
            hmi_ui_button(
                side_button,
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
            active_drag = TEACH_DRAG_NONE;
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
            CheckCollisionPointRec(
                mouse,
                viewport
            )
        )
        {
            hovered_axis =
                hit_test_gizmo_axis(
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
            const Ray ray =
                viewport_mouse_ray(
                    mouse,
                    viewport,
                    camera
                );

            if (hovered_axis != TEACH_DRAG_NONE)
            {
                float parameter =
                    0.0F;

                if (
                    closest_axis_parameter(
                        target_render,
                        axis_direction_render(
                            hovered_axis
                        ),
                        ray,
                        &parameter
                    )
                )
                {
                    active_drag =
                        hovered_axis;

                    drag_origin_render =
                        target_render;

                    drag_start_parameter =
                        parameter;
                }
            }
            else if (mouse_near_target)
            {
                active_drag =
                    TEACH_DRAG_FREE;

                free_drag_plane_normal =
                    drag_plane_normal_for_view(
                        view,
                        camera
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
            const Ray ray =
                viewport_mouse_ray(
                    mouse,
                    viewport,
                    camera
                );

            bool target_changed =
                false;

            if (
                active_drag == TEACH_DRAG_X ||
                active_drag == TEACH_DRAG_Y ||
                active_drag == TEACH_DRAG_Z
            )
            {
                float parameter =
                    0.0F;

                if (
                    closest_axis_parameter(
                        drag_origin_render,
                        axis_direction_render(
                            active_drag
                        ),
                        ray,
                        &parameter
                    )
                )
                {
                    const float delta =
                        parameter -
                        drag_start_parameter;

                    const Vector3 moved =
                        Vector3Add(
                            drag_origin_render,
                            Vector3Scale(
                                axis_direction_render(
                                    active_drag
                                ),
                                delta
                            )
                        );

                    render_to_robot(
                        moved,
                        &target[0],
                        &target[1],
                        &target[2]
                    );

                    target_changed =
                        true;
                }
            }
            else if (active_drag == TEACH_DRAG_FREE)
            {
                Vector3 hit;

                if (
                    intersect_ray_plane(
                        ray,
                        target_render,
                        free_drag_plane_normal,
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

        hmi_ui_text(
            "SIMULATED HAND GUIDING",
            28.0F,
            22.0F,
            24.0F,
            HMI_C_TEXT,
            true
        );

        hmi_ui_text(
            "Drag the X / Y / Z handles like the mock editor; drag the sphere for free-plane motion.",
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
            camera
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
            GIZMO_COLOR_X,
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
            GIZMO_COLOR_Y,
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
            GIZMO_COLOR_Z,
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
                ? "Grab an axis handle or sphere"
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
