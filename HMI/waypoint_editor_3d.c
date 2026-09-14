#include "waypoint_editor_3d.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "raymath.h"

#include "circular_path.h"

#define EDITOR_TEXTURE_WIDTH   960
#define EDITOR_TEXTURE_HEIGHT  600
#define EDITOR_AXIS_LENGTH     0.18f
#define EDITOR_AXIS_RADIUS     0.006f
#define EDITOR_AXIS_HIT_PIXELS 12.0f
#define EDITOR_PREVIEW_POINTS  160U

static const Color COLOR_A = { 86, 210, 145, 255};
static const Color COLOR_B = {240, 188,  82, 255};
static const Color COLOR_C = {235,  98, 108, 255};
static const Color COLOR_X = {228,  82,  82, 255};
static const Color COLOR_Y = { 83, 201, 115, 255};
static const Color COLOR_Z = { 78, 145, 245, 255};

static Vector3 robot_to_render(float x, float y, float z)
{
    /*
     * Robot frame:  X, Y horizontal, Z vertical.
     * raylib frame: X, Z horizontal, Y vertical.
     */
    return (Vector3){x, z, -y};
}

static Vector3 pose_to_render(const HmiPose *pose)
{
    return
        robot_to_render(
            pose->value[0],
            pose->value[1],
            pose->value[2]
        );
}

static void render_to_pose_position(
    Vector3 render,
    HmiPose *pose
)
{
    if (pose == NULL)
    {
        return;
    }

    pose->value[0] = render.x;
    pose->value[1] = -render.z;
    pose->value[2] = render.y;
}

static Vec3 pose_to_control_vec3(const HmiPose *pose)
{
    return (Vec3)
    {{
        (real_t)pose->value[0],
        (real_t)pose->value[1],
        (real_t)pose->value[2]
    }};
}

static Vector3 control_vec3_to_render(Vec3 point)
{
    return
        robot_to_render(
            (float)point.v[0],
            (float)point.v[1],
            (float)point.v[2]
        );
}

static bool waypoint_is_used(
    HmiPathType path,
    int waypoint
)
{
    if (waypoint < 0 || waypoint >= HMI_NUM_WAYPOINTS)
    {
        return false;
    }

    if (path == HMI_PATH_LINE)
    {
        return waypoint < 2;
    }

    return true;
}

static Color waypoint_color(int waypoint)
{
    switch (waypoint)
    {
        case 0: return COLOR_A;
        case 1: return COLOR_B;
        case 2: return COLOR_C;
        default: return WHITE;
    }
}

static const char *waypoint_name(int waypoint)
{
    switch (waypoint)
    {
        case 0: return "A";
        case 1: return "B";
        case 2: return "C";
        default: return "?";
    }
}

static HmiWorkspaceCheck workspace_check_for_pose(
    const HmiWaypointEditor3D *editor,
    const HmiPose *pose
)
{
    if (editor == NULL || pose == NULL)
    {
        HmiWorkspaceCheck unavailable = {0};
        unavailable.status = HMI_WORKSPACE_CHECK_UNAVAILABLE;
        return unavailable;
    }

    return
        hmi_workspace_monitor_check(
            &editor->workspaceMonitor,
            (double)pose->value[0],
            (double)pose->value[1],
            (double)pose->value[2]
        );
}

static Color waypoint_display_color(
    const HmiWaypointEditor3D *editor,
    int waypoint
)
{
    HmiWorkspaceCheck check =
        workspace_check_for_pose(
            editor,
            &editor->working[waypoint]
        );

    if (check.status == HMI_WORKSPACE_CHECK_OUTSIDE)
    {
        return HMI_C_BAD;
    }

    return waypoint_color(waypoint);
}

static const char *view_name(HmiTeachView view)
{
    switch (view)
    {
        case HMI_TEACH_VIEW_PERSPECTIVE: return "Perspective";
        case HMI_TEACH_VIEW_TOP:         return "Top (X/Y)";
        case HMI_TEACH_VIEW_FRONT:       return "Front (X/Z)";
        case HMI_TEACH_VIEW_SIDE:        return "Side (Y/Z)";
        default:                         return "Unknown";
    }
}

static Vector3 average_used_points(
    const HmiWaypointEditor3D *editor
)
{
    Vector3 sum = {0};
    int count = 0;

    for (int i = 0; i < HMI_NUM_WAYPOINTS; i++)
    {
        if (!waypoint_is_used(editor->path, i))
        {
            continue;
        }

        sum = Vector3Add(sum, pose_to_render(&editor->working[i]));
        count++;
    }

    if (count == 0)
    {
        return robot_to_render(0.0f, -0.4f, 0.3f);
    }

    return Vector3Scale(sum, 1.0f / (float)count);
}

static void update_perspective_camera(HmiWaypointEditor3D *editor)
{
    float cosPitch = cosf(editor->orbitPitch);

    Vector3 offset =
    {
        editor->orbitDistance * cosPitch * cosf(editor->orbitYaw),
        editor->orbitDistance * sinf(editor->orbitPitch),
        editor->orbitDistance * cosPitch * sinf(editor->orbitYaw)
    };

    editor->camera.target = editor->orbitTarget;
    editor->camera.position = Vector3Add(editor->orbitTarget, offset);
    editor->camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    editor->camera.fovy = 45.0f;
    editor->camera.projection = CAMERA_PERSPECTIVE;
}

static void apply_view_preset(
    HmiWaypointEditor3D *editor,
    HmiTeachView view
)
{
    editor->view = view;
    editor->orbitDragging = false;
    editor->dragAxis = -1;

    if (view == HMI_TEACH_VIEW_PERSPECTIVE)
    {
        update_perspective_camera(editor);
        return;
    }

    editor->camera.target = editor->orbitTarget;
    editor->camera.fovy = 1.6f;
    editor->camera.projection = CAMERA_ORTHOGRAPHIC;

    if (view == HMI_TEACH_VIEW_TOP)
    {
        editor->camera.position =
            Vector3Add(
                editor->orbitTarget,
                (Vector3){0.0f, 2.0f, 0.0f}
            );

        /* Robot +Y points upward on screen. */
        editor->camera.up = (Vector3){0.0f, 0.0f, -1.0f};
    }
    else if (view == HMI_TEACH_VIEW_FRONT)
    {
        editor->camera.position =
            Vector3Add(
                editor->orbitTarget,
                (Vector3){0.0f, 0.0f, 2.0f}
            );

        editor->camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    }
    else
    {
        editor->camera.position =
            Vector3Add(
                editor->orbitTarget,
                (Vector3){2.0f, 0.0f, 0.0f}
            );

        editor->camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    }
}

static void sync_xyz_fields_from_active(
    HmiWaypointEditor3D *editor
)
{
    const HmiPose *pose =
        &editor->working[editor->activeWaypoint];

    hmi_numeric_field_set(&editor->xyzFields[0], pose->value[0], 6);
    hmi_numeric_field_set(&editor->xyzFields[1], pose->value[1], 6);
    hmi_numeric_field_set(&editor->xyzFields[2], pose->value[2], 6);

    editor->xyzFieldsDirty = false;
}

static void set_active_waypoint(
    HmiWaypointEditor3D *editor,
    int waypoint
)
{
    if (!waypoint_is_used(editor->path, waypoint))
    {
        return;
    }

    editor->activeWaypoint = waypoint;
    editor->dragAxis = -1;
    sync_xyz_fields_from_active(editor);
}

static Vector3 axis_direction_render(int axis)
{
    /* Robot X, Y, Z expressed in raylib render coordinates. */
    switch (axis)
    {
        case 0: return (Vector3){1.0f, 0.0f, 0.0f};
        case 1: return (Vector3){0.0f, 0.0f, -1.0f};
        case 2: return (Vector3){0.0f, 1.0f, 0.0f};
        default: return (Vector3){0.0f, 0.0f, 0.0f};
    }
}

static Color axis_color(int axis)
{
    switch (axis)
    {
        case 0: return COLOR_X;
        case 1: return COLOR_Y;
        case 2: return COLOR_Z;
        default: return WHITE;
    }
}

static float point_segment_distance(
    Vector2 point,
    Vector2 a,
    Vector2 b
)
{
    Vector2 ab = Vector2Subtract(b, a);
    float lengthSquared = Vector2DotProduct(ab, ab);

    if (lengthSquared <= 1e-8f)
    {
        return Vector2Distance(point, a);
    }

    float t =
        Vector2DotProduct(
            Vector2Subtract(point, a),
            ab
        ) /
        lengthSquared;

    t = Clamp(t, 0.0f, 1.0f);

    Vector2 closest = Vector2Add(a, Vector2Scale(ab, t));
    return Vector2Distance(point, closest);
}

static bool closest_axis_parameter(
    Vector3 axisOrigin,
    Vector3 axisDirection,
    Ray ray,
    float *parameter
)
{
    if (parameter == NULL)
    {
        return false;
    }

    Vector3 u = Vector3Normalize(axisDirection);
    Vector3 v = Vector3Normalize(ray.direction);
    Vector3 w0 = Vector3Subtract(axisOrigin, ray.position);

    float a = Vector3DotProduct(u, u);
    float b = Vector3DotProduct(u, v);
    float c = Vector3DotProduct(v, v);
    float d = Vector3DotProduct(u, w0);
    float e = Vector3DotProduct(v, w0);

    float denominator = a * c - b * b;

    if (fabsf(denominator) < 1e-6f)
    {
        return false;
    }

    *parameter = (b * e - c * d) / denominator;
    return true;
}

static bool intersect_ray_plane(
    Ray ray,
    Vector3 planePoint,
    Vector3 planeNormal,
    Vector3 *hit
)
{
    if (hit == NULL)
    {
        return false;
    }

    Vector3 normal = Vector3Normalize(planeNormal);
    float denominator = Vector3DotProduct(ray.direction, normal);

    if (fabsf(denominator) < 1e-6f)
    {
        return false;
    }

    float distance =
        Vector3DotProduct(
            Vector3Subtract(planePoint, ray.position),
            normal
        ) /
        denominator;

    if (distance < 0.0f)
    {
        return false;
    }

    *hit =
        Vector3Add(
            ray.position,
            Vector3Scale(ray.direction, distance)
        );

    return true;
}

static Rectangle fit_texture_rect(Rectangle available)
{
    const float aspect =
        (float)EDITOR_TEXTURE_WIDTH /
        (float)EDITOR_TEXTURE_HEIGHT;

    Rectangle result = available;

    if (available.width / available.height > aspect)
    {
        result.width = available.height * aspect;
        result.x =
            available.x +
            (available.width - result.width) * 0.5f;
    }
    else
    {
        result.height = available.width / aspect;
        result.y =
            available.y +
            (available.height - result.height) * 0.5f;
    }

    return result;
}

static Vector2 mouse_to_texture(
    Vector2 mouse,
    Rectangle viewRect
)
{
    return (Vector2)
    {
        (mouse.x - viewRect.x) /
            viewRect.width *
            (float)EDITOR_TEXTURE_WIDTH,

        (mouse.y - viewRect.y) /
            viewRect.height *
            (float)EDITOR_TEXTURE_HEIGHT
    };
}

static Ray texture_mouse_ray(
    const HmiWaypointEditor3D *editor,
    Vector2 textureMouse
)
{
    return
        GetScreenToWorldRayEx(
            textureMouse,
            editor->camera,
            EDITOR_TEXTURE_WIDTH,
            EDITOR_TEXTURE_HEIGHT
        );
}

static int hit_test_gizmo_axis(
    const HmiWaypointEditor3D *editor,
    Vector2 textureMouse
)
{
    Vector3 origin =
        pose_to_render(
            &editor->working[editor->activeWaypoint]
        );

    int bestAxis = -1;
    float bestDistance = EDITOR_AXIS_HIT_PIXELS;

    for (int axis = 0; axis < 3; axis++)
    {
        Vector3 end =
            Vector3Add(
                origin,
                Vector3Scale(
                    axis_direction_render(axis),
                    EDITOR_AXIS_LENGTH
                )
            );

        Vector2 a =
            GetWorldToScreenEx(
                origin,
                editor->camera,
                EDITOR_TEXTURE_WIDTH,
                EDITOR_TEXTURE_HEIGHT
            );

        Vector2 b =
            GetWorldToScreenEx(
                end,
                editor->camera,
                EDITOR_TEXTURE_WIDTH,
                EDITOR_TEXTURE_HEIGHT
            );

        float distance =
            point_segment_distance(
                textureMouse,
                a,
                b
            );

        if (distance < bestDistance)
        {
            bestDistance = distance;
            bestAxis = axis;
        }
    }

    return bestAxis;
}

static void update_camera_input(
    HmiWaypointEditor3D *editor,
    Rectangle viewRect,
    Vector2 mouse
)
{
    bool inside = CheckCollisionPointRec(mouse, viewRect);

    if (
        editor->view == HMI_TEACH_VIEW_PERSPECTIVE &&
        inside &&
        IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)
    )
    {
        editor->orbitDragging = true;
        editor->previousMouse = mouse;
    }

    if (editor->orbitDragging)
    {
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
        {
            Vector2 delta = Vector2Subtract(mouse, editor->previousMouse);
            editor->previousMouse = mouse;

            editor->orbitYaw += delta.x * 0.008f;
            editor->orbitPitch -= delta.y * 0.008f;
            editor->orbitPitch = Clamp(editor->orbitPitch, -1.30f, 1.30f);

            update_perspective_camera(editor);
        }
        else
        {
            editor->orbitDragging = false;
        }
    }

    if (inside)
    {
        float wheel = GetMouseWheelMove();

        if (wheel != 0.0f)
        {
            if (editor->view == HMI_TEACH_VIEW_PERSPECTIVE)
            {
                editor->orbitDistance *= powf(0.88f, wheel);
                editor->orbitDistance =
                    Clamp(editor->orbitDistance, 0.30f, 4.00f);
                update_perspective_camera(editor);
            }
            else
            {
                editor->camera.fovy *= powf(0.88f, wheel);
                editor->camera.fovy =
                    Clamp(editor->camera.fovy, 0.25f, 4.00f);
            }
        }
    }
}

static void update_waypoint_interaction(
    HmiWaypointEditor3D *editor,
    Rectangle viewRect,
    Vector2 mouse
)
{
    bool inside = CheckCollisionPointRec(mouse, viewRect);
    Vector2 textureMouse = mouse_to_texture(mouse, viewRect);
    Ray ray = texture_mouse_ray(editor, textureMouse);

    if (
        inside &&
        IsMouseButtonPressed(MOUSE_BUTTON_LEFT)
    )
    {
        int axis = hit_test_gizmo_axis(editor, textureMouse);

        if (axis >= 0)
        {
            Vector3 origin =
                pose_to_render(
                    &editor->working[editor->activeWaypoint]
                );

            float parameter = 0.0f;

            if (
                closest_axis_parameter(
                    origin,
                    axis_direction_render(axis),
                    ray,
                    &parameter
                )
            )
            {
                editor->dragAxis = axis;
                editor->dragOriginRender = origin;
                editor->dragStartParameter = parameter;
            }
        }
        else
        {
            Vector3 selected =
                pose_to_render(
                    &editor->working[editor->activeWaypoint]
                );

            Vector3 normal;

            if (editor->view == HMI_TEACH_VIEW_TOP)
            {
                normal = (Vector3){0.0f, 1.0f, 0.0f};
            }
            else if (editor->view == HMI_TEACH_VIEW_FRONT)
            {
                normal = (Vector3){0.0f, 0.0f, 1.0f};
            }
            else if (editor->view == HMI_TEACH_VIEW_SIDE)
            {
                normal = (Vector3){1.0f, 0.0f, 0.0f};
            }
            else
            {
                normal =
                    Vector3Normalize(
                        Vector3Subtract(
                            editor->camera.target,
                            editor->camera.position
                        )
                    );
            }

            Vector3 hit;

            if (intersect_ray_plane(ray, selected, normal, &hit))
            {
                render_to_pose_position(
                    hit,
                    &editor->working[editor->activeWaypoint]
                );

                sync_xyz_fields_from_active(editor);

                snprintf(
                    editor->message,
                    sizeof(editor->message),
                    "%s moved in the %s view plane. Drag an axis for the remaining depth.",
                    waypoint_name(editor->activeWaypoint),
                    view_name(editor->view)
                );
            }
        }
    }

    if (editor->dragAxis >= 0)
    {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            float parameter = 0.0f;

            if (
                closest_axis_parameter(
                    editor->dragOriginRender,
                    axis_direction_render(editor->dragAxis),
                    ray,
                    &parameter
                )
            )
            {
                float delta =
                    parameter -
                    editor->dragStartParameter;

                Vector3 moved =
                    Vector3Add(
                        editor->dragOriginRender,
                        Vector3Scale(
                            axis_direction_render(editor->dragAxis),
                            delta
                        )
                    );

                render_to_pose_position(
                    moved,
                    &editor->working[editor->activeWaypoint]
                );

                sync_xyz_fields_from_active(editor);
            }
        }
        else
        {
            editor->dragAxis = -1;
        }
    }
}

static void draw_world_axes(void)
{
    const float length = 0.35f;

    Vector3 origin = {0.0f, 0.0f, 0.0f};

    DrawLine3D(origin, (Vector3){ length, 0.0f, 0.0f}, COLOR_X);
    DrawLine3D(origin, (Vector3){0.0f, 0.0f, -length}, COLOR_Y);
    DrawLine3D(origin, (Vector3){0.0f, length, 0.0f}, COLOR_Z);

    DrawSphere(origin, 0.012f, (Color){210, 215, 225, 255});
}

static void draw_path_preview_3d(
    const HmiWaypointEditor3D *editor
)
{
    Vector3 a = pose_to_render(&editor->working[0]);
    Vector3 b = pose_to_render(&editor->working[1]);

    if (editor->path == HMI_PATH_LINE)
    {
        DrawLine3D(a, b, HMI_C_ACCENT);
        return;
    }

    Vec3 points[EDITOR_PREVIEW_POINTS];
    CircularPathInfo info;

    bool ok;

    if (editor->path == HMI_PATH_ARC)
    {
        ok =
            generate_arc_waypoints(
                pose_to_control_vec3(&editor->working[0]),
                pose_to_control_vec3(&editor->working[1]),
                pose_to_control_vec3(&editor->working[2]),
                EDITOR_PREVIEW_POINTS,
                points,
                &info
            );
    }
    else
    {
        ok =
            generate_full_circle_waypoints(
                pose_to_control_vec3(&editor->working[0]),
                pose_to_control_vec3(&editor->working[1]),
                pose_to_control_vec3(&editor->working[2]),
                +1,
                EDITOR_PREVIEW_POINTS,
                points,
                &info
            );
    }

    if (!ok)
    {
        DrawLine3D(a, b, HMI_C_WARN);
        DrawLine3D(b, pose_to_render(&editor->working[2]), HMI_C_WARN);
        return;
    }

    for (size_t i = 1; i < EDITOR_PREVIEW_POINTS; i++)
    {
        DrawLine3D(
            control_vec3_to_render(points[i - 1]),
            control_vec3_to_render(points[i]),
            HMI_C_ACCENT
        );
    }
}

static void draw_waypoints_3d(
    const HmiWaypointEditor3D *editor
)
{
    for (int i = 0; i < HMI_NUM_WAYPOINTS; i++)
    {
        if (!waypoint_is_used(editor->path, i))
        {
            continue;
        }

        Vector3 position = pose_to_render(&editor->working[i]);
        Color color = waypoint_display_color(editor, i);

        HmiWorkspaceCheck check =
            workspace_check_for_pose(
                editor,
                &editor->working[i]
            );

        DrawSphere(position, 0.022f, color);
        DrawSphereWires(position, 0.026f, 8, 8, Fade(WHITE, 0.75f));

        if (check.status == HMI_WORKSPACE_CHECK_OUTSIDE)
        {
            DrawSphereWires(position, 0.043f, 12, 12, HMI_C_BAD);
        }

        if (i == editor->activeWaypoint)
        {
            DrawSphereWires(position, 0.036f, 10, 10, WHITE);
        }
    }
}

static void draw_gizmo_3d(
    const HmiWaypointEditor3D *editor
)
{
    Vector3 origin =
        pose_to_render(
            &editor->working[editor->activeWaypoint]
        );

    for (int axis = 0; axis < 3; axis++)
    {
        Vector3 direction = axis_direction_render(axis);
        Vector3 end =
            Vector3Add(
                origin,
                Vector3Scale(direction, EDITOR_AXIS_LENGTH)
            );

        Color color = axis_color(axis);

        DrawCylinderEx(
            origin,
            end,
            EDITOR_AXIS_RADIUS,
            EDITOR_AXIS_RADIUS,
            10,
            color
        );

        DrawSphere(end, 0.014f, color);
    }
}

static void draw_texture_labels(
    const HmiWaypointEditor3D *editor
)
{
    for (int i = 0; i < HMI_NUM_WAYPOINTS; i++)
    {
        if (!waypoint_is_used(editor->path, i))
        {
            continue;
        }

        Vector2 screen =
            GetWorldToScreenEx(
                pose_to_render(&editor->working[i]),
                editor->camera,
                EDITOR_TEXTURE_WIDTH,
                EDITOR_TEXTURE_HEIGHT
            );

        DrawText(
            waypoint_name(i),
            (int)screen.x + 10,
            (int)screen.y - 18,
            18,
            waypoint_display_color(editor, i)
        );
    }

    Vector3 origin =
        pose_to_render(
            &editor->working[editor->activeWaypoint]
        );

    const char *labels[3] = {"X", "Y", "Z"};

    for (int axis = 0; axis < 3; axis++)
    {
        Vector3 end =
            Vector3Add(
                origin,
                Vector3Scale(
                    axis_direction_render(axis),
                    EDITOR_AXIS_LENGTH
                )
            );

        Vector2 screen =
            GetWorldToScreenEx(
                end,
                editor->camera,
                EDITOR_TEXTURE_WIDTH,
                EDITOR_TEXTURE_HEIGHT
            );

        DrawText(
            labels[axis],
            (int)screen.x + 5,
            (int)screen.y - 8,
            16,
            axis_color(axis)
        );
    }

    if (editor->workspaceMonitor.ready)
    {
        DrawText(
            "RED WAYPOINT = OUTSIDE SAMPLED FK WORKSPACE",
            18,
            18,
            15,
            HMI_C_MUTED
        );
    }
}

static void render_workspace_texture(
    HmiWaypointEditor3D *editor
)
{
    BeginTextureMode(editor->renderTarget);

    ClearBackground((Color){12, 15, 20, 255});

    BeginMode3D(editor->camera);

    DrawGrid(30, 0.10f);
    draw_world_axes();
    draw_path_preview_3d(editor);
    draw_waypoints_3d(editor);
    draw_gizmo_3d(editor);

    EndMode3D();

    draw_texture_labels(editor);

    EndTextureMode();
}

static void draw_workspace_status(
    HmiWaypointEditor3D *editor,
    Rectangle panel
)
{
    HmiWorkspaceCheck check =
        workspace_check_for_pose(
            editor,
            &editor->working[editor->activeWaypoint]
        );

    hmi_ui_text(
        "SAMPLED KINEMATIC WORKSPACE",
        panel.x + 16.0f,
        panel.y + 307.0f,
        12.0f,
        HMI_C_MUTED,
        true
    );

    char detail[160];

    if (check.status == HMI_WORKSPACE_CHECK_INSIDE)
    {
        hmi_ui_text(
            "INSIDE SAMPLED WORKSPACE",
            panel.x + 16.0f,
            panel.y + 331.0f,
            12.0f,
            HMI_C_GOOD,
            true
        );

        snprintf(
            detail,
            sizeof(detail),
            "rho %.3f m | local sampled range %.3f .. %.3f m",
            check.rho,
            check.rhoMin,
            check.rhoMax
        );

        hmi_ui_text(
            detail,
            panel.x + 16.0f,
            panel.y + 351.0f,
            10.5f,
            HMI_C_FAINT,
            false
        );
    }
    else if (check.status == HMI_WORKSPACE_CHECK_OUTSIDE)
    {
        hmi_ui_text(
            "WARNING: OUTSIDE SAMPLED WORKSPACE",
            panel.x + 16.0f,
            panel.y + 331.0f,
            12.0f,
            HMI_C_BAD,
            true
        );

        if (
            check.z < check.zMin - 0.025 ||
            check.z > check.zMax + 0.025
        )
        {
            snprintf(
                detail,
                sizeof(detail),
                "Z %.3f m | sampled Z range %.3f .. %.3f m",
                check.z,
                check.zMin,
                check.zMax
            );
        }
        else
        {
            snprintf(
                detail,
                sizeof(detail),
                "rho %.3f m | local sampled range %.3f .. %.3f m",
                check.rho,
                check.rhoMin,
                check.rhoMax
            );
        }

        hmi_ui_text(
            detail,
            panel.x + 16.0f,
            panel.y + 351.0f,
            10.5f,
            HMI_C_BAD,
            false
        );
    }
    else
    {
        hmi_ui_text(
            "WORKSPACE CHECK UNAVAILABLE",
            panel.x + 16.0f,
            panel.y + 331.0f,
            12.0f,
            HMI_C_WARN,
            true
        );
    }

    hmi_ui_text(
        "Position-only FK sampling; not IK/orientation/collision validation.",
        panel.x + 16.0f,
        panel.y + 373.0f,
        10.2f,
        HMI_C_FAINT,
        false
    );
}

static void draw_sidebar_coordinates(
    HmiWaypointEditor3D *editor,
    Rectangle panel,
    Vector2 mouse
)
{
    hmi_ui_text(
        "ACTIVE WAYPOINT",
        panel.x + 16.0f,
        panel.y + 16.0f,
        12.0f,
        HMI_C_MUTED,
        true
    );

    float buttonY = panel.y + 40.0f;
    float buttonWidth = (panel.width - 48.0f) / 3.0f;

    for (int i = 0; i < HMI_NUM_WAYPOINTS; i++)
    {
        Rectangle button =
        {
            panel.x + 16.0f + i * (buttonWidth + 8.0f),
            buttonY,
            buttonWidth,
            38.0f
        };

        char label[24];
        snprintf(
            label,
            sizeof(label),
            "%s%s",
            waypoint_name(i),
            i == editor->activeWaypoint ? "  SELECTED" : ""
        );

        bool enabled = waypoint_is_used(editor->path, i);

        if (
            hmi_ui_button(
                button,
                label,
                i == editor->activeWaypoint
                    ? HMI_BUTTON_PRIMARY
                    : HMI_BUTTON_NORMAL,
                enabled,
                mouse,
                false
            )
        )
        {
            set_active_waypoint(editor, i);
        }
    }

    hmi_ui_text(
        "POSITION  [m]",
        panel.x + 16.0f,
        panel.y + 96.0f,
        12.0f,
        HMI_C_MUTED,
        true
    );

    const char *labels[3] = {"X", "Y", "Z"};

    for (int axis = 0; axis < 3; axis++)
    {
        float y = panel.y + 121.0f + axis * 61.0f;

        hmi_ui_text(
            labels[axis],
            panel.x + 16.0f,
            y + 9.0f,
            14.0f,
            axis_color(axis),
            true
        );

        Rectangle field =
        {
            panel.x + 42.0f,
            y,
            panel.width - 58.0f,
            39.0f
        };

        hmi_numeric_field_update_draw(
            &editor->xyzFields[axis],
            field,
            true,
            mouse,
            false
        );
    }

    double values[3];
    bool valid = true;

    for (int axis = 0; axis < 3; axis++)
    {
        if (
            !hmi_numeric_field_parse(
                &editor->xyzFields[axis],
                &values[axis]
            )
        )
        {
            valid = false;
        }
    }

    if (valid)
    {
        HmiPose *active =
            &editor->working[editor->activeWaypoint];

        active->value[0] = (float)values[0];
        active->value[1] = (float)values[1];
        active->value[2] = (float)values[2];
    }

    draw_workspace_status(editor, panel);

    hmi_ui_text(
        "HOW TO MOVE",
        panel.x + 16.0f,
        panel.y + 407.0f,
        12.0f,
        HMI_C_MUTED,
        true
    );

    hmi_ui_text(
        "Click viewport: move in current view plane",
        panel.x + 16.0f,
        panel.y + 432.0f,
        11.5f,
        HMI_C_FAINT,
        false
    );

    hmi_ui_text(
        "Drag X / Y / Z gizmo: move on one axis",
        panel.x + 16.0f,
        panel.y + 453.0f,
        11.5f,
        HMI_C_FAINT,
        false
    );

    hmi_ui_text(
        "Right-drag: orbit  |  Wheel: zoom",
        panel.x + 16.0f,
        panel.y + 474.0f,
        11.5f,
        HMI_C_FAINT,
        false
    );

    hmi_ui_text(
        "Tip: Top keeps Z, Front keeps Y, Side keeps X.",
        panel.x + 16.0f,
        panel.y + 507.0f,
        11.0f,
        HMI_C_WARN,
        false
    );
}

bool hmi_waypoint_editor_init(HmiWaypointEditor3D *editor)
{
    if (editor == NULL)
    {
        return false;
    }

    memset(editor, 0, sizeof(*editor));

    editor->renderTarget =
        LoadRenderTexture(
            EDITOR_TEXTURE_WIDTH,
            EDITOR_TEXTURE_HEIGHT
        );

    editor->renderTargetReady =
        editor->renderTarget.id != 0;

    if (editor->renderTargetReady)
    {
        SetTextureFilter(
            editor->renderTarget.texture,
            TEXTURE_FILTER_BILINEAR
        );
    }

    if (!hmi_workspace_monitor_init(&editor->workspaceMonitor))
    {
        fprintf(
            stderr,
            "Warning: sampled workspace monitor could not be initialized; 3D teaching will continue without workspace warnings.\n"
        );
    }

    editor->dragAxis = -1;
    editor->orbitYaw = 0.78f;
    editor->orbitPitch = 0.45f;
    editor->orbitDistance = 1.45f;

    return editor->renderTargetReady;
}

void hmi_waypoint_editor_shutdown(HmiWaypointEditor3D *editor)
{
    if (editor == NULL)
    {
        return;
    }

    hmi_workspace_monitor_shutdown(&editor->workspaceMonitor);

    if (editor->renderTargetReady)
    {
        UnloadRenderTexture(editor->renderTarget);
    }

    editor->renderTargetReady = false;
    editor->open = false;
}

void hmi_waypoint_editor_open(
    HmiWaypointEditor3D *editor,
    HmiPathType path,
    const HmiPose waypoints[HMI_NUM_WAYPOINTS]
)
{
    if (editor == NULL || waypoints == NULL)
    {
        return;
    }

    editor->path = path;
    memcpy(editor->working, waypoints, sizeof(editor->working));

    editor->activeWaypoint = 0;
    editor->view = HMI_TEACH_VIEW_PERSPECTIVE;
    editor->dragAxis = -1;
    editor->orbitDragging = false;

    editor->orbitTarget = average_used_points(editor);
    editor->orbitYaw = 0.78f;
    editor->orbitPitch = 0.45f;
    editor->orbitDistance = 1.45f;

    apply_view_preset(editor, HMI_TEACH_VIEW_PERSPECTIVE);
    sync_xyz_fields_from_active(editor);

    if (editor->workspaceMonitor.ready)
    {
        snprintf(
            editor->message,
            sizeof(editor->message),
            "Workspace warning active: a red waypoint is outside the sampled FK workspace."
        );
    }
    else
    {
        snprintf(
            editor->message,
            sizeof(editor->message),
            "Free-space teaching active. Sampled workspace warning is unavailable."
        );
    }

    editor->open = true;
}

bool hmi_waypoint_editor_is_open(
    const HmiWaypointEditor3D *editor
)
{
    return editor != NULL && editor->open;
}

HmiWaypointEditorResult hmi_waypoint_editor_frame(
    HmiWaypointEditor3D *editor,
    Rectangle bounds,
    Vector2 mouse,
    HmiPose outputWaypoints[HMI_NUM_WAYPOINTS]
)
{
    if (
        editor == NULL ||
        !editor->open ||
        !editor->renderTargetReady
    )
    {
        return HMI_EDITOR_RESULT_NONE;
    }

    const float headerHeight = 58.0f;
    const float footerHeight = 58.0f;
    const float sidebarWidth = 310.0f;
    const float gap = 14.0f;

    Rectangle viewportArea =
    {
        bounds.x + 16.0f,
        bounds.y + headerHeight,
        bounds.width - sidebarWidth - gap - 32.0f,
        bounds.height - headerHeight - footerHeight
    };

    Rectangle viewRect = fit_texture_rect(viewportArea);

    Rectangle sidebar =
    {
        bounds.x + bounds.width - sidebarWidth - 16.0f,
        bounds.y + headerHeight,
        sidebarWidth,
        bounds.height - headerHeight - footerHeight
    };

    bool mouseInsideView =
        CheckCollisionPointRec(mouse, viewRect);

    update_camera_input(editor, viewRect, mouse);

    if (
        !editor->orbitDragging &&
        mouseInsideView
    )
    {
        update_waypoint_interaction(editor, viewRect, mouse);
    }

    render_workspace_texture(editor);

    DrawRectangle(
        0,
        0,
        GetScreenWidth(),
        GetScreenHeight(),
        Fade(BLACK, 0.72f)
    );

    hmi_ui_panel(bounds, (Color){18, 22, 29, 255});

    hmi_ui_text(
        "3D WAYPOINT TEACHING",
        bounds.x + 18.0f,
        bounds.y + 15.0f,
        22.0f,
        HMI_C_TEXT,
        true
    );

    hmi_ui_text(
        "Free-space XYZ placement. Orientation stays in the main HMI for now.",
        bounds.x + 18.0f,
        bounds.y + 39.0f,
        12.0f,
        HMI_C_MUTED,
        false
    );

    Rectangle frameButton =
    {
        bounds.x + bounds.width - 455.0f,
        bounds.y + 12.0f,
        110.0f,
        36.0f
    };

    if (
        hmi_ui_button(
            frameButton,
            "FRAME POINTS",
            HMI_BUTTON_NORMAL,
            true,
            mouse,
            false
        )
    )
    {
        editor->orbitTarget = average_used_points(editor);
        apply_view_preset(editor, editor->view);
    }

    const char *viewLabels[4] =
    {
        "PERSPECTIVE",
        "TOP",
        "FRONT",
        "SIDE"
    };

    for (int i = 0; i < 4; i++)
    {
        Rectangle button =
        {
            bounds.x + bounds.width - 334.0f + i * 78.0f,
            bounds.y + 12.0f,
            70.0f,
            36.0f
        };

        if (
            hmi_ui_button(
                button,
                viewLabels[i],
                editor->view == (HmiTeachView)i
                    ? HMI_BUTTON_PRIMARY
                    : HMI_BUTTON_NORMAL,
                true,
                mouse,
                false
            )
        )
        {
            apply_view_preset(editor, (HmiTeachView)i);
        }
    }

    DrawTexturePro(
        editor->renderTarget.texture,
        (Rectangle)
        {
            0.0f,
            0.0f,
            (float)editor->renderTarget.texture.width,
            -(float)editor->renderTarget.texture.height
        },
        viewRect,
        (Vector2){0.0f, 0.0f},
        0.0f,
        WHITE
    );

    DrawRectangleLinesEx(viewRect, 1.5f, HMI_C_BORDER_HI);

    hmi_ui_panel(sidebar, HMI_C_PANEL);
    draw_sidebar_coordinates(editor, sidebar, mouse);

    HmiWorkspaceCheck activeCheck =
        workspace_check_for_pose(
            editor,
            &editor->working[editor->activeWaypoint]
        );

    hmi_ui_text(
        editor->message,
        bounds.x + 18.0f,
        bounds.y + bounds.height - 39.0f,
        11.5f,
        activeCheck.status == HMI_WORKSPACE_CHECK_OUTSIDE
            ? HMI_C_BAD
            : HMI_C_MUTED,
        false
    );

    Rectangle cancelButton =
    {
        bounds.x + bounds.width - 252.0f,
        bounds.y + bounds.height - 47.0f,
        104.0f,
        36.0f
    };

    Rectangle applyButton =
    {
        bounds.x + bounds.width - 138.0f,
        bounds.y + bounds.height - 47.0f,
        120.0f,
        36.0f
    };

    if (
        hmi_ui_button(
            cancelButton,
            "CANCEL",
            HMI_BUTTON_NORMAL,
            true,
            mouse,
            false
        ) ||
        IsKeyPressed(KEY_ESCAPE)
    )
    {
        editor->open = false;
        editor->dragAxis = -1;
        editor->orbitDragging = false;
        return HMI_EDITOR_RESULT_CANCELLED;
    }

    if (
        hmi_ui_button(
            applyButton,
            "APPLY XYZ",
            activeCheck.status == HMI_WORKSPACE_CHECK_OUTSIDE
                ? HMI_BUTTON_DANGER
                : HMI_BUTTON_PRIMARY,
            true,
            mouse,
            false
        )
    )
    {
        if (outputWaypoints != NULL)
        {
            memcpy(
                outputWaypoints,
                editor->working,
                sizeof(editor->working)
            );
        }

        editor->open = false;
        editor->dragAxis = -1;
        editor->orbitDragging = false;
        return HMI_EDITOR_RESULT_APPLIED;
    }

    return HMI_EDITOR_RESULT_NONE;
}
