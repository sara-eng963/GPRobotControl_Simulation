#ifndef HMI_SCROLL_HOOKS_H
#define HMI_SCROLL_HOOKS_H

/*
 * ============================================================================
 * HMI FIXED-SCALE RESPONSIVE VIEWPORT
 * ============================================================================
 *
 * UX rule:
 *
 *   - The operator does NOT zoom the HMI.
 *   - The HMI always renders at one fixed, readable design scale.
 *   - Resizing the desktop window only changes the visible viewport.
 *   - Scrollbars appear only when the viewport is smaller than the content.
 *
 * The original 1440 x 900 layout is intentionally rendered at 90%.
 * This keeps the entire interface comfortably inside a typical laptop-height
 * window while avoiding the tiny text produced by "fit everything to width".
 *
 * Controls:
 *
 *      Mouse wheel          vertical scroll
 *      Shift + mouse wheel  horizontal scroll
 *      Horizontal wheel     horizontal scroll (trackpads / tilt wheels)
 *      Drag scrollbar thumb direct navigation
 *
 * If content overflows only horizontally, the normal wheel moves horizontally,
 * matching the behavior of the earlier HMI.
 *
 * There is no zoom state, no zoom keyboard shortcut, and no auto-fit-to-width.
 * ============================================================================
 */

#include "raylib.h"

#include <math.h>
#include <stdbool.h>

#define HMI_VIEW_LOGICAL_WIDTH    1440.0f
#define HMI_VIEW_LOGICAL_HEIGHT    900.0f

/* Fixed visual scale. This is a design choice, not user zoom. */
#define HMI_VIEW_SCALE               0.90f

#define HMI_VIEW_CONTENT_WIDTH  (HMI_VIEW_LOGICAL_WIDTH  * HMI_VIEW_SCALE)
#define HMI_VIEW_CONTENT_HEIGHT (HMI_VIEW_LOGICAL_HEIGHT * HMI_VIEW_SCALE)

#define HMI_SCROLLBAR_SIZE           13.0f
#define HMI_SCROLL_STEP              64.0f
#define HMI_SCROLL_MIN_THUMB         52.0f
#define HMI_SCROLL_EDGE_MARGIN        3.0f

static float hmi_scroll_x = 0.0f;
static float hmi_scroll_y = 0.0f;

static bool hmi_drag_x = false;
static bool hmi_drag_y = false;
static float hmi_drag_offset_x = 0.0f;
static float hmi_drag_offset_y = 0.0f;


static float hmi_view_clampf(
    float value,
    float minimum,
    float maximum
)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}


static void hmi_view_limits(
    float *max_x,
    float *max_y,
    float *viewport_w,
    float *viewport_h,
    float *origin_x,
    float *origin_y
)
{
    float actual_w = (float)GetScreenWidth();
    float actual_h = (float)GetScreenHeight();

    bool need_x = HMI_VIEW_CONTENT_WIDTH > actual_w;
    bool need_y = HMI_VIEW_CONTENT_HEIGHT > actual_h;

    float usable_w =
        actual_w - (need_y ? HMI_SCROLLBAR_SIZE : 0.0f);

    float usable_h =
        actual_h - (need_x ? HMI_SCROLLBAR_SIZE : 0.0f);

    /* One scrollbar can force the other axis to overflow. */
    need_x = HMI_VIEW_CONTENT_WIDTH > usable_w;
    need_y = HMI_VIEW_CONTENT_HEIGHT > usable_h;

    usable_w =
        actual_w - (need_y ? HMI_SCROLLBAR_SIZE : 0.0f);

    usable_h =
        actual_h - (need_x ? HMI_SCROLLBAR_SIZE : 0.0f);

    if (usable_w < 1.0f)
    {
        usable_w = 1.0f;
    }

    if (usable_h < 1.0f)
    {
        usable_h = 1.0f;
    }

    float local_max_x =
        fmaxf(0.0f, HMI_VIEW_CONTENT_WIDTH - usable_w);

    float local_max_y =
        fmaxf(0.0f, HMI_VIEW_CONTENT_HEIGHT - usable_h);

    float local_origin_x =
        local_max_x <= 0.0f
        ? (usable_w - HMI_VIEW_CONTENT_WIDTH) * 0.5f
        : 0.0f;

    float local_origin_y =
        local_max_y <= 0.0f
        ? (usable_h - HMI_VIEW_CONTENT_HEIGHT) * 0.5f
        : 0.0f;

    if (max_x != NULL) *max_x = local_max_x;
    if (max_y != NULL) *max_y = local_max_y;
    if (viewport_w != NULL) *viewport_w = usable_w;
    if (viewport_h != NULL) *viewport_h = usable_h;
    if (origin_x != NULL) *origin_x = local_origin_x;
    if (origin_y != NULL) *origin_y = local_origin_y;
}


static Rectangle hmi_horizontal_track(
    float viewport_w,
    float viewport_h
)
{
    return
        (Rectangle)
        {
            HMI_SCROLL_EDGE_MARGIN,
            viewport_h + 2.0f,
            fmaxf(
                1.0f,
                viewport_w - 2.0f * HMI_SCROLL_EDGE_MARGIN
            ),
            HMI_SCROLLBAR_SIZE - 4.0f
        };
}


static Rectangle hmi_vertical_track(
    float viewport_w,
    float viewport_h
)
{
    return
        (Rectangle)
        {
            viewport_w + 2.0f,
            HMI_SCROLL_EDGE_MARGIN,
            HMI_SCROLLBAR_SIZE - 4.0f,
            fmaxf(
                1.0f,
                viewport_h - 2.0f * HMI_SCROLL_EDGE_MARGIN
            )
        };
}


static Rectangle hmi_horizontal_thumb(
    Rectangle track,
    float viewport_w,
    float max_x
)
{
    float thumb_w =
        track.width *
        (viewport_w / HMI_VIEW_CONTENT_WIDTH);

    thumb_w =
        hmi_view_clampf(
            thumb_w,
            HMI_SCROLL_MIN_THUMB,
            track.width
        );

    float travel = track.width - thumb_w;
    float x = track.x;

    if (max_x > 0.0f && travel > 0.0f)
    {
        x += (hmi_scroll_x / max_x) * travel;
    }

    return
        (Rectangle)
        {
            x,
            track.y,
            thumb_w,
            track.height
        };
}


static Rectangle hmi_vertical_thumb(
    Rectangle track,
    float viewport_h,
    float max_y
)
{
    float thumb_h =
        track.height *
        (viewport_h / HMI_VIEW_CONTENT_HEIGHT);

    thumb_h =
        hmi_view_clampf(
            thumb_h,
            HMI_SCROLL_MIN_THUMB,
            track.height
        );

    float travel = track.height - thumb_h;
    float y = track.y;

    if (max_y > 0.0f && travel > 0.0f)
    {
        y += (hmi_scroll_y / max_y) * travel;
    }

    return
        (Rectangle)
        {
            track.x,
            y,
            track.width,
            thumb_h
        };
}


static bool hmi_pointer_on_scrollbar(void)
{
    float max_x;
    float max_y;
    float viewport_w;
    float viewport_h;

    hmi_view_limits(
        &max_x,
        &max_y,
        &viewport_w,
        &viewport_h,
        NULL,
        NULL
    );

    Vector2 mouse = GetMousePosition();

    if (max_x > 0.0f)
    {
        if (
            CheckCollisionPointRec(
                mouse,
                hmi_horizontal_track(viewport_w, viewport_h)
            )
        )
        {
            return true;
        }
    }

    if (max_y > 0.0f)
    {
        if (
            CheckCollisionPointRec(
                mouse,
                hmi_vertical_track(viewport_w, viewport_h)
            )
        )
        {
            return true;
        }
    }

    return false;
}


static void hmi_scroll_update(void)
{
    float max_x;
    float max_y;
    float viewport_w;
    float viewport_h;

    hmi_view_limits(
        &max_x,
        &max_y,
        &viewport_w,
        &viewport_h,
        NULL,
        NULL
    );

    if (max_x <= 0.0f)
    {
        hmi_scroll_x = 0.0f;
        hmi_drag_x = false;
    }

    if (max_y <= 0.0f)
    {
        hmi_scroll_y = 0.0f;
        hmi_drag_y = false;
    }

    Vector2 wheel = GetMouseWheelMoveV();

    bool shift =
        IsKeyDown(KEY_LEFT_SHIFT) ||
        IsKeyDown(KEY_RIGHT_SHIFT);

    if (
        max_x > 0.0f &&
        fabsf(wheel.x) > 1e-4f
    )
    {
        hmi_scroll_x -=
            wheel.x *
            HMI_SCROLL_STEP;
    }

    if (fabsf(wheel.y) > 1e-4f)
    {
        if (
            max_x > 0.0f &&
            (shift || max_y <= 0.0f)
        )
        {
            hmi_scroll_x -=
                wheel.y *
                HMI_SCROLL_STEP;
        }
        else if (max_y > 0.0f)
        {
            hmi_scroll_y -=
                wheel.y *
                HMI_SCROLL_STEP;
        }
    }

    Vector2 mouse = GetMousePosition();

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
        if (max_x > 0.0f)
        {
            Rectangle track =
                hmi_horizontal_track(
                    viewport_w,
                    viewport_h
                );

            Rectangle thumb =
                hmi_horizontal_thumb(
                    track,
                    viewport_w,
                    max_x
                );

            if (CheckCollisionPointRec(mouse, thumb))
            {
                hmi_drag_x = true;
                hmi_drag_offset_x =
                    mouse.x - thumb.x;
            }
            else if (CheckCollisionPointRec(mouse, track))
            {
                float travel =
                    track.width - thumb.width;

                float new_thumb_x =
                    hmi_view_clampf(
                        mouse.x - thumb.width * 0.5f,
                        track.x,
                        track.x + travel
                    );

                if (travel > 0.0f)
                {
                    hmi_scroll_x =
                        ((new_thumb_x - track.x) / travel) *
                        max_x;
                }

                hmi_drag_x = true;
                hmi_drag_offset_x =
                    thumb.width * 0.5f;
            }
        }

        if (max_y > 0.0f)
        {
            Rectangle track =
                hmi_vertical_track(
                    viewport_w,
                    viewport_h
                );

            Rectangle thumb =
                hmi_vertical_thumb(
                    track,
                    viewport_h,
                    max_y
                );

            if (CheckCollisionPointRec(mouse, thumb))
            {
                hmi_drag_y = true;
                hmi_drag_offset_y =
                    mouse.y - thumb.y;
            }
            else if (CheckCollisionPointRec(mouse, track))
            {
                float travel =
                    track.height - thumb.height;

                float new_thumb_y =
                    hmi_view_clampf(
                        mouse.y - thumb.height * 0.5f,
                        track.y,
                        track.y + travel
                    );

                if (travel > 0.0f)
                {
                    hmi_scroll_y =
                        ((new_thumb_y - track.y) / travel) *
                        max_y;
                }

                hmi_drag_y = true;
                hmi_drag_offset_y =
                    thumb.height * 0.5f;
            }
        }
    }

    if (hmi_drag_x)
    {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            Rectangle track =
                hmi_horizontal_track(viewport_w, viewport_h);

            Rectangle thumb =
                hmi_horizontal_thumb(track, viewport_w, max_x);

            float travel = track.width - thumb.width;

            float new_thumb_x =
                hmi_view_clampf(
                    mouse.x - hmi_drag_offset_x,
                    track.x,
                    track.x + travel
                );

            if (travel > 0.0f)
            {
                hmi_scroll_x =
                    ((new_thumb_x - track.x) / travel) *
                    max_x;
            }
        }
        else
        {
            hmi_drag_x = false;
        }
    }

    if (hmi_drag_y)
    {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            Rectangle track =
                hmi_vertical_track(viewport_w, viewport_h);

            Rectangle thumb =
                hmi_vertical_thumb(track, viewport_h, max_y);

            float travel = track.height - thumb.height;

            float new_thumb_y =
                hmi_view_clampf(
                    mouse.y - hmi_drag_offset_y,
                    track.y,
                    track.y + travel
                );

            if (travel > 0.0f)
            {
                hmi_scroll_y =
                    ((new_thumb_y - track.y) / travel) *
                    max_y;
            }
        }
        else
        {
            hmi_drag_y = false;
        }
    }

    hmi_scroll_x =
        hmi_view_clampf(hmi_scroll_x, 0.0f, max_x);

    hmi_scroll_y =
        hmi_view_clampf(hmi_scroll_y, 0.0f, max_y);
}


static void hmi_draw_scrollbars(void)
{
    float max_x;
    float max_y;
    float viewport_w;
    float viewport_h;

    hmi_view_limits(
        &max_x,
        &max_y,
        &viewport_w,
        &viewport_h,
        NULL,
        NULL
    );

    Vector2 mouse = GetMousePosition();

    const Color track_color =
        (Color){18, 22, 29, 255};

    const Color thumb_color =
        (Color){75, 88, 108, 255};

    const Color thumb_hover =
        (Color){101, 120, 146, 255};

    const Color thumb_active =
        (Color){88, 166, 255, 255};

    if (max_x > 0.0f)
    {
        Rectangle track =
            hmi_horizontal_track(
                viewport_w,
                viewport_h
            );

        Rectangle thumb =
            hmi_horizontal_thumb(
                track,
                viewport_w,
                max_x
            );

        DrawRectangleRounded(
            track,
            1.0f,
            8,
            track_color
        );

        Color thumb_fill =
            hmi_drag_x
            ? thumb_active
            : CheckCollisionPointRec(mouse, thumb)
                ? thumb_hover
                : thumb_color;

        DrawRectangleRounded(
            thumb,
            1.0f,
            8,
            thumb_fill
        );
    }

    if (max_y > 0.0f)
    {
        Rectangle track =
            hmi_vertical_track(
                viewport_w,
                viewport_h
            );

        Rectangle thumb =
            hmi_vertical_thumb(
                track,
                viewport_h,
                max_y
            );

        DrawRectangleRounded(
            track,
            1.0f,
            8,
            track_color
        );

        Color thumb_fill =
            hmi_drag_y
            ? thumb_active
            : CheckCollisionPointRec(mouse, thumb)
                ? thumb_hover
                : thumb_color;

        DrawRectangleRounded(
            thumb,
            1.0f,
            8,
            thumb_fill
        );
    }
}


/* ============================================================================
 * HOOKS USED BY hmi.c
 * ============================================================================
 */

static int hmi_virtual_width(void)
{
    return
        (int)lroundf(
            HMI_VIEW_LOGICAL_WIDTH *
            HMI_VIEW_SCALE
        );
}


static int hmi_virtual_height(void)
{
    return
        (int)lroundf(
            HMI_VIEW_LOGICAL_HEIGHT *
            HMI_VIEW_SCALE
        );
}


static Vector2 hmi_view_mouse_position(void)
{
    if (
        hmi_drag_x ||
        hmi_drag_y ||
        hmi_pointer_on_scrollbar()
    )
    {
        return
            (Vector2)
            {
                -10000.0f,
                -10000.0f
            };
    }

    float origin_x;
    float origin_y;

    hmi_view_limits(
        NULL,
        NULL,
        NULL,
        NULL,
        &origin_x,
        &origin_y
    );

    Vector2 mouse = GetMousePosition();

    mouse.x =
        mouse.x -
        origin_x +
        hmi_scroll_x;

    mouse.y =
        mouse.y -
        origin_y +
        hmi_scroll_y;

    return mouse;
}


static void hmi_view_begin_mode_2d(
    Camera2D camera
)
{
    hmi_scroll_update();

    float origin_x;
    float origin_y;

    hmi_view_limits(
        NULL,
        NULL,
        NULL,
        NULL,
        &origin_x,
        &origin_y
    );

    camera.offset =
        (Vector2)
        {
            origin_x - hmi_scroll_x,
            origin_y - hmi_scroll_y
        };

    camera.target =
        (Vector2)
        {
            0.0f,
            0.0f
        };

    camera.rotation = 0.0f;
    camera.zoom = HMI_VIEW_SCALE;

    BeginMode2D(camera);
}


static void hmi_view_end_drawing(void)
{
    hmi_draw_scrollbars();
    EndDrawing();
}


/*
 * Apply hooks only after the helpers above are compiled so they continue to
 * call raylib's real functions and cannot recurse into the wrappers.
 */
#define GetScreenWidth()     hmi_virtual_width()
#define GetScreenHeight()    hmi_virtual_height()
#define GetMousePosition()   hmi_view_mouse_position()
#define BeginMode2D(camera)  hmi_view_begin_mode_2d((camera))
#define EndDrawing()         hmi_view_end_drawing()


#endif /* HMI_SCROLL_HOOKS_H */
