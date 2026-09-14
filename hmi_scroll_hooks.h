#ifndef HMI_SCROLL_HOOKS_H
#define HMI_SCROLL_HOOKS_H

/*
 * ============================================================================
 * HMI FIXED-SIZE SCROLL LAYER
 * ============================================================================
 *
 * The HMI is authored on a 1440 x 900 logical canvas. When the desktop window
 * is resized smaller than that canvas, keep the UI at its normal 1:1 size and
 * scroll around it instead of shrinking the text and controls.
 *
 * Controls:
 *
 *      Mouse wheel          scroll vertically
 *      Shift + mouse wheel  scroll horizontally
 *
 * If only horizontal scrolling is required, the normal mouse wheel scrolls
 * horizontally automatically (the old HMI behavior).
 *
 * Both scrollbars can also be clicked or dragged directly.
 *
 * This header is force-included only for the mock_hmi target. It changes only
 * the desktop view; waypoint values, UDP packets and controller behavior are
 * untouched.
 * ============================================================================
 */

#include "raylib.h"

#include <math.h>
#include <stdbool.h>

#define HMI_SCROLL_CONTENT_WIDTH   1440.0f
#define HMI_SCROLL_CONTENT_HEIGHT   900.0f
#define HMI_SCROLLBAR_SIZE           14.0f
#define HMI_SCROLL_STEP              70.0f
#define HMI_SCROLL_MIN_THUMB          48.0f

static float hmi_scroll_x = 0.0f;
static float hmi_scroll_y = 0.0f;

static bool hmi_scroll_drag_x = false;
static bool hmi_scroll_drag_y = false;
static float hmi_scroll_drag_offset_x = 0.0f;
static float hmi_scroll_drag_offset_y = 0.0f;


static float hmi_scroll_clampf(
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


static void hmi_scroll_limits(
    float *max_x,
    float *max_y,
    float *viewport_w,
    float *viewport_h
)
{
    float actual_w =
        (float)GetScreenWidth();

    float actual_h =
        (float)GetScreenHeight();

    bool need_x =
        actual_w < HMI_SCROLL_CONTENT_WIDTH;

    bool need_y =
        actual_h < HMI_SCROLL_CONTENT_HEIGHT;

    float usable_w =
        actual_w -
        (need_y ? HMI_SCROLLBAR_SIZE : 0.0f);

    float usable_h =
        actual_h -
        (need_x ? HMI_SCROLLBAR_SIZE : 0.0f);

    /* A scrollbar can make the other axis overflow, so resolve once more. */
    need_x =
        usable_w < HMI_SCROLL_CONTENT_WIDTH;

    need_y =
        usable_h < HMI_SCROLL_CONTENT_HEIGHT;

    usable_w =
        actual_w -
        (need_y ? HMI_SCROLLBAR_SIZE : 0.0f);

    usable_h =
        actual_h -
        (need_x ? HMI_SCROLLBAR_SIZE : 0.0f);

    if (usable_w < 1.0f)
    {
        usable_w = 1.0f;
    }

    if (usable_h < 1.0f)
    {
        usable_h = 1.0f;
    }

    if (viewport_w != NULL)
    {
        *viewport_w = usable_w;
    }

    if (viewport_h != NULL)
    {
        *viewport_h = usable_h;
    }

    if (max_x != NULL)
    {
        *max_x =
            fmaxf(
                0.0f,
                HMI_SCROLL_CONTENT_WIDTH - usable_w
            );
    }

    if (max_y != NULL)
    {
        *max_y =
            fmaxf(
                0.0f,
                HMI_SCROLL_CONTENT_HEIGHT - usable_h
            );
    }
}


static Rectangle hmi_scroll_horizontal_track(
    float viewport_w,
    float viewport_h
)
{
    return
        (Rectangle)
        {
            4.0f,
            viewport_h + 2.0f,
            fmaxf(1.0f, viewport_w - 8.0f),
            HMI_SCROLLBAR_SIZE - 4.0f
        };
}


static Rectangle hmi_scroll_vertical_track(
    float viewport_w,
    float viewport_h
)
{
    return
        (Rectangle)
        {
            viewport_w + 2.0f,
            4.0f,
            HMI_SCROLLBAR_SIZE - 4.0f,
            fmaxf(1.0f, viewport_h - 8.0f)
        };
}


static Rectangle hmi_scroll_horizontal_thumb(
    Rectangle track,
    float viewport_w,
    float max_x
)
{
    float thumb_w =
        track.width *
        (viewport_w / HMI_SCROLL_CONTENT_WIDTH);

    thumb_w =
        hmi_scroll_clampf(
            thumb_w,
            HMI_SCROLL_MIN_THUMB,
            track.width
        );

    float travel =
        track.width - thumb_w;

    float x =
        track.x;

    if (
        max_x > 0.0f &&
        travel > 0.0f
    )
    {
        x +=
            (hmi_scroll_x / max_x) *
            travel;
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


static Rectangle hmi_scroll_vertical_thumb(
    Rectangle track,
    float viewport_h,
    float max_y
)
{
    float thumb_h =
        track.height *
        (viewport_h / HMI_SCROLL_CONTENT_HEIGHT);

    thumb_h =
        hmi_scroll_clampf(
            thumb_h,
            HMI_SCROLL_MIN_THUMB,
            track.height
        );

    float travel =
        track.height - thumb_h;

    float y =
        track.y;

    if (
        max_y > 0.0f &&
        travel > 0.0f
    )
    {
        y +=
            (hmi_scroll_y / max_y) *
            travel;
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


static bool hmi_scroll_pointer_on_bar(void)
{
    float max_x;
    float max_y;
    float viewport_w;
    float viewport_h;

    hmi_scroll_limits(
        &max_x,
        &max_y,
        &viewport_w,
        &viewport_h
    );

    Vector2 mouse =
        GetMousePosition();

    if (max_x > 0.0f)
    {
        Rectangle track =
            hmi_scroll_horizontal_track(
                viewport_w,
                viewport_h
            );

        if (CheckCollisionPointRec(mouse, track))
        {
            return true;
        }
    }

    if (max_y > 0.0f)
    {
        Rectangle track =
            hmi_scroll_vertical_track(
                viewport_w,
                viewport_h
            );

        if (CheckCollisionPointRec(mouse, track))
        {
            return true;
        }
    }

    return false;
}


static void hmi_scroll_update_input(void)
{
    float max_x;
    float max_y;
    float viewport_w;
    float viewport_h;

    hmi_scroll_limits(
        &max_x,
        &max_y,
        &viewport_w,
        &viewport_h
    );

    if (max_x <= 0.0f)
    {
        hmi_scroll_x = 0.0f;
        hmi_scroll_drag_x = false;
    }

    if (max_y <= 0.0f)
    {
        hmi_scroll_y = 0.0f;
        hmi_scroll_drag_y = false;
    }


    /* ------------------------------------------------------------------------
     * Wheel scrolling.
     * ------------------------------------------------------------------------
     * If the content only overflows horizontally, preserve the original HMI
     * behavior: the normal wheel moves left/right. When both axes overflow,
     * normal wheel is vertical and Shift + wheel is horizontal.
     * ------------------------------------------------------------------------
     */

    float wheel =
        GetMouseWheelMove();

    bool shift =
        IsKeyDown(KEY_LEFT_SHIFT) ||
        IsKeyDown(KEY_RIGHT_SHIFT);

    if (wheel != 0.0f)
    {
        if (
            max_x > 0.0f &&
            (shift || max_y <= 0.0f)
        )
        {
            hmi_scroll_x -=
                wheel *
                HMI_SCROLL_STEP;
        }
        else if (max_y > 0.0f)
        {
            hmi_scroll_y -=
                wheel *
                HMI_SCROLL_STEP;
        }
    }


    Vector2 mouse =
        GetMousePosition();


    if (
        IsMouseButtonPressed(
            MOUSE_BUTTON_LEFT
        )
    )
    {
        if (max_x > 0.0f)
        {
            Rectangle track =
                hmi_scroll_horizontal_track(
                    viewport_w,
                    viewport_h
                );

            Rectangle thumb =
                hmi_scroll_horizontal_thumb(
                    track,
                    viewport_w,
                    max_x
                );

            if (CheckCollisionPointRec(mouse, thumb))
            {
                hmi_scroll_drag_x = true;
                hmi_scroll_drag_offset_x =
                    mouse.x - thumb.x;
            }
            else if (CheckCollisionPointRec(mouse, track))
            {
                float thumb_x =
                    mouse.x - thumb.width * 0.5f;

                float travel =
                    track.width - thumb.width;

                thumb_x =
                    hmi_scroll_clampf(
                        thumb_x,
                        track.x,
                        track.x + travel
                    );

                if (travel > 0.0f)
                {
                    hmi_scroll_x =
                        ((thumb_x - track.x) / travel) *
                        max_x;
                }

                hmi_scroll_drag_x = true;
                hmi_scroll_drag_offset_x =
                    thumb.width * 0.5f;
            }
        }

        if (max_y > 0.0f)
        {
            Rectangle track =
                hmi_scroll_vertical_track(
                    viewport_w,
                    viewport_h
                );

            Rectangle thumb =
                hmi_scroll_vertical_thumb(
                    track,
                    viewport_h,
                    max_y
                );

            if (CheckCollisionPointRec(mouse, thumb))
            {
                hmi_scroll_drag_y = true;
                hmi_scroll_drag_offset_y =
                    mouse.y - thumb.y;
            }
            else if (CheckCollisionPointRec(mouse, track))
            {
                float thumb_y =
                    mouse.y - thumb.height * 0.5f;

                float travel =
                    track.height - thumb.height;

                thumb_y =
                    hmi_scroll_clampf(
                        thumb_y,
                        track.y,
                        track.y + travel
                    );

                if (travel > 0.0f)
                {
                    hmi_scroll_y =
                        ((thumb_y - track.y) / travel) *
                        max_y;
                }

                hmi_scroll_drag_y = true;
                hmi_scroll_drag_offset_y =
                    thumb.height * 0.5f;
            }
        }
    }


    if (hmi_scroll_drag_x)
    {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            Rectangle track =
                hmi_scroll_horizontal_track(
                    viewport_w,
                    viewport_h
                );

            Rectangle thumb =
                hmi_scroll_horizontal_thumb(
                    track,
                    viewport_w,
                    max_x
                );

            float travel =
                track.width - thumb.width;

            float thumb_x =
                hmi_scroll_clampf(
                    mouse.x - hmi_scroll_drag_offset_x,
                    track.x,
                    track.x + travel
                );

            if (travel > 0.0f)
            {
                hmi_scroll_x =
                    ((thumb_x - track.x) / travel) *
                    max_x;
            }
        }
        else
        {
            hmi_scroll_drag_x = false;
        }
    }


    if (hmi_scroll_drag_y)
    {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            Rectangle track =
                hmi_scroll_vertical_track(
                    viewport_w,
                    viewport_h
                );

            Rectangle thumb =
                hmi_scroll_vertical_thumb(
                    track,
                    viewport_h,
                    max_y
                );

            float travel =
                track.height - thumb.height;

            float thumb_y =
                hmi_scroll_clampf(
                    mouse.y - hmi_scroll_drag_offset_y,
                    track.y,
                    track.y + travel
                );

            if (travel > 0.0f)
            {
                hmi_scroll_y =
                    ((thumb_y - track.y) / travel) *
                    max_y;
            }
        }
        else
        {
            hmi_scroll_drag_y = false;
        }
    }


    hmi_scroll_x =
        hmi_scroll_clampf(
            hmi_scroll_x,
            0.0f,
            max_x
        );

    hmi_scroll_y =
        hmi_scroll_clampf(
            hmi_scroll_y,
            0.0f,
            max_y
        );
}


static void hmi_scroll_draw_bars(void)
{
    float max_x;
    float max_y;
    float viewport_w;
    float viewport_h;

    hmi_scroll_limits(
        &max_x,
        &max_y,
        &viewport_w,
        &viewport_h
    );

    const Color track_color =
        (Color){22, 27, 35, 255};

    const Color thumb_color =
        (Color){76, 91, 114, 255};

    const Color thumb_active =
        (Color){88, 166, 255, 255};

    if (max_x > 0.0f)
    {
        Rectangle track =
            hmi_scroll_horizontal_track(
                viewport_w,
                viewport_h
            );

        Rectangle thumb =
            hmi_scroll_horizontal_thumb(
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

        DrawRectangleRounded(
            thumb,
            1.0f,
            8,
            hmi_scroll_drag_x
                ? thumb_active
                : thumb_color
        );
    }

    if (max_y > 0.0f)
    {
        Rectangle track =
            hmi_scroll_vertical_track(
                viewport_w,
                viewport_h
            );

        Rectangle thumb =
            hmi_scroll_vertical_thumb(
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

        DrawRectangleRounded(
            thumb,
            1.0f,
            8,
            hmi_scroll_drag_y
                ? thumb_active
                : thumb_color
        );
    }
}


/* ============================================================================
 * HOOKS USED BY hmi.c
 * ============================================================================
 */

/*
 * Make hmi.c believe its full logical canvas is always available. Its existing
 * fit calculation therefore resolves to ui_scale = 1 and ui_origin = (0,0).
 */
static int hmi_scroll_virtual_width(void)
{
    return (int)HMI_SCROLL_CONTENT_WIDTH;
}


static int hmi_scroll_virtual_height(void)
{
    return (int)HMI_SCROLL_CONTENT_HEIGHT;
}


/*
 * UI hit-testing needs mouse coordinates in logical-canvas space. Scrollbars
 * are screen-space controls, so hide their clicks from the HMI underneath.
 */
static Vector2 hmi_scroll_mouse_position(void)
{
    if (
        hmi_scroll_drag_x ||
        hmi_scroll_drag_y ||
        hmi_scroll_pointer_on_bar()
    )
    {
        return
            (Vector2)
            {
                -10000.0f,
                -10000.0f
            };
    }

    Vector2 mouse =
        GetMousePosition();

    mouse.x +=
        hmi_scroll_x;

    mouse.y +=
        hmi_scroll_y;

    return mouse;
}


static void hmi_scroll_begin_mode_2d(
    Camera2D camera
)
{
    hmi_scroll_update_input();

    camera.offset =
        (Vector2)
        {
            -hmi_scroll_x,
            -hmi_scroll_y
        };

    camera.target =
        (Vector2)
        {
            0.0f,
            0.0f
        };

    camera.rotation = 0.0f;
    camera.zoom = 1.0f;

    BeginMode2D(camera);
}


static void hmi_scroll_end_drawing(void)
{
    hmi_scroll_draw_bars();
    EndDrawing();
}


/*
 * Apply hooks only after all helper functions above have compiled, preventing
 * recursion into our own wrappers.
 */
#define GetScreenWidth()     hmi_scroll_virtual_width()
#define GetScreenHeight()    hmi_scroll_virtual_height()
#define GetMousePosition()   hmi_scroll_mouse_position()
#define BeginMode2D(camera)  hmi_scroll_begin_mode_2d((camera))
#define EndDrawing()         hmi_scroll_end_drawing()


#endif /* HMI_SCROLL_HOOKS_H */
