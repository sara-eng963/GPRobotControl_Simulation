#ifndef HMI_SCROLL_HOOKS_H
#define HMI_SCROLL_HOOKS_H

/*
 * ============================================================================
 * ROBOT HMI - FIXED-SCALE VIEWPORT + SCROLLING
 * ============================================================================
 *
 * UX POLICY
 * ---------
 * The HMI has ONE operator-facing visual scale. The window is a viewport only.
 *
 *   - NO user zoom.
 *   - NO auto-fit-to-width.
 *   - NO shrinking when the window is resized.
 *   - The 1440 x 900 layout stays at native 100% size.
 *   - If the window is smaller, scrollbars expose the hidden content.
 *   - If the window is larger, the HMI is centered.
 *
 * The previous 90% view made the typography smaller without solving the real
 * readability problem. This version returns the canvas to 100% and increases
 * text rendering independently by 20%, so labels and values are readable while
 * the physical control layout remains stable.
 *
 * Navigation
 * ----------
 *   Mouse wheel                vertical scroll
 *   Shift + mouse wheel        horizontal scroll
 *   Horizontal wheel/trackpad  horizontal scroll
 *   Drag scrollbar thumb       direct navigation
 *
 * If only horizontal overflow exists, the normal mouse wheel scrolls
 * horizontally, matching the original HMI behavior.
 * ============================================================================
 */

#include "raylib.h"

#include <math.h>
#include <stdbool.h>

#define HMI_CANVAS_WIDTH        1440.0f
#define HMI_CANVAS_HEIGHT        900.0f

/* Fixed design scale: never changes with window size. */
#define HMI_VIEW_SCALE             1.00f

/* Typography is deliberately larger than the original UI. */
#define HMI_TEXT_SCALE             1.20f

#define HMI_CONTENT_WIDTH  (HMI_CANVAS_WIDTH  * HMI_VIEW_SCALE)
#define HMI_CONTENT_HEIGHT (HMI_CANVAS_HEIGHT * HMI_VIEW_SCALE)

#define HMI_SCROLLBAR_SIZE         13.0f
#define HMI_SCROLL_STEP            64.0f
#define HMI_SCROLL_MIN_THUMB       52.0f
#define HMI_SCROLL_MARGIN           3.0f

static float hmi_scroll_x = 0.0f;
static float hmi_scroll_y = 0.0f;

static bool hmi_drag_x = false;
static bool hmi_drag_y = false;
static float hmi_drag_offset_x = 0.0f;
static float hmi_drag_offset_y = 0.0f;


static float hmi_clampf(
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


typedef struct
{
    float viewportW;
    float viewportH;

    float maxScrollX;
    float maxScrollY;

    float originX;
    float originY;

    bool needX;
    bool needY;

} HMIViewport;


static HMIViewport hmi_viewport(void)
{
    HMIViewport view = {0};

    float actualW = (float)GetScreenWidth();
    float actualH = (float)GetScreenHeight();

    bool needX =
        HMI_CONTENT_WIDTH > actualW;

    bool needY =
        HMI_CONTENT_HEIGHT > actualH;

    float viewportW =
        actualW -
        (needY ? HMI_SCROLLBAR_SIZE : 0.0f);

    float viewportH =
        actualH -
        (needX ? HMI_SCROLLBAR_SIZE : 0.0f);

    /* One scrollbar may cause overflow on the other axis. */
    needX =
        HMI_CONTENT_WIDTH > viewportW;

    needY =
        HMI_CONTENT_HEIGHT > viewportH;

    viewportW =
        actualW -
        (needY ? HMI_SCROLLBAR_SIZE : 0.0f);

    viewportH =
        actualH -
        (needX ? HMI_SCROLLBAR_SIZE : 0.0f);

    if (viewportW < 1.0f)
    {
        viewportW = 1.0f;
    }

    if (viewportH < 1.0f)
    {
        viewportH = 1.0f;
    }

    view.viewportW = viewportW;
    view.viewportH = viewportH;
    view.needX = needX;
    view.needY = needY;

    view.maxScrollX =
        fmaxf(
            0.0f,
            HMI_CONTENT_WIDTH - viewportW
        );

    view.maxScrollY =
        fmaxf(
            0.0f,
            HMI_CONTENT_HEIGHT - viewportH
        );

    if (view.maxScrollX > 0.0f)
    {
        view.originX =
            -hmi_scroll_x;
    }
    else
    {
        view.originX =
            (viewportW - HMI_CONTENT_WIDTH) * 0.5f;
    }

    if (view.maxScrollY > 0.0f)
    {
        view.originY =
            -hmi_scroll_y;
    }
    else
    {
        view.originY =
            (viewportH - HMI_CONTENT_HEIGHT) * 0.5f;
    }

    return view;
}


static Rectangle hmi_horizontal_track(
    HMIViewport view
)
{
    return
        (Rectangle)
        {
            HMI_SCROLL_MARGIN,
            view.viewportH + 2.0f,
            fmaxf(
                1.0f,
                view.viewportW -
                2.0f * HMI_SCROLL_MARGIN
            ),
            HMI_SCROLLBAR_SIZE - 4.0f
        };
}


static Rectangle hmi_vertical_track(
    HMIViewport view
)
{
    return
        (Rectangle)
        {
            view.viewportW + 2.0f,
            HMI_SCROLL_MARGIN,
            HMI_SCROLLBAR_SIZE - 4.0f,
            fmaxf(
                1.0f,
                view.viewportH -
                2.0f * HMI_SCROLL_MARGIN
            )
        };
}


static Rectangle hmi_horizontal_thumb(
    HMIViewport view,
    Rectangle track
)
{
    float width =
        track.width *
        (view.viewportW / HMI_CONTENT_WIDTH);

    width =
        hmi_clampf(
            width,
            HMI_SCROLL_MIN_THUMB,
            track.width
        );

    float travel =
        track.width - width;

    float x = track.x;

    if (
        view.maxScrollX > 0.0f &&
        travel > 0.0f
    )
    {
        x +=
            (hmi_scroll_x / view.maxScrollX) *
            travel;
    }

    return
        (Rectangle)
        {
            x,
            track.y,
            width,
            track.height
        };
}


static Rectangle hmi_vertical_thumb(
    HMIViewport view,
    Rectangle track
)
{
    float height =
        track.height *
        (view.viewportH / HMI_CONTENT_HEIGHT);

    height =
        hmi_clampf(
            height,
            HMI_SCROLL_MIN_THUMB,
            track.height
        );

    float travel =
        track.height - height;

    float y = track.y;

    if (
        view.maxScrollY > 0.0f &&
        travel > 0.0f
    )
    {
        y +=
            (hmi_scroll_y / view.maxScrollY) *
            travel;
    }

    return
        (Rectangle)
        {
            track.x,
            y,
            track.width,
            height
        };
}


static bool hmi_mouse_over_scrollbar(void)
{
    HMIViewport view =
        hmi_viewport();

    Vector2 mouse =
        GetMousePosition();

    if (view.needX)
    {
        if (
            CheckCollisionPointRec(
                mouse,
                hmi_horizontal_track(view)
            )
        )
        {
            return true;
        }
    }

    if (view.needY)
    {
        if (
            CheckCollisionPointRec(
                mouse,
                hmi_vertical_track(view)
            )
        )
        {
            return true;
        }
    }

    return false;
}


static void hmi_update_scroll(void)
{
    HMIViewport view =
        hmi_viewport();

    if (!view.needX)
    {
        hmi_scroll_x = 0.0f;
        hmi_drag_x = false;
    }

    if (!view.needY)
    {
        hmi_scroll_y = 0.0f;
        hmi_drag_y = false;
    }

    Vector2 wheel =
        GetMouseWheelMoveV();

    bool shift =
        IsKeyDown(KEY_LEFT_SHIFT) ||
        IsKeyDown(KEY_RIGHT_SHIFT);

    if (
        view.maxScrollX > 0.0f &&
        fabsf(wheel.x) > 0.0f
    )
    {
        hmi_scroll_x -=
            wheel.x *
            HMI_SCROLL_STEP;
    }

    if (fabsf(wheel.y) > 0.0f)
    {
        if (
            view.maxScrollX > 0.0f &&
            (
                shift ||
                view.maxScrollY <= 0.0f
            )
        )
        {
            hmi_scroll_x -=
                wheel.y *
                HMI_SCROLL_STEP;
        }
        else if (view.maxScrollY > 0.0f)
        {
            hmi_scroll_y -=
                wheel.y *
                HMI_SCROLL_STEP;
        }
    }

    Vector2 mouse =
        GetMousePosition();

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
        if (view.needX)
        {
            Rectangle track =
                hmi_horizontal_track(view);

            Rectangle thumb =
                hmi_horizontal_thumb(
                    view,
                    track
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

                float newX =
                    hmi_clampf(
                        mouse.x - thumb.width * 0.5f,
                        track.x,
                        track.x + travel
                    );

                if (travel > 0.0f)
                {
                    hmi_scroll_x =
                        ((newX - track.x) / travel) *
                        view.maxScrollX;
                }

                hmi_drag_x = true;
                hmi_drag_offset_x =
                    thumb.width * 0.5f;
            }
        }

        if (view.needY)
        {
            Rectangle track =
                hmi_vertical_track(view);

            Rectangle thumb =
                hmi_vertical_thumb(
                    view,
                    track
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

                float newY =
                    hmi_clampf(
                        mouse.y - thumb.height * 0.5f,
                        track.y,
                        track.y + travel
                    );

                if (travel > 0.0f)
                {
                    hmi_scroll_y =
                        ((newY - track.y) / travel) *
                        view.maxScrollY;
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
                hmi_horizontal_track(view);

            Rectangle thumb =
                hmi_horizontal_thumb(
                    view,
                    track
                );

            float travel =
                track.width - thumb.width;

            float newX =
                hmi_clampf(
                    mouse.x - hmi_drag_offset_x,
                    track.x,
                    track.x + travel
                );

            if (travel > 0.0f)
            {
                hmi_scroll_x =
                    ((newX - track.x) / travel) *
                    view.maxScrollX;
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
                hmi_vertical_track(view);

            Rectangle thumb =
                hmi_vertical_thumb(
                    view,
                    track
                );

            float travel =
                track.height - thumb.height;

            float newY =
                hmi_clampf(
                    mouse.y - hmi_drag_offset_y,
                    track.y,
                    track.y + travel
                );

            if (travel > 0.0f)
            {
                hmi_scroll_y =
                    ((newY - track.y) / travel) *
                    view.maxScrollY;
            }
        }
        else
        {
            hmi_drag_y = false;
        }
    }

    hmi_scroll_x =
        hmi_clampf(
            hmi_scroll_x,
            0.0f,
            view.maxScrollX
        );

    hmi_scroll_y =
        hmi_clampf(
            hmi_scroll_y,
            0.0f,
            view.maxScrollY
        );
}


static void hmi_draw_scrollbars(void)
{
    HMIViewport view =
        hmi_viewport();

    const Color trackColor =
        (Color){18, 23, 31, 255};

    const Color thumbColor =
        (Color){83, 99, 123, 255};

    const Color hoverColor =
        (Color){104, 126, 157, 255};

    const Color activeColor =
        (Color){88, 166, 255, 255};

    Vector2 mouse =
        GetMousePosition();

    if (view.needX)
    {
        Rectangle track =
            hmi_horizontal_track(view);

        Rectangle thumb =
            hmi_horizontal_thumb(
                view,
                track
            );

        bool hovered =
            CheckCollisionPointRec(
                mouse,
                thumb
            );

        DrawRectangleRounded(
            track,
            1.0f,
            8,
            trackColor
        );

        DrawRectangleRounded(
            thumb,
            1.0f,
            8,
            hmi_drag_x
                ? activeColor
                : hovered
                    ? hoverColor
                    : thumbColor
        );
    }

    if (view.needY)
    {
        Rectangle track =
            hmi_vertical_track(view);

        Rectangle thumb =
            hmi_vertical_thumb(
                view,
                track
            );

        bool hovered =
            CheckCollisionPointRec(
                mouse,
                thumb
            );

        DrawRectangleRounded(
            track,
            1.0f,
            8,
            trackColor
        );

        DrawRectangleRounded(
            thumb,
            1.0f,
            8,
            hmi_drag_y
                ? activeColor
                : hovered
                    ? hoverColor
                    : thumbColor
        );
    }
}


/* ============================================================================
 * HMI.C VIEW HOOKS
 * ============================================================================
 */

/*
 * hmi.c uses GetScreenWidth/Height to calculate a fit scale. Returning the
 * authored canvas size prevents that calculation from shrinking the interface.
 */
static int hmi_virtual_width(void)
{
    return (int)HMI_CANVAS_WIDTH;
}


static int hmi_virtual_height(void)
{
    return (int)HMI_CANVAS_HEIGHT;
}


/*
 * Convert physical window coordinates into HMI logical coordinates.
 */
static Vector2 hmi_view_mouse_position(void)
{
    if (
        hmi_drag_x ||
        hmi_drag_y ||
        hmi_mouse_over_scrollbar()
    )
    {
        return
            (Vector2)
            {
                -10000.0f,
                -10000.0f
            };
    }

    HMIViewport view =
        hmi_viewport();

    Vector2 mouse =
        GetMousePosition();

    return
        (Vector2)
        {
            (mouse.x - view.originX) /
                HMI_VIEW_SCALE,

            (mouse.y - view.originY) /
                HMI_VIEW_SCALE
        };
}


static void hmi_view_begin_mode_2d(
    Camera2D camera
)
{
    hmi_update_scroll();

    HMIViewport view =
        hmi_viewport();

    camera.offset =
        (Vector2)
        {
            view.originX,
            view.originY
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


/* ============================================================================
 * TYPOGRAPHY HOOKS
 * ============================================================================
 *
 * Increase text independently from panel/control geometry. This fixes the real
 * readability issue without zooming the entire application or increasing the
 * amount of scrolling.
 * ============================================================================
 */

static void hmi_draw_text_ex(
    Font font,
    const char *text,
    Vector2 position,
    float fontSize,
    float spacing,
    Color tint
)
{
    DrawTextEx(
        font,
        text,
        position,
        fontSize * HMI_TEXT_SCALE,
        spacing * HMI_TEXT_SCALE,
        tint
    );
}


static Vector2 hmi_measure_text_ex(
    Font font,
    const char *text,
    float fontSize,
    float spacing
)
{
    return
        MeasureTextEx(
            font,
            text,
            fontSize * HMI_TEXT_SCALE,
            spacing * HMI_TEXT_SCALE
        );
}


/*
 * Apply hooks only after their implementations above have been compiled, so
 * calls inside these wrappers still reach the original raylib functions.
 */
#define GetScreenWidth()       hmi_virtual_width()
#define GetScreenHeight()      hmi_virtual_height()
#define GetMousePosition()     hmi_view_mouse_position()
#define BeginMode2D(camera)    hmi_view_begin_mode_2d((camera))
#define EndDrawing()           hmi_view_end_drawing()
#define DrawTextEx(f,t,p,s,sp,c) hmi_draw_text_ex((f),(t),(p),(s),(sp),(c))
#define MeasureTextEx(f,t,s,sp)  hmi_measure_text_ex((f),(t),(s),(sp))

#endif /* HMI_SCROLL_HOOKS_H */
