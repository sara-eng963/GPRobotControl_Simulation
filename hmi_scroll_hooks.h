#ifndef HMI_SCROLL_HOOKS_H
#define HMI_SCROLL_HOOKS_H

/*
 * ============================================================================
 * HMI FIXED-SCALE VIEWPORT / SCROLL LAYER
 * ============================================================================
 *
 * UX RULES
 * --------
 * 1. The HMI has ONE visual scale: its native 1440 x 900 design size.
 * 2. Resizing the desktop window NEVER zooms or shrinks the interface.
 * 3. If the window is smaller than the design canvas, the window becomes a
 *    viewport and scrollbars appear only on the overflowing axes.
 * 4. If the window is larger than the design canvas, the canvas is centered.
 * 5. Mouse hit-testing follows the scrolled viewport exactly.
 *
 * Navigation
 * ----------
 *      Mouse wheel                vertical scroll
 *      Shift + mouse wheel        horizontal scroll
 *      Horizontal trackpad wheel  horizontal scroll
 *      Drag scrollbar thumb       direct navigation
 *      Click scrollbar track      jump toward that position
 *
 * When only one axis overflows, the normal wheel follows that axis. There is
 * deliberately NO zoom gesture, zoom hotkey, zoom factor, or auto-fit scaling.
 *
 * This header is force-included only for mock_hmi. It changes presentation
 * only; trajectory values, UDP packets and controller behavior are untouched.
 * ============================================================================
 */

#include "raylib.h"

#include <math.h>
#include <stdbool.h>


#define HMI_VIEW_CONTENT_WIDTH     1440.0f
#define HMI_VIEW_CONTENT_HEIGHT     900.0f

#define HMI_VIEW_SCROLLBAR_GUTTER     14.0f
#define HMI_VIEW_TRACK_INSET            3.0f
#define HMI_VIEW_SCROLL_STEP            62.0f
#define HMI_VIEW_MIN_THUMB              52.0f


static float hmi_view_scroll_x = 0.0f;
static float hmi_view_scroll_y = 0.0f;

static bool hmi_view_drag_x = false;
static bool hmi_view_drag_y = false;

static float hmi_view_drag_offset_x = 0.0f;
static float hmi_view_drag_offset_y = 0.0f;


typedef struct
{
    float viewportWidth;
    float viewportHeight;

    float maxScrollX;
    float maxScrollY;

    bool showHorizontal;
    bool showVertical;

} HmiViewportMetrics;


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


/*
 * Resolve viewport dimensions and scrollbar visibility. A scrollbar on one
 * axis can create overflow on the other, so the calculation is stabilized in
 * two passes.
 */
static HmiViewportMetrics hmi_view_metrics(void)
{
    HmiViewportMetrics metrics = {0};

    float windowWidth =
        (float)GetScreenWidth();

    float windowHeight =
        (float)GetScreenHeight();

    if (windowWidth < 1.0f)
    {
        windowWidth = 1.0f;
    }

    if (windowHeight < 1.0f)
    {
        windowHeight = 1.0f;
    }

    bool horizontal =
        windowWidth < HMI_VIEW_CONTENT_WIDTH;

    bool vertical =
        windowHeight < HMI_VIEW_CONTENT_HEIGHT;

    for (int pass = 0; pass < 2; pass++)
    {
        float availableWidth =
            windowWidth -
            (vertical ? HMI_VIEW_SCROLLBAR_GUTTER : 0.0f);

        float availableHeight =
            windowHeight -
            (horizontal ? HMI_VIEW_SCROLLBAR_GUTTER : 0.0f);

        horizontal =
            availableWidth < HMI_VIEW_CONTENT_WIDTH;

        vertical =
            availableHeight < HMI_VIEW_CONTENT_HEIGHT;
    }

    metrics.showHorizontal = horizontal;
    metrics.showVertical = vertical;

    metrics.viewportWidth =
        windowWidth -
        (vertical ? HMI_VIEW_SCROLLBAR_GUTTER : 0.0f);

    metrics.viewportHeight =
        windowHeight -
        (horizontal ? HMI_VIEW_SCROLLBAR_GUTTER : 0.0f);

    if (metrics.viewportWidth < 1.0f)
    {
        metrics.viewportWidth = 1.0f;
    }

    if (metrics.viewportHeight < 1.0f)
    {
        metrics.viewportHeight = 1.0f;
    }

    metrics.maxScrollX =
        fmaxf(
            0.0f,
            HMI_VIEW_CONTENT_WIDTH -
            metrics.viewportWidth
        );

    metrics.maxScrollY =
        fmaxf(
            0.0f,
            HMI_VIEW_CONTENT_HEIGHT -
            metrics.viewportHeight
        );

    return metrics;
}


static Rectangle hmi_view_horizontal_track(
    HmiViewportMetrics metrics
)
{
    return
        (Rectangle)
        {
            HMI_VIEW_TRACK_INSET,
            metrics.viewportHeight + HMI_VIEW_TRACK_INSET,
            fmaxf(
                1.0f,
                metrics.viewportWidth -
                2.0f * HMI_VIEW_TRACK_INSET
            ),
            HMI_VIEW_SCROLLBAR_GUTTER -
                2.0f * HMI_VIEW_TRACK_INSET
        };
}


static Rectangle hmi_view_vertical_track(
    HmiViewportMetrics metrics
)
{
    return
        (Rectangle)
        {
            metrics.viewportWidth + HMI_VIEW_TRACK_INSET,
            HMI_VIEW_TRACK_INSET,
            HMI_VIEW_SCROLLBAR_GUTTER -
                2.0f * HMI_VIEW_TRACK_INSET,
            fmaxf(
                1.0f,
                metrics.viewportHeight -
                2.0f * HMI_VIEW_TRACK_INSET
            )
        };
}


static Rectangle hmi_view_horizontal_thumb(
    HmiViewportMetrics metrics,
    Rectangle track
)
{
    float thumbWidth =
        track.width *
        (
            metrics.viewportWidth /
            HMI_VIEW_CONTENT_WIDTH
        );

    thumbWidth =
        hmi_view_clampf(
            thumbWidth,
            HMI_VIEW_MIN_THUMB,
            track.width
        );

    float travel =
        track.width -
        thumbWidth;

    float thumbX =
        track.x;

    if (
        metrics.maxScrollX > 0.0f &&
        travel > 0.0f
    )
    {
        thumbX +=
            (
                hmi_view_scroll_x /
                metrics.maxScrollX
            ) *
            travel;
    }

    return
        (Rectangle)
        {
            thumbX,
            track.y,
            thumbWidth,
            track.height
        };
}


static Rectangle hmi_view_vertical_thumb(
    HmiViewportMetrics metrics,
    Rectangle track
)
{
    float thumbHeight =
        track.height *
        (
            metrics.viewportHeight /
            HMI_VIEW_CONTENT_HEIGHT
        );

    thumbHeight =
        hmi_view_clampf(
            thumbHeight,
            HMI_VIEW_MIN_THUMB,
            track.height
        );

    float travel =
        track.height -
        thumbHeight;

    float thumbY =
        track.y;

    if (
        metrics.maxScrollY > 0.0f &&
        travel > 0.0f
    )
    {
        thumbY +=
            (
                hmi_view_scroll_y /
                metrics.maxScrollY
            ) *
            travel;
    }

    return
        (Rectangle)
        {
            track.x,
            thumbY,
            track.width,
            thumbHeight
        };
}


static bool hmi_view_pointer_on_scrollbar(void)
{
    HmiViewportMetrics metrics =
        hmi_view_metrics();

    Vector2 mouse =
        GetMousePosition();

    if (metrics.showHorizontal)
    {
        Rectangle track =
            hmi_view_horizontal_track(
                metrics
            );

        if (
            CheckCollisionPointRec(
                mouse,
                track
            )
        )
        {
            return true;
        }
    }

    if (metrics.showVertical)
    {
        Rectangle track =
            hmi_view_vertical_track(
                metrics
            );

        if (
            CheckCollisionPointRec(
                mouse,
                track
            )
        )
        {
            return true;
        }
    }

    return false;
}


static void hmi_view_clamp_scroll(
    HmiViewportMetrics metrics
)
{
    hmi_view_scroll_x =
        hmi_view_clampf(
            hmi_view_scroll_x,
            0.0f,
            metrics.maxScrollX
        );

    hmi_view_scroll_y =
        hmi_view_clampf(
            hmi_view_scroll_y,
            0.0f,
            metrics.maxScrollY
        );

    if (!metrics.showHorizontal)
    {
        hmi_view_scroll_x = 0.0f;
        hmi_view_drag_x = false;
    }

    if (!metrics.showVertical)
    {
        hmi_view_scroll_y = 0.0f;
        hmi_view_drag_y = false;
    }
}


static void hmi_view_update_wheel(
    HmiViewportMetrics metrics
)
{
    Vector2 wheel =
        GetMouseWheelMoveV();

    bool shift =
        IsKeyDown(KEY_LEFT_SHIFT) ||
        IsKeyDown(KEY_RIGHT_SHIFT);

    /*
     * Native horizontal wheel/trackpad motion gets first priority.
     */
    if (
        metrics.showHorizontal &&
        fabsf(wheel.x) > 1e-6f
    )
    {
        hmi_view_scroll_x -=
            wheel.x *
            HMI_VIEW_SCROLL_STEP;
    }

    if (fabsf(wheel.y) <= 1e-6f)
    {
        return;
    }

    if (
        metrics.showHorizontal &&
        (
            shift ||
            !metrics.showVertical
        )
    )
    {
        hmi_view_scroll_x -=
            wheel.y *
            HMI_VIEW_SCROLL_STEP;
    }
    else if (metrics.showVertical)
    {
        hmi_view_scroll_y -=
            wheel.y *
            HMI_VIEW_SCROLL_STEP;
    }
}


static void hmi_view_update_scrollbar_drag(
    HmiViewportMetrics metrics
)
{
    Vector2 mouse =
        GetMousePosition();

    if (
        IsMouseButtonPressed(
            MOUSE_BUTTON_LEFT
        )
    )
    {
        if (metrics.showHorizontal)
        {
            Rectangle track =
                hmi_view_horizontal_track(
                    metrics
                );

            Rectangle thumb =
                hmi_view_horizontal_thumb(
                    metrics,
                    track
                );

            if (
                CheckCollisionPointRec(
                    mouse,
                    thumb
                )
            )
            {
                hmi_view_drag_x = true;

                hmi_view_drag_offset_x =
                    mouse.x -
                    thumb.x;
            }
            else if (
                CheckCollisionPointRec(
                    mouse,
                    track
                )
            )
            {
                float travel =
                    track.width -
                    thumb.width;

                float thumbX =
                    hmi_view_clampf(
                        mouse.x -
                            thumb.width * 0.5f,
                        track.x,
                        track.x + travel
                    );

                if (travel > 0.0f)
                {
                    hmi_view_scroll_x =
                        (
                            (thumbX - track.x) /
                            travel
                        ) *
                        metrics.maxScrollX;
                }

                hmi_view_drag_x = true;
                hmi_view_drag_offset_x =
                    thumb.width * 0.5f;
            }
        }

        if (metrics.showVertical)
        {
            Rectangle track =
                hmi_view_vertical_track(
                    metrics
                );

            Rectangle thumb =
                hmi_view_vertical_thumb(
                    metrics,
                    track
                );

            if (
                CheckCollisionPointRec(
                    mouse,
                    thumb
                )
            )
            {
                hmi_view_drag_y = true;

                hmi_view_drag_offset_y =
                    mouse.y -
                    thumb.y;
            }
            else if (
                CheckCollisionPointRec(
                    mouse,
                    track
                )
            )
            {
                float travel =
                    track.height -
                    thumb.height;

                float thumbY =
                    hmi_view_clampf(
                        mouse.y -
                            thumb.height * 0.5f,
                        track.y,
                        track.y + travel
                    );

                if (travel > 0.0f)
                {
                    hmi_view_scroll_y =
                        (
                            (thumbY - track.y) /
                            travel
                        ) *
                        metrics.maxScrollY;
                }

                hmi_view_drag_y = true;
                hmi_view_drag_offset_y =
                    thumb.height * 0.5f;
            }
        }
    }

    if (hmi_view_drag_x)
    {
        if (
            IsMouseButtonDown(
                MOUSE_BUTTON_LEFT
            )
        )
        {
            Rectangle track =
                hmi_view_horizontal_track(
                    metrics
                );

            Rectangle thumb =
                hmi_view_horizontal_thumb(
                    metrics,
                    track
                );

            float travel =
                track.width -
                thumb.width;

            float thumbX =
                hmi_view_clampf(
                    mouse.x -
                        hmi_view_drag_offset_x,
                    track.x,
                    track.x + travel
                );

            if (travel > 0.0f)
            {
                hmi_view_scroll_x =
                    (
                        (thumbX - track.x) /
                        travel
                    ) *
                    metrics.maxScrollX;
            }
        }
        else
        {
            hmi_view_drag_x = false;
        }
    }

    if (hmi_view_drag_y)
    {
        if (
            IsMouseButtonDown(
                MOUSE_BUTTON_LEFT
            )
        )
        {
            Rectangle track =
                hmi_view_vertical_track(
                    metrics
                );

            Rectangle thumb =
                hmi_view_vertical_thumb(
                    metrics,
                    track
                );

            float travel =
                track.height -
                thumb.height;

            float thumbY =
                hmi_view_clampf(
                    mouse.y -
                        hmi_view_drag_offset_y,
                    track.y,
                    track.y + travel
                );

            if (travel > 0.0f)
            {
                hmi_view_scroll_y =
                    (
                        (thumbY - track.y) /
                        travel
                    ) *
                    metrics.maxScrollY;
            }
        }
        else
        {
            hmi_view_drag_y = false;
        }
    }
}


static void hmi_view_update_input(void)
{
    HmiViewportMetrics metrics =
        hmi_view_metrics();

    hmi_view_clamp_scroll(
        metrics
    );

    hmi_view_update_wheel(
        metrics
    );

    hmi_view_update_scrollbar_drag(
        metrics
    );

    hmi_view_clamp_scroll(
        metrics
    );
}


static Vector2 hmi_view_canvas_origin(void)
{
    HmiViewportMetrics metrics =
        hmi_view_metrics();

    float x =
        metrics.showHorizontal
        ? -hmi_view_scroll_x
        : fmaxf(
            0.0f,
            (
                metrics.viewportWidth -
                HMI_VIEW_CONTENT_WIDTH
            ) *
            0.5f
        );

    float y =
        metrics.showVertical
        ? -hmi_view_scroll_y
        : fmaxf(
            0.0f,
            (
                metrics.viewportHeight -
                HMI_VIEW_CONTENT_HEIGHT
            ) *
            0.5f
        );

    return
        (Vector2)
        {
            x,
            y
        };
}


static void hmi_view_draw_scrollbars(void)
{
    HmiViewportMetrics metrics =
        hmi_view_metrics();

    Vector2 mouse =
        GetMousePosition();

    const Color gutterColor =
        (Color){15, 18, 24, 255};

    const Color trackColor =
        (Color){24, 29, 38, 255};

    const Color thumbColor =
        (Color){73, 86, 106, 255};

    const Color thumbHover =
        (Color){94, 111, 137, 255};

    const Color thumbActive =
        (Color){88, 166, 255, 255};

    if (metrics.showHorizontal)
    {
        DrawRectangle(
            0,
            (int)metrics.viewportHeight,
            (int)metrics.viewportWidth,
            (int)HMI_VIEW_SCROLLBAR_GUTTER,
            gutterColor
        );

        Rectangle track =
            hmi_view_horizontal_track(
                metrics
            );

        Rectangle thumb =
            hmi_view_horizontal_thumb(
                metrics,
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
            hmi_view_drag_x
                ? thumbActive
                : hovered
                    ? thumbHover
                    : thumbColor
        );
    }

    if (metrics.showVertical)
    {
        DrawRectangle(
            (int)metrics.viewportWidth,
            0,
            (int)HMI_VIEW_SCROLLBAR_GUTTER,
            (int)metrics.viewportHeight,
            gutterColor
        );

        Rectangle track =
            hmi_view_vertical_track(
                metrics
            );

        Rectangle thumb =
            hmi_view_vertical_thumb(
                metrics,
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
            hmi_view_drag_y
                ? thumbActive
                : hovered
                    ? thumbHover
                    : thumbColor
        );
    }

    if (
        metrics.showHorizontal &&
        metrics.showVertical
    )
    {
        DrawRectangle(
            (int)metrics.viewportWidth,
            (int)metrics.viewportHeight,
            (int)HMI_VIEW_SCROLLBAR_GUTTER,
            (int)HMI_VIEW_SCROLLBAR_GUTTER,
            gutterColor
        );
    }
}


/* ============================================================================
 * HOOKS USED BY hmi.c
 * ============================================================================
 */

/*
 * hmi.c calculates a fit scale from GetScreenWidth()/GetScreenHeight(). Give it
 * the exact design dimensions so its own calculation always resolves to 1.0.
 * There is intentionally no zoom state anywhere in this file.
 */
static int hmi_view_virtual_width(void)
{
    return
        (int)HMI_VIEW_CONTENT_WIDTH;
}


static int hmi_view_virtual_height(void)
{
    return
        (int)HMI_VIEW_CONTENT_HEIGHT;
}


/*
 * Translate real window coordinates into the fixed 1440 x 900 canvas. Hide
 * scrollbar clicks from controls underneath the scrollbar gutters.
 */
static Vector2 hmi_view_mouse_position(void)
{
    if (
        hmi_view_drag_x ||
        hmi_view_drag_y ||
        hmi_view_pointer_on_scrollbar()
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

    Vector2 origin =
        hmi_view_canvas_origin();

    return
        (Vector2)
        {
            mouse.x - origin.x,
            mouse.y - origin.y
        };
}


static void hmi_view_begin_mode_2d(
    Camera2D camera
)
{
    hmi_view_update_input();

    camera.offset =
        hmi_view_canvas_origin();

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


static void hmi_view_end_drawing(void)
{
    hmi_view_draw_scrollbars();
    EndDrawing();
}


/*
 * Apply hooks only after all helpers above compile, so helper calls still reach
 * raylib's real window/input functions instead of recursively calling wrappers.
 */
#define GetScreenWidth()     hmi_view_virtual_width()
#define GetScreenHeight()    hmi_view_virtual_height()
#define GetMousePosition()   hmi_view_mouse_position()
#define BeginMode2D(camera)  hmi_view_begin_mode_2d((camera))
#define EndDrawing()         hmi_view_end_drawing()


#endif /* HMI_SCROLL_HOOKS_H */
