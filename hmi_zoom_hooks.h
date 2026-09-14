#ifndef HMI_ZOOM_HOOKS_H
#define HMI_ZOOM_HOOKS_H

/*
 * ============================================================================
 * HMI ZOOM / PAN LAYER
 * ============================================================================
 *
 * The main HMI is authored on a fixed 1440 x 900 logical canvas and normally
 * scales itself so the entire canvas fits inside the current desktop window.
 * On a narrow window that makes the controls/text too small.
 *
 * This lightweight input/view layer adds operator-controlled zoom without
 * changing the HMI's logical layout or UDP/controller behavior.
 *
 * Controls:
 *
 *      Mouse wheel        zoom in / out
 *      + / numpad +       zoom in
 *      - / numpad -       zoom out
 *      0                  reset to Fit (100%)
 *      Middle-mouse drag  pan while zoomed in
 *
 * 100% means "fit the complete 1440 x 900 HMI into the current window".
 * Zoom is intentionally limited so the UI cannot disappear completely.
 *
 * This header is force-included only for the mock_hmi target by CMake.
 * ============================================================================
 */

#include "raylib.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#define HMI_ZOOM_LOGICAL_WIDTH   1440.0f
#define HMI_ZOOM_LOGICAL_HEIGHT   900.0f

#define HMI_ZOOM_MIN 0.70f
#define HMI_ZOOM_MAX 2.50f
#define HMI_ZOOM_STEP 1.10f


static float hmi_zoom_factor = 1.0f;
static Vector2 hmi_zoom_pan = {0.0f, 0.0f};
static Vector2 hmi_zoom_last_mouse = {0.0f, 0.0f};
static bool hmi_zoom_panning = false;
static bool hmi_zoom_title_initialized = false;


static float hmi_zoom_clampf(
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


static float hmi_zoom_fit_scale(void)
{
    float scale_x =
        (float)GetScreenWidth() /
        HMI_ZOOM_LOGICAL_WIDTH;

    float scale_y =
        (float)GetScreenHeight() /
        HMI_ZOOM_LOGICAL_HEIGHT;

    float scale =
        fminf(
            scale_x,
            scale_y
        );

    return
        scale > 1e-6f
        ? scale
        : 1.0f;
}


static float hmi_zoom_effective_scale(void)
{
    return
        hmi_zoom_fit_scale() *
        hmi_zoom_factor;
}


static void hmi_zoom_clamp_pan(void)
{
    float actual_width =
        (float)GetScreenWidth();

    float actual_height =
        (float)GetScreenHeight();

    float scale =
        hmi_zoom_effective_scale();

    float content_width =
        HMI_ZOOM_LOGICAL_WIDTH *
        scale;

    float content_height =
        HMI_ZOOM_LOGICAL_HEIGHT *
        scale;

    float max_pan_x =
        fmaxf(
            0.0f,
            (content_width - actual_width) * 0.5f
        );

    float max_pan_y =
        fmaxf(
            0.0f,
            (content_height - actual_height) * 0.5f
        );

    hmi_zoom_pan.x =
        hmi_zoom_clampf(
            hmi_zoom_pan.x,
            -max_pan_x,
            max_pan_x
        );

    hmi_zoom_pan.y =
        hmi_zoom_clampf(
            hmi_zoom_pan.y,
            -max_pan_y,
            max_pan_y
        );
}


static void hmi_zoom_update_title(void)
{
    char title[160];

    snprintf(
        title,
        sizeof(title),
        "Robot Motion Console | Zoom %d%% | wheel: zoom | middle-drag: pan | 0: fit",
        (int)lroundf(
            hmi_zoom_factor *
            100.0f
        )
    );

    SetWindowTitle(title);
}


static void hmi_zoom_update_input(void)
{
    bool zoom_changed =
        false;

    float wheel =
        GetMouseWheelMove();

    if (wheel > 0.0f)
    {
        hmi_zoom_factor *=
            powf(
                HMI_ZOOM_STEP,
                wheel
            );

        zoom_changed =
            true;
    }
    else if (wheel < 0.0f)
    {
        hmi_zoom_factor /=
            powf(
                HMI_ZOOM_STEP,
                -wheel
            );

        zoom_changed =
            true;
    }

    if (
        IsKeyPressed(KEY_EQUAL) ||
        IsKeyPressed(KEY_KP_ADD)
    )
    {
        hmi_zoom_factor *=
            HMI_ZOOM_STEP;

        zoom_changed =
            true;
    }

    if (
        IsKeyPressed(KEY_MINUS) ||
        IsKeyPressed(KEY_KP_SUBTRACT)
    )
    {
        hmi_zoom_factor /=
            HMI_ZOOM_STEP;

        zoom_changed =
            true;
    }

    if (
        IsKeyPressed(KEY_ZERO) ||
        IsKeyPressed(KEY_KP_0)
    )
    {
        hmi_zoom_factor =
            1.0f;

        hmi_zoom_pan =
            (Vector2)
            {
                0.0f,
                0.0f
            };

        zoom_changed =
            true;
    }

    hmi_zoom_factor =
        hmi_zoom_clampf(
            hmi_zoom_factor,
            HMI_ZOOM_MIN,
            HMI_ZOOM_MAX
        );


    /* ------------------------------------------------------------------------
     * Middle-mouse panning is enabled only as a view operation. It never
     * changes any waypoint value or sends anything to the controller.
     * ------------------------------------------------------------------------
     */

    Vector2 mouse =
        GetMousePosition();

    if (
        IsMouseButtonPressed(
            MOUSE_BUTTON_MIDDLE
        )
    )
    {
        hmi_zoom_panning =
            true;

        hmi_zoom_last_mouse =
            mouse;
    }

    if (hmi_zoom_panning)
    {
        if (
            IsMouseButtonDown(
                MOUSE_BUTTON_MIDDLE
            )
        )
        {
            Vector2 delta =
            {
                mouse.x - hmi_zoom_last_mouse.x,
                mouse.y - hmi_zoom_last_mouse.y
            };

            hmi_zoom_pan.x +=
                delta.x;

            hmi_zoom_pan.y +=
                delta.y;

            hmi_zoom_last_mouse =
                mouse;
        }
        else
        {
            hmi_zoom_panning =
                false;
        }
    }


    hmi_zoom_clamp_pan();

    if (
        zoom_changed ||
        !hmi_zoom_title_initialized
    )
    {
        hmi_zoom_update_title();

        hmi_zoom_title_initialized =
            true;
    }
}


/*
 * hmi.c obtains GetScreenWidth()/GetScreenHeight() and uses them to calculate
 * its fit scale. Returning a virtual size multiplied by hmi_zoom_factor causes
 * that existing calculation to produce:
 *
 *      effective scale = fit scale * user zoom
 *
 * without modifying the HMI's logical layout.
 */
static int hmi_zoom_screen_width(void)
{
    hmi_zoom_update_input();

    int width =
        (int)lroundf(
            (float)GetScreenWidth() *
            hmi_zoom_factor
        );

    return
        width > 0
        ? width
        : 1;
}


static int hmi_zoom_screen_height(void)
{
    int height =
        (int)lroundf(
            (float)GetScreenHeight() *
            hmi_zoom_factor
        );

    return
        height > 0
        ? height
        : 1;
}


static Vector2 hmi_zoom_desired_origin(void)
{
    float scale =
        hmi_zoom_effective_scale();

    return
        (Vector2)
        {
            (
                (float)GetScreenWidth() -
                HMI_ZOOM_LOGICAL_WIDTH * scale
            ) * 0.5f +
            hmi_zoom_pan.x,

            (
                (float)GetScreenHeight() -
                HMI_ZOOM_LOGICAL_HEIGHT * scale
            ) * 0.5f +
            hmi_zoom_pan.y
        };
}


/*
 * The HMI's internal ui_origin is based on the virtual dimensions above.
 * This is the exact origin it will compute, reconstructed here so mouse hit
 * testing can be translated to the camera origin actually shown on screen.
 */
static Vector2 hmi_zoom_internal_origin(void)
{
    float scale =
        hmi_zoom_effective_scale();

    float virtual_width =
        (float)GetScreenWidth() *
        hmi_zoom_factor;

    float virtual_height =
        (float)GetScreenHeight() *
        hmi_zoom_factor;

    return
        (Vector2)
        {
            (
                virtual_width -
                HMI_ZOOM_LOGICAL_WIDTH * scale
            ) * 0.5f,

            (
                virtual_height -
                HMI_ZOOM_LOGICAL_HEIGHT * scale
            ) * 0.5f
        };
}


static Vector2 hmi_zoom_mouse_position(void)
{
    Vector2 actual_mouse =
        GetMousePosition();

    Vector2 internal_origin =
        hmi_zoom_internal_origin();

    Vector2 desired_origin =
        hmi_zoom_desired_origin();

    /*
     * hmi.c will subsequently subtract its internal ui_origin and divide by
     * ui_scale. Offset the mouse here so the result corresponds to the camera
     * origin that is really being rendered.
     */
    return
        (Vector2)
        {
            actual_mouse.x +
                internal_origin.x -
                desired_origin.x,

            actual_mouse.y +
                internal_origin.y -
                desired_origin.y
        };
}


static void hmi_zoom_begin_mode_2d(
    Camera2D camera
)
{
    camera.offset =
        hmi_zoom_desired_origin();

    BeginMode2D(camera);
}


/*
 * Apply the hooks only after all helper functions above have been compiled, so
 * their calls still reach raylib's original functions rather than recursing.
 */
#define GetScreenWidth()     hmi_zoom_screen_width()
#define GetScreenHeight()    hmi_zoom_screen_height()
#define GetMousePosition()   hmi_zoom_mouse_position()
#define BeginMode2D(camera)  hmi_zoom_begin_mode_2d((camera))


#endif /* HMI_ZOOM_HOOKS_H */
