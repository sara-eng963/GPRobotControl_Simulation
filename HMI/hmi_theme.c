#include "hmi_theme.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const Color HMI_C_BG        = { 15,  18,  24, 255};
const Color HMI_C_PANEL     = { 22,  27,  35, 255};
const Color HMI_C_PANEL_2   = { 27,  33,  43, 255};
const Color HMI_C_FIELD     = { 31,  38,  49, 255};
const Color HMI_C_FIELD_HI  = { 38,  49,  64, 255};
const Color HMI_C_BORDER    = { 52,  63,  80, 255};
const Color HMI_C_BORDER_HI = { 78, 109, 153, 255};
const Color HMI_C_TEXT      = {235, 239, 246, 255};
const Color HMI_C_MUTED     = {158, 170, 189, 255};
const Color HMI_C_FAINT     = {112, 125, 146, 255};
const Color HMI_C_ACCENT    = { 88, 166, 255, 255};
const Color HMI_C_ACCENT_2  = { 65, 128, 204, 255};
const Color HMI_C_GOOD      = { 91, 201, 142, 255};
const Color HMI_C_WARN      = {239, 184,  86, 255};
const Color HMI_C_BAD       = {232, 105, 111, 255};
const Color HMI_C_DISABLED  = { 78,  88, 105, 255};

static Font uiFont;
static Font uiFontBold;
static bool uiFontOwned = false;
static bool uiFontBoldOwned = false;

static Font load_first_font(
    const char *const *candidates,
    size_t count,
    int size,
    bool *owned
)
{
    if (owned != NULL)
    {
        *owned = false;
    }

    for (size_t i = 0; i < count; i++)
    {
        if (FileExists(candidates[i]))
        {
            Font font = LoadFontEx(candidates[i], size, NULL, 0);
            SetTextureFilter(font.texture, TEXTURE_FILTER_BILINEAR);

            if (owned != NULL)
            {
                *owned = true;
            }

            return font;
        }
    }

    return GetFontDefault();
}

void hmi_ui_init(void)
{
    const char *regularCandidates[] =
    {
        "/mnt/c/Windows/Fonts/segoeui.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf"
    };

    const char *boldCandidates[] =
    {
        "/mnt/c/Windows/Fonts/seguisb.ttf",
        "/mnt/c/Windows/Fonts/segoeuib.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf"
    };

    uiFont =
        load_first_font(
            regularCandidates,
            sizeof(regularCandidates) / sizeof(regularCandidates[0]),
            40,
            &uiFontOwned
        );

    uiFontBold =
        load_first_font(
            boldCandidates,
            sizeof(boldCandidates) / sizeof(boldCandidates[0]),
            42,
            &uiFontBoldOwned
        );
}

void hmi_ui_shutdown(void)
{
    if (uiFontBoldOwned)
    {
        UnloadFont(uiFontBold);
    }

    if (uiFontOwned)
    {
        UnloadFont(uiFont);
    }

    uiFontOwned = false;
    uiFontBoldOwned = false;
}

void hmi_ui_text(
    const char *text,
    float x,
    float y,
    float size,
    Color color,
    bool bold
)
{
    Font font = bold ? uiFontBold : uiFont;

    DrawTextEx(
        font,
        text,
        (Vector2){x, y},
        size,
        size * 0.015f,
        color
    );
}

float hmi_ui_text_width(
    const char *text,
    float size,
    bool bold
)
{
    Font font = bold ? uiFontBold : uiFont;

    return MeasureTextEx(
        font,
        text,
        size,
        size * 0.015f
    ).x;
}

void hmi_ui_panel(
    Rectangle bounds,
    Color fill
)
{
    DrawRectangleRounded(bounds, 0.045f, 12, fill);
    DrawRectangleLinesEx(bounds, 1.0f, HMI_C_BORDER);
}

void hmi_ui_status_dot(
    float x,
    float y,
    bool active,
    Color activeColor
)
{
    DrawCircleV(
        (Vector2){x, y},
        5.0f,
        active ? activeColor : HMI_C_DISABLED
    );
}

bool hmi_ui_button(
    Rectangle bounds,
    const char *label,
    HmiButtonStyle style,
    bool enabled,
    Vector2 mouse,
    bool pointerBlocked
)
{
    bool hovered =
        enabled &&
        !pointerBlocked &&
        CheckCollisionPointRec(mouse, bounds);

    Color fill;
    Color border;
    Color foreground;

    if (!enabled)
    {
        fill = (Color){31, 36, 45, 255};
        border = HMI_C_BORDER;
        foreground = HMI_C_DISABLED;
    }
    else if (style == HMI_BUTTON_DANGER)
    {
        fill = hovered
            ? (Color){129, 51, 58, 255}
            : (Color){96, 42, 49, 255};
        border = HMI_C_BAD;
        foreground = HMI_C_TEXT;
    }
    else if (style == HMI_BUTTON_PRIMARY)
    {
        fill = hovered
            ? (Color){77, 151, 236, 255}
            : HMI_C_ACCENT_2;
        border = HMI_C_ACCENT;
        foreground = HMI_C_TEXT;
    }
    else
    {
        fill = hovered
            ? (Color){42, 51, 65, 255}
            : HMI_C_PANEL_2;
        border = hovered ? HMI_C_BORDER_HI : HMI_C_BORDER;
        foreground = HMI_C_TEXT;
    }

    DrawRectangleRounded(bounds, 0.14f, 10, fill);
    DrawRectangleLinesEx(bounds, 1.0f, border);

    float fontSize = 14.0f;
    float width = hmi_ui_text_width(label, fontSize, true);

    hmi_ui_text(
        label,
        bounds.x + bounds.width * 0.5f - width * 0.5f,
        bounds.y + bounds.height * 0.5f - fontSize * 0.58f,
        fontSize,
        foreground,
        true
    );

    return
        hovered &&
        IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

void hmi_numeric_field_set(
    HmiNumericField *field,
    double value,
    int decimals
)
{
    if (field == NULL)
    {
        return;
    }

    snprintf(
        field->text,
        sizeof(field->text),
        "%.*f",
        decimals,
        value
    );

    field->active = false;
    field->replaceOnType = false;
}

bool hmi_numeric_field_parse(
    const HmiNumericField *field,
    double *value
)
{
    if (field == NULL || value == NULL)
    {
        return false;
    }

    char *end = NULL;
    double parsed = strtod(field->text, &end);

    if (
        end == field->text ||
        *end != '\0' ||
        !isfinite(parsed)
    )
    {
        return false;
    }

    *value = parsed;
    return true;
}

static void update_numeric_field(
    HmiNumericField *field,
    Rectangle bounds,
    bool enabled,
    Vector2 mouse,
    bool pointerBlocked
)
{
    if (field == NULL)
    {
        return;
    }

    if (
        IsMouseButtonPressed(MOUSE_BUTTON_LEFT) &&
        !pointerBlocked
    )
    {
        bool clicked =
            enabled &&
            CheckCollisionPointRec(mouse, bounds);

        if (clicked)
        {
            field->active = true;
            field->replaceOnType = true;
        }
        else
        {
            field->active = false;
            field->replaceOnType = false;
        }
    }

    if (!enabled || !field->active)
    {
        return;
    }

    if (
        (IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)) &&
        IsKeyPressed(KEY_A)
    )
    {
        field->replaceOnType = true;
    }

    int key = GetCharPressed();

    while (key > 0)
    {
        bool allowed =
            (key >= '0' && key <= '9') ||
            key == '-' ||
            key == '+' ||
            key == '.' ||
            key == 'e' ||
            key == 'E';

        if (allowed)
        {
            if (field->replaceOnType)
            {
                field->text[0] = '\0';
                field->replaceOnType = false;
            }

            size_t length = strlen(field->text);

            if (length + 1 < sizeof(field->text))
            {
                field->text[length] = (char)key;
                field->text[length + 1] = '\0';
            }
        }

        key = GetCharPressed();
    }

    if (IsKeyPressed(KEY_BACKSPACE) || IsKeyPressed(KEY_DELETE))
    {
        if (field->replaceOnType)
        {
            field->text[0] = '\0';
            field->replaceOnType = false;
        }
        else
        {
            size_t length = strlen(field->text);

            if (length > 0)
            {
                field->text[length - 1] = '\0';
            }
        }
    }

    if (
        IsKeyPressed(KEY_ENTER) ||
        IsKeyPressed(KEY_KP_ENTER) ||
        IsKeyPressed(KEY_ESCAPE)
    )
    {
        field->active = false;
        field->replaceOnType = false;
    }
}

static void draw_numeric_field(
    HmiNumericField *field,
    Rectangle bounds,
    bool enabled
)
{
    Color fill =
        !enabled
            ? (Color){24, 29, 37, 255}
            : field->active
                ? HMI_C_FIELD_HI
                : HMI_C_FIELD;

    Color border =
        !enabled
            ? (Color){42, 49, 61, 255}
            : field->active
                ? HMI_C_ACCENT
                : HMI_C_BORDER;

    Color foreground = enabled ? HMI_C_TEXT : HMI_C_FAINT;

    DrawRectangleRounded(bounds, 0.13f, 8, fill);
    DrawRectangleLinesEx(
        bounds,
        field->active && enabled ? 1.7f : 1.0f,
        border
    );

    if (enabled && field->active && field->replaceOnType)
    {
        float selectionWidth =
            hmi_ui_text_width(field->text, 16.0f, false);

        DrawRectangleRounded(
            (Rectangle)
            {
                bounds.x + 8.0f,
                bounds.y + 7.0f,
                selectionWidth + 7.0f,
                bounds.height - 14.0f
            },
            0.15f,
            6,
            (Color){51, 88, 132, 255}
        );
    }

    hmi_ui_text(
        field->text,
        bounds.x + 10.0f,
        bounds.y + 8.0f,
        16.0f,
        foreground,
        false
    );

    if (
        enabled &&
        field->active &&
        !field->replaceOnType &&
        ((int)(GetTime() * 2.0) % 2 == 0)
    )
    {
        float caretX =
            bounds.x +
            10.0f +
            hmi_ui_text_width(field->text, 16.0f, false) +
            2.0f;

        DrawLineEx(
            (Vector2){caretX, bounds.y + 8.0f},
            (Vector2){caretX, bounds.y + bounds.height - 8.0f},
            1.5f,
            HMI_C_TEXT
        );
    }
}

void hmi_numeric_field_update_draw(
    HmiNumericField *field,
    Rectangle bounds,
    bool enabled,
    Vector2 mouse,
    bool pointerBlocked
)
{
    update_numeric_field(
        field,
        bounds,
        enabled,
        mouse,
        pointerBlocked
    );

    draw_numeric_field(
        field,
        bounds,
        enabled
    );
}
