#ifndef HMI_THEME_H
#define HMI_THEME_H

#include <stdbool.h>

#include "raylib.h"

typedef enum
{
    HMI_BUTTON_NORMAL = 0,
    HMI_BUTTON_PRIMARY,
    HMI_BUTTON_DANGER
} HmiButtonStyle;

typedef struct
{
    char text[32];
    bool active;
    bool replaceOnType;
} HmiNumericField;

extern const Color HMI_C_BG;
extern const Color HMI_C_PANEL;
extern const Color HMI_C_PANEL_2;
extern const Color HMI_C_FIELD;
extern const Color HMI_C_FIELD_HI;
extern const Color HMI_C_BORDER;
extern const Color HMI_C_BORDER_HI;
extern const Color HMI_C_TEXT;
extern const Color HMI_C_MUTED;
extern const Color HMI_C_FAINT;
extern const Color HMI_C_ACCENT;
extern const Color HMI_C_ACCENT_2;
extern const Color HMI_C_GOOD;
extern const Color HMI_C_WARN;
extern const Color HMI_C_BAD;
extern const Color HMI_C_DISABLED;

void hmi_ui_init(void);
void hmi_ui_shutdown(void);

void hmi_ui_text(
    const char *text,
    float x,
    float y,
    float size,
    Color color,
    bool bold
);

float hmi_ui_text_width(
    const char *text,
    float size,
    bool bold
);

void hmi_ui_panel(
    Rectangle bounds,
    Color fill
);

void hmi_ui_status_dot(
    float x,
    float y,
    bool active,
    Color activeColor
);

bool hmi_ui_button(
    Rectangle bounds,
    const char *label,
    HmiButtonStyle style,
    bool enabled,
    Vector2 mouse,
    bool pointerBlocked
);

void hmi_numeric_field_set(
    HmiNumericField *field,
    double value,
    int decimals
);

bool hmi_numeric_field_parse(
    const HmiNumericField *field,
    double *value
);

void hmi_numeric_field_update_draw(
    HmiNumericField *field,
    Rectangle bounds,
    bool enabled,
    Vector2 mouse,
    bool pointerBlocked
);

#endif /* HMI_THEME_H */
