/* Deterministic renderer/input replacement. Compiles the ACTUAL GUI main;
 * no display required, no replacement sender or controller. */
#define _POSIX_C_SOURCE 200809L
#include "../../HMI/hmi_theme.h"
#include "../../HMI/hmi_protocol.h"
#include <time.h>
#include <stdio.h>
static unsigned frame;
static double epoch;
static double mono(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return t.tv_sec+t.tv_nsec/1e9; }
static void pause_ms(long ms) { struct timespec t={ms/1000,(ms%1000)*1000000L}; nanosleep(&t,0); }
void SetConfigFlags(unsigned int f) {(void)f;}
void InitWindow(int w,int h,const char *s) {(void)w;(void)h;(void)s;epoch=mono();}
bool IsWindowReady(void) {return true;}
bool WindowShouldClose(void) {return frame>=130;}
bool IsWindowFocused(void) {return frame<80 || frame>=105;}
bool IsWindowMinimized(void) {return false;}
void SetTargetFPS(int n) {(void)n;}
double GetTime(void) {return mono()-epoch;}
Vector2 GetMousePosition(void) {return (Vector2){0};}
bool IsMouseButtonDown(int b) {(void)b;return false;}
bool IsMouseButtonPressed(int b) {(void)b;return false;}
bool IsKeyDown(int k) {return k==KEY_RIGHT && frame>=20 && frame<105;}
bool CheckCollisionPointRec(Vector2 p,Rectangle r) {(void)p;(void)r;return false;}
void BeginDrawing(void) {}
void EndDrawing(void) {
    if(frame==40) { puts("RENDER_STALL_BEGIN");fflush(stdout);pause_ms(400);puts("RENDER_STALL_END");fflush(stdout); }
    pause_ms(8);++frame;
}
void CloseWindow(void) {}
void ClearBackground(Color c) {(void)c;}
void DrawRectangleRounded(Rectangle r,float x,int s,Color c) {(void)r;(void)x;(void)s;(void)c;}
void DrawRectangleLinesEx(Rectangle r,float x,Color c) {(void)r;(void)x;(void)c;}
void DrawCircleV(Vector2 p,float r,Color c) {(void)p;(void)r;(void)c;}
void DrawCircle(int x,int y,float r,Color c) {(void)x;(void)y;(void)r;(void)c;}
#define COLOR(n) const Color n={0}
COLOR(HMI_C_BG);COLOR(HMI_C_PANEL);COLOR(HMI_C_PANEL_2);COLOR(HMI_C_FIELD);
COLOR(HMI_C_FIELD_HI);COLOR(HMI_C_BORDER);COLOR(HMI_C_TEXT);COLOR(HMI_C_MUTED);
COLOR(HMI_C_FAINT);COLOR(HMI_C_ACCENT);COLOR(HMI_C_GOOD);COLOR(HMI_C_WARN);COLOR(HMI_C_BAD);
void hmi_ui_init(void) {} void hmi_ui_shutdown(void) {}
void hmi_ui_text(const char *s,float x,float y,float z,Color c,bool b) {(void)s;(void)x;(void)y;(void)z;(void)c;(void)b;}
float hmi_ui_text_width(const char *s,float z,bool b) {(void)s;(void)z;(void)b;return 1;}
void hmi_ui_panel(Rectangle r,Color c) {(void)r;(void)c;}
bool hmi_protocol_init(HmiProtocol *p,uint16_t port) {(void)port;p->status.robot_state=3;p->status.can_ready_nodes=6;p->status.can_expected_nodes=6;return true;}
void hmi_protocol_shutdown(HmiProtocol *p) {(void)p;}
void hmi_protocol_poll_status(HmiProtocol *p) {(void)p;}
bool hmi_protocol_controller_online(const HmiProtocol *p,double t) {(void)p;(void)t;return true;}
