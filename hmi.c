/*
 * ============================================================================
 *  CARTESIAN LINE TRAJECTORY HMI
 * ============================================================================
 *
 * PURPOSE
 * -------
 * Desktop-side raylib HMI for requesting a SINGLE Cartesian line trajectory:
 *
 *      Waypoint A  --->  Waypoint B
 *
 * The HMI sends:
 *
 *      A position      : X Y Z [m]
 *      A orientation   : yaw pitch roll [deg], ZYX convention
 *      B position      : X Y Z [m]
 *      B orientation   : yaw pitch roll [deg], ZYX convention
 *      TCP speed       : [m/s]
 *      TCP acceleration: [m/s^2]
 *      TCP jerk        : [m/s^3]
 *
 * The controller is responsible for:
 *
 *      line geometry
 *          ->
 *      arc-length parameterization
 *          ->
 *      S-curve time scaling
 *          ->
 *      SLERP orientation
 *          ->
 *      sequential ADLS IK
 *          ->
 *      qPath at 1 ms
 *          ->
 *      EtherCAT CSP
 *
 * IMPORTANT
 * ---------
 * This HMI uses a NEW packet format ("RBT2").
 * main.c must be updated to decode this exact packet before this HMI can
 * control the trajectory.
 *
 * ============================================================================
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "raylib.h"


/* ============================================================================
 * CONFIGURATION
 * ============================================================================
 */

#define CONTROLLER_IP   "127.0.0.1"
#define CONTROLLER_PORT 5006
#define STATUS_PORT     5007

#define NUM_POSE_VALUES 6
#define PLAN_FLOAT_COUNT 15

/* ============================================================================
 * RESIZABLE / HORIZONTAL SCROLLING
 * ============================================================================
 *
 * The HMI layout itself is still designed on a 1400 px wide canvas.
 *
 * If the OS window becomes narrower than that, we simply view a smaller
 * portion of the canvas and allow horizontal scrolling.
 *
 * Nothing about the actual GUI layout changes.
 * ============================================================================
 */

#define HMI_CONTENT_WIDTH   1400.0f
#define HMI_SCROLL_STEP       80.0f


static float hmi_scroll_x =
    0.0f;


static bool hmi_scroll_dragging =
    false;


static float hmi_scroll_drag_offset =
    0.0f;


/* --------------------------------------------------------------------------
 * Small local clamp helper
 * --------------------------------------------------------------------------
 */

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


/* --------------------------------------------------------------------------
 * Mouse position in HMI CANVAS coordinates
 * --------------------------------------------------------------------------
 *
 * Raylib gives mouse position relative to the visible window.
 *
 * But our controls are positioned in the 1400 px virtual HMI canvas.
 *
 * Therefore:
 *
 *      canvas mouse X = screen mouse X + horizontal scroll
 */

static Vector2 hmi_mouse_position(void)
{
    Vector2 mouse =
        GetMousePosition();


    mouse.x +=
        hmi_scroll_x;


    return mouse;
}


/* --------------------------------------------------------------------------
 * Horizontal scrollbar input
 * --------------------------------------------------------------------------
 */

static void update_horizontal_scroll(void)
{
    float viewport_width =
        (float)GetScreenWidth();


    float max_scroll =
        HMI_CONTENT_WIDTH -
        viewport_width;


    /*
     * Full HMI already fits.
     */
    if (max_scroll <= 0.0f)
    {
        hmi_scroll_x =
            0.0f;

        hmi_scroll_dragging =
            false;

        return;
    }


    /*
     * Normal mouse wheel scrolls horizontally whenever the HMI is narrower
     * than its virtual canvas.
     *
     * Wheel down  -> right
     * Wheel up    -> left
     */
    float wheel =
        GetMouseWheelMove();


    if (wheel != 0.0f)
    {
        hmi_scroll_x -=
            wheel *
            HMI_SCROLL_STEP;
    }


    hmi_scroll_x =
        hmi_clampf(
            hmi_scroll_x,
            0.0f,
            max_scroll
        );


    /* ----------------------------------------------------------------------
     * Scrollbar geometry is SCREEN-SPACE, not canvas-space.
     * ----------------------------------------------------------------------
     */

    float track_x =
        10.0f;


    float track_width =
        viewport_width -
        20.0f;


    if (track_width < 80.0f)
    {
        track_width =
            80.0f;
    }


    float thumb_width =
        track_width *
        (
            viewport_width /
            HMI_CONTENT_WIDTH
        );


    if (thumb_width < 60.0f)
    {
        thumb_width =
            60.0f;
    }


    if (thumb_width > track_width)
    {
        thumb_width =
            track_width;
    }


    float usable_width =
        track_width -
        thumb_width;


    float thumb_x =
        track_x;


    if (
        max_scroll > 0.0f &&
        usable_width > 0.0f
    )
    {
        thumb_x +=
            (
                hmi_scroll_x /
                max_scroll
            ) *
            usable_width;
    }


    Rectangle track =
    {
        track_x,
        (float)GetScreenHeight() - 16.0f,
        track_width,
        9.0f
    };


    Rectangle thumb =
    {
        thumb_x,
        track.y,
        thumb_width,
        track.height
    };


    Vector2 mouse =
        GetMousePosition();


    /* Start dragging the thumb. */
    if (
        IsMouseButtonPressed(
            MOUSE_BUTTON_LEFT
        )
    )
    {
        if (
            CheckCollisionPointRec(
                mouse,
                thumb
            )
        )
        {
            hmi_scroll_dragging =
                true;


            hmi_scroll_drag_offset =
                mouse.x -
                thumb.x;
        }

        /*
         * Clicking somewhere else on the scrollbar jumps there and begins
         * dragging immediately.
         */
        else if (
            CheckCollisionPointRec(
                mouse,
                track
            ) &&
            usable_width > 0.0f
        )
        {
            float new_thumb_x =
                mouse.x -
                thumb_width / 2.0f;


            new_thumb_x =
                hmi_clampf(
                    new_thumb_x,
                    track_x,
                    track_x +
                    usable_width
                );


            hmi_scroll_x =
                (
                    (
                        new_thumb_x -
                        track_x
                    ) /
                    usable_width
                ) *
                max_scroll;


            hmi_scroll_dragging =
                true;


            hmi_scroll_drag_offset =
                thumb_width /
                2.0f;
        }
    }


    /* Continue dragging. */
    if (hmi_scroll_dragging)
    {
        if (
            IsMouseButtonDown(
                MOUSE_BUTTON_LEFT
            )
        )
        {
            float new_thumb_x =
                mouse.x -
                hmi_scroll_drag_offset;


            new_thumb_x =
                hmi_clampf(
                    new_thumb_x,
                    track_x,
                    track_x +
                    usable_width
                );


            if (usable_width > 0.0f)
            {
                hmi_scroll_x =
                    (
                        (
                            new_thumb_x -
                            track_x
                        ) /
                        usable_width
                    ) *
                    max_scroll;
            }
        }
        else
        {
            hmi_scroll_dragging =
                false;
        }
    }


    hmi_scroll_x =
        hmi_clampf(
            hmi_scroll_x,
            0.0f,
            max_scroll
        );
}


/* --------------------------------------------------------------------------
 * Draw scrollbar on top of the HMI
 * --------------------------------------------------------------------------
 */

static void draw_horizontal_scrollbar(void)
{
    float viewport_width =
        (float)GetScreenWidth();


    float max_scroll =
        HMI_CONTENT_WIDTH -
        viewport_width;


    if (max_scroll <= 0.0f)
    {
        return;
    }


    float track_x =
        10.0f;


    float track_width =
        viewport_width -
        20.0f;


    if (track_width < 80.0f)
    {
        track_width =
            80.0f;
    }


    float thumb_width =
        track_width *
        (
            viewport_width /
            HMI_CONTENT_WIDTH
        );


    if (thumb_width < 60.0f)
    {
        thumb_width =
            60.0f;
    }


    if (thumb_width > track_width)
    {
        thumb_width =
            track_width;
    }


    float usable_width =
        track_width -
        thumb_width;


    float thumb_x =
        track_x;


    if (usable_width > 0.0f)
    {
        thumb_x +=
            (
                hmi_scroll_x /
                max_scroll
            ) *
            usable_width;
    }


    Rectangle track =
    {
        track_x,
        (float)GetScreenHeight() - 16.0f,
        track_width,
        9.0f
    };


    Rectangle thumb =
    {
        thumb_x,
        track.y,
        thumb_width,
        track.height
    };


    DrawRectangleRounded(
        track,
        1.0f,
        8,
        (Color){38, 43, 55, 255}
    );


    DrawRectangleRounded(
        thumb,
        1.0f,
        8,
        hmi_scroll_dragging
            ? (Color){120, 165, 230, 255}
            : (Color){83, 91, 111, 255}
    );
}

/*
 * Controller -> HMI live-status packet.
 *
 * 13 uint32 words = 52 bytes:
 *
 *   0      magic "STA2"
 *   1      MotionState
 *   2      last HMI sequence
 *   3      trajectory index
 *   4      trajectory sample count
 *   5      actual EtherCAT WKC
 *   6      expected EtherCAT WKC
 *   7..12  raw CiA-402 Statusword for J1..J6
 */
#define STATUS_PACKET_MAGIC 0x53544132U   /* ASCII "STA2" */
#define STATUS_WORD_COUNT   13U

/*
 * Packet:
 *
 *      uint32 magic
 *      uint32 command
 *      uint32 sequence
 *      float  A.x
 *      float  A.y
 *      float  A.z
 *      float  A.yaw
 *      float  A.pitch
 *      float  A.roll
 *      float  B.x
 *      float  B.y
 *      float  B.z
 *      float  B.yaw
 *      float  B.pitch
 *      float  B.roll
 *      float  tcpSpeed
 *      float  tcpAccel
 *      float  tcpJerk
 *
 * Every 32-bit word is sent in network byte order.
 *
 * Total = 18 words = 72 bytes.
 */

#define PACKET_MAGIC 0x52425432U   /* ASCII "RBT2" */

typedef enum
{
    COMMAND_PLAN_AND_RUN_LINE = 1,
    COMMAND_STOP              = 2
} CommandType;


/* ============================================================================
 * SMALL GUI HELPERS
 * ============================================================================
 */

typedef struct
{
    char text[32];
    bool active;

    /*
     * Clicking a field logically selects the old value.
     * The next typed character replaces it.
     */
    bool replace_on_type;
} NumericField;


static void numeric_field_set(
    NumericField *field,
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
    field->replace_on_type = false;
}


static double numeric_field_value(
    const NumericField *field
)
{
    if (field == NULL)
    {
        return 0.0;
    }

    char *end = NULL;

    double value =
        strtod(
            field->text,
            &end
        );

    if (end == field->text)
    {
        return 0.0;
    }

    return value;
}


static void update_numeric_field(
    NumericField *field,
    Rectangle bounds
)
{
    Vector2 mouse =
         hmi_mouse_position();


    /*
     * Click a box to activate it.
     *
     * The old value becomes logically selected, so typing immediately replaces
     * the old number instead of appending to it.
     */
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
        bool clicked =
            CheckCollisionPointRec(
                mouse,
                bounds
            );


        if (clicked)
        {
            field->active =
                true;

            field->replace_on_type =
                true;
        }
        else
        {
            field->active =
                false;

            field->replace_on_type =
                false;
        }
    }


    if (!field->active)
    {
        return;
    }


    /*
     * Ctrl+A = select the full current value.
     */
    if (
        (
            IsKeyDown(KEY_LEFT_CONTROL) ||
            IsKeyDown(KEY_RIGHT_CONTROL)
        )
        &&
        IsKeyPressed(KEY_A)
    )
    {
        field->replace_on_type =
            true;
    }


    int key =
        GetCharPressed();


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
            if (field->replace_on_type)
            {
                field->text[0] =
                    '\0';

                field->replace_on_type =
                    false;
            }


            size_t length =
                strlen(field->text);


            if (
                length + 1 <
                sizeof(field->text)
            )
            {
                field->text[length] =
                    (char)key;

                field->text[length + 1] =
                    '\0';
            }
        }


        key =
            GetCharPressed();
    }


    if (
        IsKeyPressed(KEY_BACKSPACE) ||
        IsKeyPressed(KEY_DELETE)
    )
    {
        if (field->replace_on_type)
        {
            field->text[0] =
                '\0';

            field->replace_on_type =
                false;
        }
        else
        {
            size_t length =
                strlen(field->text);


            if (length > 0)
            {
                field->text[length - 1] =
                    '\0';
            }
        }
    }


    if (
        IsKeyPressed(KEY_ENTER) ||
        IsKeyPressed(KEY_KP_ENTER) ||
        IsKeyPressed(KEY_ESCAPE)
    )
    {
        field->active =
            false;

        field->replace_on_type =
            false;
    }
}


static void draw_numeric_field(
    NumericField *field,
    Rectangle bounds
)
{
    Color background =
        field->active
        ? (Color){55, 64, 83, 255}
        : (Color){38, 43, 55, 255};


    Color border =
        field->active
        ? (Color){120, 165, 230, 255}
        : (Color){83, 91, 111, 255};


    DrawRectangleRec(
        bounds,
        background
    );


    DrawRectangleLinesEx(
        bounds,
        2.0f,
        border
    );


    /*
     * Highlight the value after click to make "type to replace" obvious.
     */
    if (
        field->active &&
        field->replace_on_type
    )
    {
        int text_width =
            MeasureText(
                field->text,
                20
            );


        Rectangle selected =
        {
            bounds.x + 7.0f,
            bounds.y + 5.0f,
            (float)text_width + 8.0f,
            bounds.height - 10.0f
        };


        DrawRectangleRec(
            selected,
            (Color){75, 104, 149, 255}
        );
    }


    DrawText(
        field->text,
        (int)bounds.x + 10,
        (int)bounds.y + 8,
        20,
        RAYWHITE
    );


    /*
     * Blinking caret after the user starts typing.
     */
    if (
        field->active &&
        !field->replace_on_type &&
        ((int)(GetTime() * 2.0) % 2 == 0)
    )
    {
        int text_width =
            MeasureText(
                field->text,
                20
            );


        int caret_x =
            (int)bounds.x +
            10 +
            text_width +
            1;


        DrawLine(
            caret_x,
            (int)bounds.y + 7,
            caret_x,
            (int)(bounds.y + bounds.height - 7),
            RAYWHITE
        );
    }
}


static int button(
    Rectangle bounds,
    const char *text
)
{
    Vector2 mouse =
        hmi_mouse_position();

    bool hovered =
        CheckCollisionPointRec(
            mouse,
            bounds
        );

    Color fill =
        hovered
        ? (Color){72, 84, 108, 255}
        : (Color){51, 59, 77, 255};

    DrawRectangleRounded(
        bounds,
        0.12f,
        8,
        fill
    );

    DrawRectangleLinesEx(
        bounds,
        1.5f,
        (Color){105, 117, 145, 255}
    );

    int font_size = 20;

    int width =
        MeasureText(
            text,
            font_size
        );

    DrawText(
        text,
        (int)(
            bounds.x +
            bounds.width / 2.0f -
            width / 2.0f
        ),
        (int)(
            bounds.y +
            bounds.height / 2.0f -
            font_size / 2.0f
        ),
        font_size,
        RAYWHITE
    );

    return
        hovered &&
        IsMouseButtonPressed(
            MOUSE_BUTTON_LEFT
        );
}


/* ============================================================================
 * UDP SERIALIZATION
 * ============================================================================
 */

static uint32_t float_to_network_word(
    float value
)
{
    uint32_t bits = 0;

    memcpy(
        &bits,
        &value,
        sizeof(bits)
    );

    return htonl(bits);
}


static int send_plan_command(
    int socket_fd,
    const struct sockaddr_in *controller_address,
    uint32_t sequence,
    const float waypoint_a[NUM_POSE_VALUES],
    const float waypoint_b[NUM_POSE_VALUES],
    float tcp_speed,
    float tcp_accel,
    float tcp_jerk
)
{
    uint32_t packet[
        3 + PLAN_FLOAT_COUNT
    ];

    packet[0] =
        htonl(PACKET_MAGIC);

    packet[1] =
        htonl(
            (uint32_t)COMMAND_PLAN_AND_RUN_LINE
        );

    packet[2] =
        htonl(sequence);


    int index = 3;

    for (int i = 0;
         i < NUM_POSE_VALUES;
         i++)
    {
        packet[index++] =
            float_to_network_word(
                waypoint_a[i]
            );
    }

    for (int i = 0;
         i < NUM_POSE_VALUES;
         i++)
    {
        packet[index++] =
            float_to_network_word(
                waypoint_b[i]
            );
    }

    packet[index++] =
        float_to_network_word(
            tcp_speed
        );

    packet[index++] =
        float_to_network_word(
            tcp_accel
        );

    packet[index++] =
        float_to_network_word(
            tcp_jerk
        );


    ssize_t sent =
        sendto(
            socket_fd,
            packet,
            sizeof(packet),
            0,
            (const struct sockaddr *)controller_address,
            sizeof(*controller_address)
        );

    return
        sent ==
        (ssize_t)sizeof(packet);
}


static int send_stop_command(
    int socket_fd,
    const struct sockaddr_in *controller_address,
    uint32_t sequence
)
{
    uint32_t packet[3];

    packet[0] =
        htonl(PACKET_MAGIC);

    packet[1] =
        htonl(
            (uint32_t)COMMAND_STOP
        );

    packet[2] =
        htonl(sequence);


    ssize_t sent =
        sendto(
            socket_fd,
            packet,
            sizeof(packet),
            0,
            (const struct sockaddr *)controller_address,
            sizeof(*controller_address)
        );

    return
        sent ==
        (ssize_t)sizeof(packet);
}


/* ============================================================================
 * CONTROLLER LIVE STATUS
 * ============================================================================
 */

typedef struct
{
    bool valid;

    double lastReceiveTime;

    uint32_t motionState;
    uint32_t lastSequence;

    uint32_t trajectoryIndex;
    uint32_t trajectoryCount;

    uint32_t wkc;
    uint32_t expectedWkc;

    uint16_t statusword[6];

} ControllerStatus;


static const char *motion_state_name_hmi(
    uint32_t state
)
{
    switch (state)
    {
        case 0:
            return "WAITING";

        case 1:
            return "PLANNING";

        case 2:
            return "PREPOSITION";

        case 3:
            return "RUNNING";

        case 4:
            return "FINISHED";

        case 5:
            return "HOLD";

        default:
            return "UNKNOWN";
    }
}


static const char *cia402_state_name(
    uint16_t statusword
)
{
    uint16_t state =
        statusword &
        0x006F;


    switch (state)
    {
        case 0x0000:
            return "NOT READY";

        case 0x0040:
            return "SWITCH ON DISABLED";

        case 0x0021:
            return "READY TO SWITCH ON";

        case 0x0023:
            return "SWITCHED ON";

        case 0x0027:
            return "OPERATION ENABLED";

        case 0x0007:
            return "QUICK STOP ACTIVE";

        case 0x000F:
            return "FAULT REACTION";

        case 0x0008:
            return "FAULT";

        default:
            return "UNKNOWN";
    }
}


static void receive_controller_status(
    int status_socket,
    ControllerStatus *status
)
{
    if (
        status_socket < 0 ||
        status == NULL
    )
    {
        return;
    }


    for (;;)
    {
        uint32_t packet[
            STATUS_WORD_COUNT
        ];


        ssize_t received =
            recvfrom(
                status_socket,
                packet,
                sizeof(packet),
                MSG_DONTWAIT,
                NULL,
                NULL
            );


        if (received < 0)
        {
            if (
                errno == EAGAIN ||
                errno == EWOULDBLOCK
            )
            {
                break;
            }


            perror("status recvfrom");
            break;
        }


        if (
            received !=
            (ssize_t)sizeof(packet)
        )
        {
            continue;
        }


        if (
            ntohl(packet[0]) !=
            STATUS_PACKET_MAGIC
        )
        {
            continue;
        }


        status->motionState =
            ntohl(
                packet[1]
            );


        status->lastSequence =
            ntohl(
                packet[2]
            );


        status->trajectoryIndex =
            ntohl(
                packet[3]
            );


        status->trajectoryCount =
            ntohl(
                packet[4]
            );


        status->wkc =
            ntohl(
                packet[5]
            );


        status->expectedWkc =
            ntohl(
                packet[6]
            );


        for (int joint = 0;
             joint < 6;
             joint++)
        {
            status->statusword[joint] =
                (uint16_t)ntohl(
                    packet[7 + joint]
                );
        }


        status->valid =
            true;

        status->lastReceiveTime =
            GetTime();
    }
}


/* ============================================================================
 * REFERENCE VALUES
 * ============================================================================
 *
 * These reproduce the current validated SingleSegmentTest_line case:
 *
 *      qStart = [30 -45 60 20 -30 45] deg
 *
 * which gives approximately:
 *
 *      A = [-0.421962 -0.451956 0.234228] m
 *
 * and:
 *
 *      B = A + [0.50 0 0] m
 *
 * The Euler values are the corresponding ZYX yaw/pitch/roll values used by
 * the validated reference case.
 * ============================================================================
 */

static void load_reference_case(
    NumericField a_fields[NUM_POSE_VALUES],
    NumericField b_fields[NUM_POSE_VALUES],
    NumericField *speed,
    NumericField *accel,
    NumericField *jerk
)
{
    const double A[NUM_POSE_VALUES] =
    {
        -0.421962,
        -0.451956,
         0.234228,
       104.80,
       -68.51,
        38.48
    };

    const double B[NUM_POSE_VALUES] =
    {
         0.078038,
        -0.451956,
         0.234228,
       147.58,
       -58.31,
        12.75
    };

    for (int i = 0;
         i < NUM_POSE_VALUES;
         i++)
    {
        int decimals =
            i < 3
            ? 6
            : 2;

        numeric_field_set(
            &a_fields[i],
            A[i],
            decimals
        );

        numeric_field_set(
            &b_fields[i],
            B[i],
            decimals
        );
    }

    numeric_field_set(
        speed,
        0.10,
        3
    );

    numeric_field_set(
        accel,
        0.25,
        3
    );

    numeric_field_set(
        jerk,
        1.00,
        3
    );
}


/* ============================================================================
 * MAIN
 * ============================================================================
 */

int main(void)
{
    /* ------------------------------------------------------------------------
     * UDP
     * ------------------------------------------------------------------------
     */

    int command_socket =
        socket(
            AF_INET,
            SOCK_DGRAM,
            0
        );

    if (command_socket < 0)
    {
        perror("socket");
        return 1;
    }


    struct sockaddr_in controller_address;

    memset(
        &controller_address,
        0,
        sizeof(controller_address)
    );

    controller_address.sin_family =
        AF_INET;

    controller_address.sin_port =
        htons(
            CONTROLLER_PORT
        );

    if (
        inet_pton(
            AF_INET,
            CONTROLLER_IP,
            &controller_address.sin_addr
        ) != 1
    )
    {
        fprintf(
            stderr,
            "Invalid controller IP\n"
        );

        close(command_socket);
        return 1;
    }


    /*
     * Controller -> HMI live state feedback socket.
     *
     * Controller sends STA2 packets to UDP :5007.
     */
    int status_socket =
        socket(
            AF_INET,
            SOCK_DGRAM,
            0
        );


    if (status_socket < 0)
    {
        perror("status socket");

        close(command_socket);
        return 1;
    }


    struct sockaddr_in status_address;

    memset(
        &status_address,
        0,
        sizeof(status_address)
    );


    status_address.sin_family =
        AF_INET;

    status_address.sin_port =
        htons(
            STATUS_PORT
        );

    status_address.sin_addr.s_addr =
        htonl(
            INADDR_ANY
        );


    if (
        bind(
            status_socket,
            (struct sockaddr *)&status_address,
            sizeof(status_address)
        ) < 0
    )
    {
        perror("status bind");

        close(status_socket);
        close(command_socket);

        return 1;
    }


    /* ------------------------------------------------------------------------
     * HMI STATE
     * ------------------------------------------------------------------------
     */

    NumericField waypoint_a[
        NUM_POSE_VALUES
    ] = {0};

    NumericField waypoint_b[
        NUM_POSE_VALUES
    ] = {0};

    NumericField speed_field = {0};
    NumericField accel_field = {0};
    NumericField jerk_field  = {0};

    load_reference_case(
        waypoint_a,
        waypoint_b,
        &speed_field,
        &accel_field,
        &jerk_field
    );


    uint32_t sequence = 0;


    ControllerStatus controller_status;

    memset(
        &controller_status,
        0,
        sizeof(controller_status)
    );


    char status_text[256];

    snprintf(
        status_text,
        sizeof(status_text),
        "Ready - edit A/B then press PLAN + RUN"
    );


    /* ------------------------------------------------------------------------
     * WINDOW
     * ------------------------------------------------------------------------
     */

    const int window_width  = 1400;
const int window_height = 760;


/*
 * Allow the user to resize the desktop HMI window.
 */
SetConfigFlags(
    FLAG_WINDOW_RESIZABLE
);


InitWindow(
    window_width,
    window_height,
    "Robot Controller - Cartesian Trajectory HMI"
);

    SetTargetFPS(60);


    const char *pose_labels[
        NUM_POSE_VALUES
    ] =
    {
        "X [m]",
        "Y [m]",
        "Z [m]",
        "Yaw Z [deg]",
        "Pitch Y [deg]",
        "Roll X [deg]"
    };


    /* ------------------------------------------------------------------------
     * GUI LOOP
     * ------------------------------------------------------------------------
     */

    while (!WindowShouldClose())
    {
        /*
         * Drain the newest live controller status before rendering.
         */
        receive_controller_status(
            status_socket,
            &controller_status
        );
        update_horizontal_scroll();


Camera2D scroll_camera =
{
    0
};


scroll_camera.offset =
    (Vector2)
    {
        -hmi_scroll_x,
        0.0f
    };


scroll_camera.target =
    (Vector2)
    {
        0.0f,
        0.0f
    };


scroll_camera.rotation =
    0.0f;


scroll_camera.zoom =
    1.0f;


        BeginDrawing();

        ClearBackground(
            (Color){24, 27, 35, 255}
        );

        BeginMode2D(
    scroll_camera
);

        /* Header */
        DrawText(
            "CARTESIAN TRAJECTORY HMI",
            40,
            26,
            30,
            RAYWHITE
        );

        DrawText(
            "SingleSegment line: A -> B | S-curve | SLERP | ADLS IK | 1 ms CSP",
            40,
            66,
            18,
            (Color){164, 171, 190, 255}
        );

        DrawText(
            "Click a value, then type to replace it | Enter = commit",
            960,
            68,
            14,
            (Color){126, 138, 164, 255}
        );


        DrawLine(
            40,
            102,
            1360,
            102,
            (Color){67, 74, 91, 255}
        );


        /* --------------------------------------------------------------------
         * WAYPOINT PANELS
         * --------------------------------------------------------------------
         */

        Rectangle panel_a =
        {
            40.0f,
            125.0f,
            490.0f,
            390.0f
        };

        Rectangle panel_b =
        {
            590.0f,
            125.0f,
            490.0f,
            390.0f
        };

        DrawRectangleRounded(
            panel_a,
            0.03f,
            8,
            (Color){30, 34, 44, 255}
        );

        DrawRectangleRounded(
            panel_b,
            0.03f,
            8,
            (Color){30, 34, 44, 255}
        );

        DrawRectangleLinesEx(
            panel_a,
            1.0f,
            (Color){65, 73, 92, 255}
        );

        DrawRectangleLinesEx(
            panel_b,
            1.0f,
            (Color){65, 73, 92, 255}
        );


        DrawText(
            "WAYPOINT A",
            65,
            145,
            24,
            RAYWHITE
        );

        DrawText(
            "Start Cartesian pose",
            65,
            176,
            16,
            (Color){155, 163, 181, 255}
        );


        DrawText(
            "WAYPOINT B",
            615,
            145,
            24,
            RAYWHITE
        );

        DrawText(
            "End Cartesian pose",
            615,
            176,
            16,
            (Color){155, 163, 181, 255}
        );


        for (int i = 0;
             i < NUM_POSE_VALUES;
             i++)
        {
            int y =
                215 +
                i * 46;

            DrawText(
                pose_labels[i],
                65,
                y + 8,
                17,
                (Color){202, 207, 220, 255}
            );

            DrawText(
                pose_labels[i],
                615,
                y + 8,
                17,
                (Color){202, 207, 220, 255}
            );


            Rectangle a_bounds =
            {
                250.0f,
                (float)y,
                240.0f,
                36.0f
            };

            Rectangle b_bounds =
            {
                800.0f,
                (float)y,
                240.0f,
                36.0f
            };


            update_numeric_field(
                &waypoint_a[i],
                a_bounds
            );

            update_numeric_field(
                &waypoint_b[i],
                b_bounds
            );

            draw_numeric_field(
                &waypoint_a[i],
                a_bounds
            );

            draw_numeric_field(
                &waypoint_b[i],
                b_bounds
            );
        }


        /* --------------------------------------------------------------------
         * MOTION PROFILE
         * --------------------------------------------------------------------
         */

        DrawText(
            "MOTION PROFILE",
            40,
            540,
            22,
            RAYWHITE
        );


        const char *motion_labels[3] =
        {
            "TCP speed [m/s]",
            "TCP accel [m/s^2]",
            "TCP jerk [m/s^3]"
        };

        NumericField *motion_fields[3] =
        {
            &speed_field,
            &accel_field,
            &jerk_field
        };


        for (int i = 0;
             i < 3;
             i++)
        {
            int x =
                40 +
                i * 245;

            DrawText(
                motion_labels[i],
                x,
                574,
                16,
                (Color){166, 174, 194, 255}
            );

            Rectangle bounds =
            {
                (float)x,
                600.0f,
                205.0f,
                38.0f
            };

            update_numeric_field(
                motion_fields[i],
                bounds
            );

            draw_numeric_field(
                motion_fields[i],
                bounds
            );
        }


        /* --------------------------------------------------------------------
         * BUTTONS
         * --------------------------------------------------------------------
         */

        Rectangle run_button =
        {
            785.0f,
            550.0f,
            295.0f,
            55.0f
        };

        Rectangle reference_button =
        {
            785.0f,
            615.0f,
            140.0f,
            48.0f
        };

        Rectangle stop_button =
        {
            940.0f,
            615.0f,
            140.0f,
            48.0f
        };


        if (
            button(
                run_button,
                "PLAN + RUN A -> B"
            )
        )
        {
            float A[
                NUM_POSE_VALUES
            ];

            float B[
                NUM_POSE_VALUES
            ];

            for (int i = 0;
                 i < NUM_POSE_VALUES;
                 i++)
            {
                A[i] =
                    (float)numeric_field_value(
                        &waypoint_a[i]
                    );

                B[i] =
                    (float)numeric_field_value(
                        &waypoint_b[i]
                    );
            }


            float speed =
                (float)numeric_field_value(
                    &speed_field
                );

            float accel =
                (float)numeric_field_value(
                    &accel_field
                );

            float jerk =
                (float)numeric_field_value(
                    &jerk_field
                );


            if (
                speed <= 0.0f ||
                accel <= 0.0f ||
                jerk <= 0.0f
            )
            {
                snprintf(
                    status_text,
                    sizeof(status_text),
                    "ERROR: speed, acceleration and jerk must be > 0"
                );
            }
            else
            {
                sequence++;

                int success =
                    send_plan_command(
                        command_socket,
                        &controller_address,
                        sequence,
                        A,
                        B,
                        speed,
                        accel,
                        jerk
                    );

                if (success)
                {
                    snprintf(
                        status_text,
                        sizeof(status_text),
                        "Sent PLAN + RUN #%u | A -> B",
                        sequence
                    );

                    printf(
                        "\nPLAN + RUN #%u\n"
                        "A: p=[%.6f %.6f %.6f] m | YPR=[%.2f %.2f %.2f] deg\n"
                        "B: p=[%.6f %.6f %.6f] m | YPR=[%.2f %.2f %.2f] deg\n"
                        "Profile: speed=%.4f m/s accel=%.4f m/s^2 jerk=%.4f m/s^3\n",
                        sequence,
                        A[0], A[1], A[2],
                        A[3], A[4], A[5],
                        B[0], B[1], B[2],
                        B[3], B[4], B[5],
                        speed,
                        accel,
                        jerk
                    );

                    fflush(stdout);
                }
                else
                {
                    snprintf(
                        status_text,
                        sizeof(status_text),
                        "UDP send failed"
                    );
                }
            }
        }


        if (
            button(
                reference_button,
                "REFERENCE"
            )
        )
        {
            load_reference_case(
                waypoint_a,
                waypoint_b,
                &speed_field,
                &accel_field,
                &jerk_field
            );

            snprintf(
                status_text,
                sizeof(status_text),
                "Loaded validated SingleSegmentTest_line reference"
            );
        }


        if (
            button(
                stop_button,
                "STOP"
            )
        )
        {
            sequence++;

            int success =
                send_stop_command(
                    command_socket,
                    &controller_address,
                    sequence
                );

            if (success)
            {
                snprintf(
                    status_text,
                    sizeof(status_text),
                    "STOP command #%u sent",
                    sequence
                );

                printf(
                    "STOP command #%u sent\n",
                    sequence
                );
            }
            else
            {
                snprintf(
                    status_text,
                    sizeof(status_text),
                    "STOP UDP send failed"
                );
            }
        }


        /* --------------------------------------------------------------------
         * LIVE ROBOT / STATE-MACHINE PANEL
         * --------------------------------------------------------------------
         */

        Rectangle state_panel =
        {
            1110.0f,
            125.0f,
            250.0f,
            538.0f
        };


        DrawRectangleRounded(
            state_panel,
            0.03f,
            8,
            (Color){30, 34, 44, 255}
        );


        DrawRectangleLinesEx(
            state_panel,
            1.0f,
            (Color){65, 73, 92, 255}
        );


        DrawText(
            "ROBOT STATE",
            1130,
            145,
            24,
            RAYWHITE
        );


        bool status_online =
            controller_status.valid &&
            (
                GetTime() -
                controller_status.lastReceiveTime
            ) < 1.0;


        DrawText(
            status_online
                ? "LINK  ONLINE"
                : "LINK  OFFLINE",
            1130,
            180,
            16,
            status_online
                ? (Color){133, 220, 166, 255}
                : (Color){222, 132, 132, 255}
        );


        DrawText(
            "CONTROLLER",
            1130,
            220,
            15,
            (Color){155, 163, 181, 255}
        );


        DrawText(
            status_online
                ? motion_state_name_hmi(
                    controller_status.motionState
                )
                : "---",
            1130,
            244,
            22,
            status_online
                ? (Color){193, 214, 255, 255}
                : (Color){126, 134, 151, 255}
        );


        char live_line[96];


        snprintf(
            live_line,
            sizeof(live_line),
            "Trajectory  %u / %u",
            controller_status.trajectoryCount > 0
                ? controller_status.trajectoryIndex + 1
                : 0,
            controller_status.trajectoryCount
        );


        DrawText(
            live_line,
            1130,
            282,
            15,
            (Color){183, 190, 207, 255}
        );


        snprintf(
            live_line,
            sizeof(live_line),
            "WKC         %u / %u",
            controller_status.wkc,
            controller_status.expectedWkc
        );


        DrawText(
            live_line,
            1130,
            306,
            15,
            (
                status_online &&
                controller_status.wkc ==
                controller_status.expectedWkc
            )
                ? (Color){133, 220, 166, 255}
                : (Color){222, 166, 132, 255}
        );


        snprintf(
            live_line,
            sizeof(live_line),
            "Last cmd    #%u",
            controller_status.lastSequence
        );


        DrawText(
            live_line,
            1130,
            330,
            15,
            (Color){183, 190, 207, 255}
        );


        DrawLine(
            1130,
            364,
            1340,
            364,
            (Color){67, 74, 91, 255}
        );


        DrawText(
            "CiA-402 DRIVES",
            1130,
            382,
            16,
            RAYWHITE
        );


        for (int joint = 0;
             joint < 6;
             joint++)
        {
            int y =
                418 +
                joint * 34;


            char joint_label[8];

            snprintf(
                joint_label,
                sizeof(joint_label),
                "J%d",
                joint + 1
            );


            DrawText(
                joint_label,
                1130,
                y,
                16,
                (Color){202, 207, 220, 255}
            );


            const char *drive_state =
                status_online
                ? cia402_state_name(
                    controller_status.statusword[joint]
                )
                : "---";


            bool operation_enabled =
                status_online &&
                (
                    controller_status.statusword[joint] &
                    0x006F
                ) == 0x0027;


            DrawText(
                drive_state,
                1170,
                y,
                14,
                operation_enabled
                    ? (Color){133, 220, 166, 255}
                    : (Color){202, 207, 220, 255}
            );
        }


        /* --------------------------------------------------------------------
         * FOOTER / STATUS
         * --------------------------------------------------------------------
         */

        DrawLine(
            40,
            685,
            1360,
            685,
            (Color){67, 74, 91, 255}
        );

        DrawText(
            status_text,
            40,
            704,
            17,
            (Color){181, 191, 215, 255}
        );

        DrawText(
            "CMD :5006 RBT2 | STATUS :5007 STA2",
            1050,
            704,
            16,
            (Color){139, 148, 168, 255}
        );


        /*
 * Finish drawing the scrollable 1400 px HMI canvas.
 */
EndMode2D();


/*
 * The scrollbar itself stays fixed to the physical window.
 */
draw_horizontal_scrollbar();


EndDrawing();
    }


    /* ------------------------------------------------------------------------
     * CLEANUP
     * ------------------------------------------------------------------------
     */

    close(
        status_socket
    );


    close(
        command_socket
    );

    CloseWindow();

    return 0;
}
