#include "hmi_protocol.h"

#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

static double monotonic_seconds(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
    {
        return 0.0;
    }

    return
        (double)ts.tv_sec +
        (double)ts.tv_nsec / 1000000000.0;
}

static uint32_t float_to_network_word(float value)
{
    uint32_t bits = 0U;
    memcpy(&bits, &value, sizeof(bits));
    return htonl(bits);
}

static float network_word_to_float(uint32_t value)
{
    uint32_t bits = ntohl(value);
    float result = 0.0F;
    memcpy(&result, &bits, sizeof(result));
    return result;
}

static bool send_packet(
    HmiProtocol *protocol,
    uint32_t command_code,
    float arg0,
    float arg1,
    float arg2,
    float arg3
)
{
    if (
        protocol == NULL ||
        protocol->command_socket < 0 ||
        command_code == 0U
    )
    {
        return false;
    }

    uint32_t packet[HMI_COMMAND_WORD_COUNT];

    protocol->next_sequence++;

    packet[HMI_COMMAND_WORD_MAGIC] =
        htonl(HMI_COMMAND_MAGIC);

    packet[HMI_COMMAND_WORD_VERSION] =
        htonl(HMI_PROTOCOL_VERSION);

    packet[HMI_COMMAND_WORD_COMMAND] =
        htonl(command_code);

    packet[HMI_COMMAND_WORD_SEQUENCE] =
        htonl(protocol->next_sequence);

    packet[HMI_COMMAND_WORD_ARG0] =
        float_to_network_word(arg0);

    packet[HMI_COMMAND_WORD_ARG1] =
        float_to_network_word(arg1);

    packet[HMI_COMMAND_WORD_ARG2] =
        float_to_network_word(arg2);

    packet[HMI_COMMAND_WORD_ARG3] =
        float_to_network_word(arg3);

    const ssize_t sent =
        sendto(
            protocol->command_socket,
            packet,
            sizeof(packet),
            0,
            (const struct sockaddr *)&protocol->controller_address,
            sizeof(protocol->controller_address)
        );

    return
        sent == (ssize_t)sizeof(packet);
}

bool hmi_protocol_init(
    HmiProtocol *protocol,
    uint16_t status_port
)
{
    if (
        protocol == NULL ||
        status_port == 0U
    )
    {
        return false;
    }

    memset(
        protocol,
        0,
        sizeof(*protocol)
    );

    protocol->command_socket =
        -1;

    protocol->status_socket =
        -1;

    protocol->command_socket =
        socket(
            AF_INET,
            SOCK_DGRAM,
            0
        );

    if (protocol->command_socket < 0)
    {
        perror("HMI command socket");
        return false;
    }

    memset(
        &protocol->controller_address,
        0,
        sizeof(protocol->controller_address)
    );

    protocol->controller_address.sin_family =
        AF_INET;

    protocol->controller_address.sin_port =
        htons(HMI_CONTROLLER_PORT);

    if (
        inet_pton(
            AF_INET,
            HMI_CONTROLLER_IP,
            &protocol->controller_address.sin_addr
        ) != 1
    )
    {
        hmi_protocol_shutdown(protocol);
        return false;
    }

    protocol->status_socket =
        socket(
            AF_INET,
            SOCK_DGRAM,
            0
        );

    if (protocol->status_socket < 0)
    {
        perror("HMI status socket");
        hmi_protocol_shutdown(protocol);
        return false;
    }

    int reuse = 1;

    (void)setsockopt(
        protocol->status_socket,
        SOL_SOCKET,
        SO_REUSEADDR,
        &reuse,
        sizeof(reuse)
    );

    struct sockaddr_in status_address;

    memset(
        &status_address,
        0,
        sizeof(status_address)
    );

    status_address.sin_family =
        AF_INET;

    status_address.sin_port =
        htons(status_port);

    status_address.sin_addr.s_addr =
        htonl(INADDR_ANY);

    if (
        bind(
            protocol->status_socket,
            (struct sockaddr *)&status_address,
            sizeof(status_address)
        ) < 0
    )
    {
        perror("HMI status bind");
        hmi_protocol_shutdown(protocol);
        return false;
    }

    return true;
}

void hmi_protocol_shutdown(
    HmiProtocol *protocol
)
{
    if (protocol == NULL)
    {
        return;
    }

    if (protocol->status_socket >= 0)
    {
        close(protocol->status_socket);
        protocol->status_socket = -1;
    }

    if (protocol->command_socket >= 0)
    {
        close(protocol->command_socket);
        protocol->command_socket = -1;
    }
}

void hmi_protocol_poll_status(
    HmiProtocol *protocol
)
{
    if (
        protocol == NULL ||
        protocol->status_socket < 0
    )
    {
        return;
    }

    for (;;)
    {
        uint32_t packet[HMI_STATUS_WORD_COUNT];

        const ssize_t received =
            recvfrom(
                protocol->status_socket,
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

            perror("HMI status recvfrom");
            break;
        }

        if (received != (ssize_t)sizeof(packet))
        {
            continue;
        }

        if (
            ntohl(packet[HMI_STATUS_WORD_MAGIC]) !=
                HMI_STATUS_MAGIC
            ||
            ntohl(packet[HMI_STATUS_WORD_VERSION]) !=
                HMI_PROTOCOL_VERSION
        )
        {
            continue;
        }

        HmiStatus *status =
            &protocol->status;

        status->sequence =
            ntohl(packet[HMI_STATUS_WORD_SEQUENCE]);

        status->robot_state =
            ntohl(packet[HMI_STATUS_WORD_ROBOT_STATE]);

        status->selected_program =
            ntohl(packet[HMI_STATUS_WORD_SELECTED_PROGRAM]);

        status->estop_active =
            ntohl(packet[HMI_STATUS_WORD_ESTOP]) != 0U;

        status->paused =
            ntohl(packet[HMI_STATUS_WORD_PAUSED]) != 0U;

        status->guidance_active =
            ntohl(packet[HMI_STATUS_WORD_GUIDANCE_ACTIVE]) != 0U;

        status->boot_phase =
            ntohl(packet[HMI_STATUS_WORD_BOOT_PHASE]);

        status->boot_error =
            ntohl(packet[HMI_STATUS_WORD_BOOT_ERROR]);

        status->homing_phase =
            ntohl(packet[HMI_STATUS_WORD_HOMING_PHASE]);

        status->homing_error =
            ntohl(packet[HMI_STATUS_WORD_HOMING_ERROR]);

        status->idle_phase =
            ntohl(packet[HMI_STATUS_WORD_IDLE_PHASE]);

        status->idle_error =
            ntohl(packet[HMI_STATUS_WORD_IDLE_ERROR]);

        status->teaching_phase =
            ntohl(packet[HMI_STATUS_WORD_TEACHING_PHASE]);

        status->teaching_error =
            ntohl(packet[HMI_STATUS_WORD_TEACHING_ERROR]);

        status->teaching_message =
            ntohl(packet[HMI_STATUS_WORD_TEACHING_MESSAGE]);

        status->teaching_next_point =
            ntohl(packet[HMI_STATUS_WORD_TEACHING_NEXT_POINT]);

        status->teaching_segment_count =
            ntohl(packet[HMI_STATUS_WORD_TEACHING_SEGMENT_COUNT]);

        status->teaching_speed_mps =
            network_word_to_float(
                packet[HMI_STATUS_WORD_TEACHING_SPEED]
            );

        status->record_allowed =
            ntohl(packet[HMI_STATUS_WORD_RECORD_ALLOWED]) != 0U;

        status->validate_allowed =
            ntohl(packet[HMI_STATUS_WORD_VALIDATE_ALLOWED]) != 0U;

        status->path_validation_phase =
            ntohl(packet[HMI_STATUS_WORD_PV_PHASE]);

        status->path_validation_result =
            ntohl(packet[HMI_STATUS_WORD_PV_RESULT]);

        status->path_validation_error =
            ntohl(packet[HMI_STATUS_WORD_PV_ERROR]);

        status->path_validation_progress =
            network_word_to_float(
                packet[HMI_STATUS_WORD_PV_PROGRESS]
            );

        status->approach_phase =
            ntohl(packet[HMI_STATUS_WORD_APPROACH_PHASE]);

        status->approach_result =
            ntohl(packet[HMI_STATUS_WORD_APPROACH_RESULT]);

        status->approach_error =
            ntohl(packet[HMI_STATUS_WORD_APPROACH_ERROR]);

        status->approach_progress =
            network_word_to_float(
                packet[HMI_STATUS_WORD_APPROACH_PROGRESS]
            );

        status->preview_active =
            ntohl(packet[HMI_STATUS_WORD_PREVIEW_ACTIVE]) != 0U;

        status->preview_complete =
            ntohl(packet[HMI_STATUS_WORD_PREVIEW_COMPLETE]) != 0U;

        status->preview_error =
            ntohl(packet[HMI_STATUS_WORD_PREVIEW_ERROR]);

        status->preview_progress =
            network_word_to_float(
                packet[HMI_STATUS_WORD_PREVIEW_PROGRESS]
            );

        status->actual_tcp_m[0] =
            network_word_to_float(
                packet[HMI_STATUS_WORD_TCP_X]
            );

        status->actual_tcp_m[1] =
            network_word_to_float(
                packet[HMI_STATUS_WORD_TCP_Y]
            );

        status->actual_tcp_m[2] =
            network_word_to_float(
                packet[HMI_STATUS_WORD_TCP_Z]
            );

        for (
            int joint = 0;
            joint < HMI_NUM_JOINTS;
            ++joint
        )
        {
            status->actual_joint_rad[joint] =
                network_word_to_float(
                    packet[
                        HMI_STATUS_WORD_Q1 +
                        joint
                    ]
                );
        }

        status->can_ready_nodes =
            ntohl(packet[HMI_STATUS_WORD_CAN_READY_NODES]);

        status->can_expected_nodes =
            ntohl(packet[HMI_STATUS_WORD_CAN_EXPECTED_NODES]);

        status->recorded_count =
            ntohl(packet[HMI_STATUS_WORD_RECORDED_COUNT]);

        for (
            int point = 0;
            point < HMI_MAX_RECORDED_POINTS;
            ++point
        )
        {
            for (
                int axis = 0;
                axis < 3;
                ++axis
            )
            {
                status->recorded_tcp_m[point][axis] =
                    network_word_to_float(
                        packet[
                            HMI_STATUS_WORD_P1_X +
                            point * 3 +
                            axis
                        ]
                    );
            }
        }

        status->last_command_sequence =
            ntohl(
                packet[
                    HMI_STATUS_WORD_LAST_COMMAND_SEQUENCE
                ]
            );

        status->guidance_error =
            ntohl(
                packet[
                    HMI_STATUS_WORD_GUIDANCE_ERROR
                ]
            );

        protocol->status_valid =
            true;

        protocol->last_receive_time =
            monotonic_seconds();
    }
}

bool hmi_protocol_controller_online(
    const HmiProtocol *protocol,
    double timeout_seconds
)
{
    if (
        protocol == NULL ||
        !protocol->status_valid ||
        timeout_seconds <= 0.0
    )
    {
        return false;
    }

    return
        monotonic_seconds() -
        protocol->last_receive_time
        <
        timeout_seconds;
}

bool hmi_protocol_send_event(
    HmiProtocol *protocol,
    HmiEvent event
)
{
    if (!hmi_event_is_valid(event))
    {
        return false;
    }

    return
        send_packet(
            protocol,
            (uint32_t)event,
            0.0F,
            0.0F,
            0.0F,
            0.0F
        );
}


bool hmi_protocol_send_sim_estop_toggle(
    HmiProtocol *protocol
)
{
    return
        send_packet(
            protocol,
            HMI_PROTOCOL_SIM_ESTOP_TOGGLE,
            0.0F,
            0.0F,
            0.0F,
            0.0F
        );
}

bool hmi_protocol_send_guidance_pose(
    HmiProtocol *protocol,
    float x_m,
    float y_m,
    float z_m
)
{
    return
        send_packet(
            protocol,
            HMI_PROTOCOL_SIM_GUIDANCE_POSE,
            x_m,
            y_m,
            z_m,
            0.0F
        );
}
