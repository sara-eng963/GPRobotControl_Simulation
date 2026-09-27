#ifndef HMI_PROTOCOL_H
#define HMI_PROTOCOL_H

#include "hmi_types.h"

#include <stdbool.h>
#include <stdint.h>
#include <netinet/in.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HMI_CONTROLLER_IP          "127.0.0.1"
#define HMI_CONTROLLER_PORT        5010
#define HMI_PANEL_STATUS_PORT      5011
#define HMI_TEACH_STATUS_PORT      5012

#define HMI_COMMAND_MAGIC          0x484D4933U /* HMI3 */
#define HMI_STATUS_MAGIC           0x53544D33U /* STM3 */
#define HMI_PROTOCOL_VERSION       3U

#define HMI_COMMAND_WORD_COUNT     8U
#define HMI_STATUS_WORD_COUNT      57U

/*
 * PC-simulation transport commands.
 * These are intentionally outside the stable HmiEvent API.
 */
#define HMI_PROTOCOL_SIM_GUIDANCE_POSE  0x80000001U
#define HMI_PROTOCOL_SIM_ESTOP_TOGGLE   0x80000002U



typedef enum
{
    HMI_COMMAND_WORD_MAGIC = 0,
    HMI_COMMAND_WORD_VERSION,
    HMI_COMMAND_WORD_COMMAND,
    HMI_COMMAND_WORD_SEQUENCE,
    HMI_COMMAND_WORD_ARG0,
    HMI_COMMAND_WORD_ARG1,
    HMI_COMMAND_WORD_ARG2,
    HMI_COMMAND_WORD_ARG3

} HmiCommandWord;

typedef enum
{
    HMI_STATUS_WORD_MAGIC = 0,
    HMI_STATUS_WORD_VERSION,
    HMI_STATUS_WORD_SEQUENCE,
    HMI_STATUS_WORD_ROBOT_STATE,
    HMI_STATUS_WORD_SELECTED_PROGRAM,
    HMI_STATUS_WORD_ESTOP,
    HMI_STATUS_WORD_PAUSED,
    HMI_STATUS_WORD_GUIDANCE_ACTIVE,

    HMI_STATUS_WORD_BOOT_PHASE,
    HMI_STATUS_WORD_BOOT_ERROR,
    HMI_STATUS_WORD_HOMING_PHASE,
    HMI_STATUS_WORD_HOMING_ERROR,
    HMI_STATUS_WORD_IDLE_PHASE,
    HMI_STATUS_WORD_IDLE_ERROR,

    HMI_STATUS_WORD_TEACHING_PHASE,
    HMI_STATUS_WORD_TEACHING_ERROR,
    HMI_STATUS_WORD_TEACHING_MESSAGE,
    HMI_STATUS_WORD_TEACHING_NEXT_POINT,
    HMI_STATUS_WORD_TEACHING_SEGMENT_COUNT,
    HMI_STATUS_WORD_TEACHING_SPEED,
    HMI_STATUS_WORD_RECORD_ALLOWED,
    HMI_STATUS_WORD_VALIDATE_ALLOWED,

    HMI_STATUS_WORD_PV_PHASE,
    HMI_STATUS_WORD_PV_RESULT,
    HMI_STATUS_WORD_PV_ERROR,
    HMI_STATUS_WORD_PV_PROGRESS,

    HMI_STATUS_WORD_APPROACH_PHASE,
    HMI_STATUS_WORD_APPROACH_RESULT,
    HMI_STATUS_WORD_APPROACH_ERROR,
    HMI_STATUS_WORD_APPROACH_PROGRESS,

    HMI_STATUS_WORD_PREVIEW_ACTIVE,
    HMI_STATUS_WORD_PREVIEW_COMPLETE,
    HMI_STATUS_WORD_PREVIEW_ERROR,
    HMI_STATUS_WORD_PREVIEW_PROGRESS,

    HMI_STATUS_WORD_TCP_X,
    HMI_STATUS_WORD_TCP_Y,
    HMI_STATUS_WORD_TCP_Z,

    HMI_STATUS_WORD_Q1,
    HMI_STATUS_WORD_Q2,
    HMI_STATUS_WORD_Q3,
    HMI_STATUS_WORD_Q4,
    HMI_STATUS_WORD_Q5,
    HMI_STATUS_WORD_Q6,

    HMI_STATUS_WORD_WKC,
    HMI_STATUS_WORD_EXPECTED_WKC,

    HMI_STATUS_WORD_RECORDED_COUNT,

    HMI_STATUS_WORD_P1_X,
    HMI_STATUS_WORD_P1_Y,
    HMI_STATUS_WORD_P1_Z,
    HMI_STATUS_WORD_P2_X,
    HMI_STATUS_WORD_P2_Y,
    HMI_STATUS_WORD_P2_Z,
    HMI_STATUS_WORD_P3_X,
    HMI_STATUS_WORD_P3_Y,
    HMI_STATUS_WORD_P3_Z,

    HMI_STATUS_WORD_LAST_COMMAND_SEQUENCE,
    HMI_STATUS_WORD_GUIDANCE_ERROR

} HmiStatusWord;

typedef struct
{
    int command_socket;
    int status_socket;

    struct sockaddr_in controller_address;

    HmiStatus status;

    bool status_valid;
    double last_receive_time;

    uint32_t next_sequence;
} HmiProtocol;

bool hmi_protocol_init(
    HmiProtocol *protocol,
    uint16_t status_port
);

void hmi_protocol_shutdown(
    HmiProtocol *protocol
);

void hmi_protocol_poll_status(
    HmiProtocol *protocol
);

bool hmi_protocol_controller_online(
    const HmiProtocol *protocol,
    double timeout_seconds
);

bool hmi_protocol_send_event(
    HmiProtocol *protocol,
    HmiEvent event
);

bool hmi_protocol_send_sim_estop_toggle(
    HmiProtocol *protocol
);

bool hmi_protocol_send_guidance_pose(
    HmiProtocol *protocol,
    float x_m,
    float y_m,
    float z_m
);

#ifdef __cplusplus
}
#endif

#endif /* HMI_PROTOCOL_H */
