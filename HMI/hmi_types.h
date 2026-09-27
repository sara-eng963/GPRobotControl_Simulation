#ifndef HMI_TYPES_H
#define HMI_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#define HMI_NUM_JOINTS 6
#define HMI_MAX_RECORDED_POINTS 3

typedef enum
{
    HMI_PROGRAM_NONE = 0,
    HMI_PROGRAM_LINE,
    HMI_PROGRAM_ARC,
    HMI_PROGRAM_CIRCLE
} HmiProgramSelection;

typedef struct
{
    bool valid;
    double last_receive_time;

    uint32_t sequence;
    uint32_t robot_state;
    uint32_t selected_program;

    bool estop_active;
    bool paused;
    bool guidance_active;

    uint32_t boot_phase;
    uint32_t boot_error;

    uint32_t homing_phase;
    uint32_t homing_error;

    uint32_t idle_phase;
    uint32_t idle_error;

    uint32_t teaching_phase;
    uint32_t teaching_error;
    uint32_t teaching_message;
    uint32_t teaching_next_point;
    uint32_t teaching_segment_count;
    float teaching_speed_mps;
    bool record_allowed;
    bool validate_allowed;

    uint32_t path_validation_phase;
    uint32_t path_validation_result;
    uint32_t path_validation_error;
    float path_validation_progress;

    uint32_t approach_phase;
    uint32_t approach_result;
    uint32_t approach_error;
    float approach_progress;

    /*
     * Simulator-only Preview playback status.
     *
     * Preview is not yet a project FSM state. These fields let the operator
     * HMI show the temporary simulator executor honestly without pretending
     * that a real Preview state has already been implemented.
     */
    bool preview_active;
    bool preview_complete;
    uint32_t preview_error;
    float preview_progress;

    float actual_tcp_m[3];
    float actual_joint_rad[HMI_NUM_JOINTS];

    uint32_t wkc;
    uint32_t expected_wkc;

    uint32_t recorded_count;
    float recorded_tcp_m[HMI_MAX_RECORDED_POINTS][3];

    uint32_t last_command_sequence;
    uint32_t guidance_error;
} HmiRobotStatus;

#endif /* HMI_TYPES_H */
