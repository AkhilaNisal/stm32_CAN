#ifndef INC_FS_AI_PROTOCOL_H_
#define INC_FS_AI_PROTOCOL_H_

#include "CANSPI.h"
#include <stdint.h>

/* FS-AI CAN IDs */
#define ID_AI2VCU_STATUS    0x510
#define ID_AI2VCU_DRIVE_F   0x511
#define ID_AI2VCU_DRIVE_R   0x512
#define ID_AI2VCU_STEER     0x513
#define ID_AI2VCU_BRAKE     0x514

#define ID_VCU2AI_STATUS    0x520
#define ID_VCU2AI_DRIVE_F   0x521
#define ID_VCU2AI_DRIVE_R   0x522
#define ID_VCU2AI_STEER     0x523
#define ID_VCU2AI_BRAKE     0x524

/* AS States according to ADS-DV Spec */
typedef enum {
    AS_OFF = 1,
    AS_READY = 2,
    AS_DRIVING = 3,
    AS_EMERGENCY_BRAKE = 4,
    AS_FINISHED = 5
} fs_ai_as_state_t;

/* AMI States */
typedef enum {
    AMI_NOT_SELECTED = 0,
    AMI_ACCELERATION = 1,
    AMI_SKIDPAD = 2,
    AMI_AUTOCROSS = 3,
    AMI_TRACK_DRIVE = 4,
    AMI_STATIC_INSPECTION_A = 5,
    AMI_STATIC_INSPECTION_B = 6,
    AMI_AUTONOMOUS_DEMO = 7
} fs_ai_ami_state_t;

/* VCU Status Structure (Bit-packed for 0x520) */
typedef struct {
    uint8_t handshake : 1;
    uint8_t shutdown_request : 1;
    uint8_t as_switch_status : 1;
    uint8_t ts_switch_status : 1;
    uint8_t go_signal : 1;
    uint8_t steering_status : 1;
    uint8_t as_state : 3;
    uint8_t ami_state : 3;
    uint8_t fault_status : 1;
    uint8_t warning_status : 1;
} VCU_Status_t;

/* Global access to received AI demands */
extern volatile int16_t AI_Steer_Request;
extern volatile uint8_t AI_Brake_Request_F;

void FS_AI_TransmitStatus(VCU_Status_t *status);
void FS_AI_ProcessIncoming(uCAN_MSG *rxMsg);

#endif /* INC_FS_AI_PROTOCOL_H_ */
