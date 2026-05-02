#ifndef HEAD_TRACKER_H
#define HEAD_TRACKER_H

#include "BMI088driver.h"

#define POSTURE_CENTER 0
#define POSTURE_LEFT   1
#define POSTURE_RIGHT  2
#define POSTURE_DOWN   3
#define POSTURE_UP     4

typedef struct {
    float pitch;
    float roll;
    float yaw;
    float pitch_delta;
    float roll_delta;
    float yaw_delta;
    float head_stability;
    uint8_t posture;
    uint8_t is_tremor;
    uint8_t is_compensatory;
    uint8_t safety_alert;
} HeadAnalysis_t;

void HeadTracker_Init(void);
void HeadTracker_Update(bmi088_euler_data_t *euler, float dt);
HeadAnalysis_t* HeadTracker_GetResult(void);
uint8_t HeadTracker_CheckSafety(void);

#endif
