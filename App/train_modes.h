#ifndef TRAIN_MODES_H
#define TRAIN_MODES_H

#include "main.h"
#include "BMI088driver.h"

typedef enum {
    MODE_A_FIXATION = 0,
    MODE_B_SACCADE  = 1,
    MODE_C_PURSUIT  = 2,
    MODE_D_FOCUS    = 3,
    MODE_E_NEGLECT  = 4,
    MODE_COUNT      = 5
} TrainMode_t;

typedef enum {
    SYS_IDLE       = 0,
    SYS_CALIBRATE  = 1,
    SYS_TRAIN      = 2,
    SYS_FEEDBACK   = 3,
    SYS_PAUSE      = 4,
} SystemState_t;

typedef struct {
    TrainMode_t mode;
    uint32_t    start_tick;
    uint32_t    end_tick;
    uint16_t    total_trials;
    uint16_t    correct_trials;
    float       avg_reaction_ms;
    float       avg_head_stability;
    uint8_t     completed;
} TrainingRecord_t;

void App_Init(void);
void App_Run(bmi088_euler_data_t *euler, float temp);
SystemState_t App_GetState(void);
TrainMode_t App_GetMode(void);
TrainingRecord_t App_GetRecord(void);
void App_SetMode(TrainMode_t mode);

#endif
