/**
 * @file    train_modes.h
 * @brief   训练模式状态机 —— 公共接口与类型定义
 *          语音驱动的五种临床训练模式 + 实时姿态监测 + 患者主动反馈
 *
 *          状态流转：
 *          SYS_IDLE_VOICE → (听到命令词) → CALIBRATE → TRAIN → FEEDBACK → SYS_IDLE_VOICE
 *                         SYS_TRAIN ⇄ SYS_PAUSE（报警自动暂停/恢复）
 */
#ifndef TRAIN_MODES_H
#define TRAIN_MODES_H

#include "main.h"
#include "BMI088driver.h"

typedef enum {
    MODE_A_FIXATION = 0,  /* 注视稳定性训练 */
    MODE_B_SACCADE  = 1,  /* 扫视训练 */
    MODE_C_PURSUIT  = 2,  /* 平稳追踪训练 */
    MODE_D_FOCUS    = 3,  /* 视觉聚焦训练 */
    MODE_E_NEGLECT  = 4,  /* 空间忽略训练 */
    MODE_COUNT      = 5
} TrainMode_t;

typedef enum {
    SYS_IDLE_VOICE = 0,  /* 空闲：轮询 Voice_GetCommand() 等待语音命令词 */
    SYS_CALIBRATE  = 1,  /* 校准：陀螺仪零偏 */
    SYS_TRAIN      = 2,  /* 训练执行中 */
    SYS_FEEDBACK   = 3,  /* 反馈：播报结果 → 回到 IDLE */
    SYS_PAUSE      = 4,  /* 暂停：姿态异常，自动恢复 */
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

#endif
