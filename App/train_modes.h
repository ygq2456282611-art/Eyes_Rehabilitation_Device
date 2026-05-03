/**
 * @file    train_modes.h
 * @brief   训练模式状态机 —— 公共接口与类型定义
 *          定义了五种临床训练模式、系统状态枚举、训练记录结构体
 *          以及应用层对外的初始化/运行/查询接口
 */

#ifndef TRAIN_MODES_H
#define TRAIN_MODES_H

#include "main.h"
#include "BMI088driver.h"

/**
 * @brief  训练模式枚举
 *         MODE_COUNT 同时用作数组大小和循环边界
 */
typedef enum {
    MODE_A_FIXATION = 0,  /* 注视稳定性训练：固定激光点，保持头部不动 */
    MODE_B_SACCADE  = 1,  /* 扫视训练：激光随机跳变，眼球快速扫视至目标 */
    MODE_C_PURSUIT  = 2,  /* 平稳追踪训练：激光连续移动，眼球平稳跟踪 */
    MODE_D_FOCUS    = 3,  /* 视觉聚焦训练：远近交替注视，训练睫状肌调节 */
    MODE_E_NEGLECT  = 4,  /* 空间忽略训练：患侧点亮，多感觉刺激 */
    MODE_COUNT      = 5   /* 模式总数，用于循环切换和越界检查 */
} TrainMode_t;

/**
 * @brief  系统状态枚举
 *         状态流转：IDLE → CALIBRATE → TRAIN → FEEDBACK → IDLE
 */
typedef enum {
    SYS_IDLE       = 0,  /* 空闲：模式选择菜单 */
    SYS_CALIBRATE  = 1,  /* 校准：陀螺仪零偏 + 初始姿态对齐 */
    SYS_TRAIN      = 2,  /* 训练：执行选中的训练模式 */
    SYS_FEEDBACK   = 3,  /* 反馈：播报训练成绩、参数汇总 */
    SYS_PAUSE      = 4,  /* 暂停：安全检测触发后自动暂停 */
} SystemState_t;

/**
 * @brief  训练记录结构体
 *         每次完整训练产生一条记录，暂存于内存
 */
typedef struct {
    TrainMode_t mode;                /* 本次训练的模式 */
    uint32_t    start_tick;          /* 训练开始时间戳 (HAL_GetTick) */
    uint32_t    end_tick;            /* 训练结束时间戳 */
    uint16_t    total_trials;        /* 总试验次数（扫视/忽略模式有效） */
    uint16_t    correct_trials;      /* 正确响应次数 */
    float       avg_reaction_ms;     /* 平均反应时间（ms） */
    float       avg_head_stability;  /* 平均头稳指标（注视模式有效） */
    uint8_t     completed;           /* 是否正常完成训练 */
} TrainingRecord_t;

/* ============ 应用层接口 ============ */

void App_Init(void);
void App_Run(bmi088_euler_data_t *euler, float temp);
SystemState_t App_GetState(void);
TrainMode_t App_GetMode(void);
TrainingRecord_t App_GetRecord(void);
void App_SetMode(TrainMode_t mode);

#endif
