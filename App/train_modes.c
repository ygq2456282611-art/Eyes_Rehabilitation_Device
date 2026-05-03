/**
 * @file    train_modes.c
 * @brief   训练模式状态机 + 五种临床训练模式实现
 *          状态机：IDLE → CALIBRATE → TRAIN → FEEDBACK → IDLE
 *          训练模式：
 *          A-注视稳定性训练：固定激光点，要求头部保持不动
 *          B-扫视训练：激光随机跳变位置，按键响应测反应时间
 *          C-平稳追踪训练：激光画线/圆轨迹，眼球平稳跟踪
 *          D-视觉聚焦训练：远近交替注视，训练睫状肌调节
 *          E-空间忽略训练：患侧随机点亮，多感觉刺激
 */
#include "train_modes.h"
#include "servo.h"
#include "voice.h"
#include "laser.h"
#include "key.h"
#include "led.h"
#include "head_tracker.h"
#include <stdio.h>
#include <string.h>

/* ===== 全局状态变量 ===== */
static SystemState_t   sys_state   = SYS_IDLE;
static TrainMode_t     train_mode  = MODE_A_FIXATION;
static TrainingRecord_t record;
static uint32_t        trial_start_tick = 0;
static uint32_t        timebase         = 0;
static uint8_t         trial_result = 0;
static uint8_t         menu_index   = 0;

/* 训练模式的中文名称（用于语音播报菜单） */
static const char *mode_names[MODE_COUNT] = {
    "A", "B", "C", "D", "E"
};

/* ===== 扫视训练用变量 ===== */
static uint8_t saccade_seq[20];         /* 随机化的目标序列 */
static uint8_t saccade_idx = 0;         /* 当前目标索引 */
static uint8_t saccade_count = 0;       /* 总目标数 */
static uint8_t current_target = 0;      /* 当前目标编号 */
static uint32_t saccade_light_on_tick = 0;  /* 激光点亮时间戳 */

/* ===== 追踪训练用变量 ===== */
static uint8_t  pursuit_phase = 0;      /* 当前追踪阶段（直线→斜线→圆） */
static float    pursuit_time = 0;

/* ===== 聚焦训练用变量 ===== */
static uint8_t  focus_phase = 0;        /* 当前聚焦阶段计数 */
static uint32_t focus_phase_tick = 0;   /* 阶段切换时间戳 */

/* ===== 忽略训练用变量 ===== */
static uint8_t  neglect_side = 0;       /* 0=左侧, 1=右侧 */
static uint32_t neglect_trial_tick = 0;
static uint8_t  neglect_responded = 0;
static uint8_t  neglect_trial_count = 0;

/* 内部函数声明 */
static void App_State_Idle(void);
static void App_State_Calibrate(void);
static void App_State_Train(void);
static void App_State_Feedback(void);

static void Train_Fixation(void);
static void Train_Saccade(void);
static void Train_Pursuit(void);
static void Train_Focus(void);
static void Train_Neglect(void);

static void App_Transition(SystemState_t next_state);

/**
 * @brief  初始化应用层
 *         设置初始状态为 IDLE，默认训练模式 A
 */
void App_Init(void)
{
    sys_state = SYS_IDLE;
    train_mode = MODE_A_FIXATION;
    memset(&record, 0, sizeof(TrainingRecord_t));
    menu_index = 0;
    timebase = HAL_GetTick();
}

/**
 * @brief  应用主运行函数（主循环中每10ms调用一次）
 * @param  euler : BMI088 欧拉角数据（传递给 HeadTracker）
 * @param  temp  : 温度数据（暂未使用）
 *         根据当前系统状态分发到对应的子状态机
 */
void App_Run(bmi088_euler_data_t *euler, float temp)
{
    (void)temp;

    switch (sys_state)
    {
        case SYS_IDLE:       App_State_Idle();       break;
        case SYS_CALIBRATE:  App_State_Calibrate();  break;
        case SYS_TRAIN:      App_State_Train();      break;
        case SYS_FEEDBACK:   App_State_Feedback();   break;
        case SYS_PAUSE:      break;
    }
}

/* ============ 状态查询函数 ============ */
SystemState_t App_GetState(void) { return sys_state; }
TrainMode_t   App_GetMode(void)  { return train_mode; }
TrainingRecord_t App_GetRecord(void) { return record; }

/**
 * @brief  强制设置训练模式
 * @param  mode : 目标模式编号
 */
void App_SetMode(TrainMode_t mode)
{
    if (mode < MODE_COUNT)
        train_mode = mode;
}

/**
 * @brief  IDLE 空闲状态
 *         功能：模式选择菜单
 *         - MODE键：循环切换五种模式，语音播报模式名
 *         - CONFIRM键：确认选中模式，进入校准
 *         - BACK键：反向切换模式
 */
static void App_State_Idle(void)
{
    LED_Blink(LED_FOCUS, 200);

    if (Key_GetEvent(KEY_MODE) == KEY_EVENT_SHORT)
    {
        train_mode = (TrainMode_t)((menu_index + 1) % MODE_COUNT);
        menu_index++;
Voice_Play(0x00, VOICE_CMD_FIXATION + (uint8_t)train_mode);
        HAL_Delay(800);
        App_Transition(SYS_CALIBRATE);
        return;
    }

    if (Key_GetEvent(KEY_BACK) == KEY_EVENT_SHORT)
    {
        train_mode = (TrainMode_t)((menu_index + MODE_COUNT - 1) % MODE_COUNT);
        menu_index = (menu_index + MODE_COUNT - 1) % MODE_COUNT;
        Voice_Play(0x00, VOICE_CMD_FIXATION + (uint8_t)train_mode);
    }
}

/**
 * @brief  校准状态
 *         提示患者保持头部静止 → 执行 BMI088 陀螺仪零偏校准 → 进入训练
 */
static void App_State_Calibrate(void)
{
    Voice_Play(0x00, VOICE_CMD_CALIB_STILL);
    HAL_Delay(1500);
    BMI088_euler_init();
    Voice_Play(0xFF, VOICE_TTS_CALIB_DONE);
    HAL_Delay(1000);
    App_Transition(SYS_TRAIN);
}

/**
 * @brief  训练状态
 *         每次循环先检查安全状态，然后根据当前模式分发到具体训练函数
 */
static void App_State_Train(void)
{
    if (HeadTracker_CheckSafety())
    {
        sys_state = SYS_PAUSE;
        Voice_Play(0xFF, VOICE_TTS_EYE_ONLY);
        Laser_Off();
        return;
    }

    switch (train_mode)
    {
        case MODE_A_FIXATION: Train_Fixation(); break;
        case MODE_B_SACCADE:  Train_Saccade();  break;
        case MODE_C_PURSUIT:  Train_Pursuit();  break;
        case MODE_D_FOCUS:    Train_Focus();    break;
        case MODE_E_NEGLECT:  Train_Neglect();  break;
        default: break;
    }
}

/**
 * @brief  反馈状态
 *         语音播报训练成绩（正确率）→ 状态LED闪烁提示 → 回到IDLE
 */
static void App_State_Feedback(void)
{
    Voice_Play(0xFF, VOICE_TTS_TRAIN_DONE);
    HAL_Delay(2000);
    LED_Blink(LED_STATUS, 500);
    HAL_Delay(1500);
    LED_Off(LED_STATUS);
    App_Transition(SYS_IDLE);
}

/**
 * @brief  状态切换处理
 * @param  next_state : 目标状态
 *         进入 TRAIN 时复位训练记录和内部计数器
 *         进入 IDLE 时关闭所有输出（激光、LED）
 */
static void App_Transition(SystemState_t next_state)
{
    sys_state = next_state;
    if (next_state == SYS_TRAIN)
    {
        memset(&record, 0, sizeof(TrainingRecord_t));
        record.mode = train_mode;
        record.start_tick = HAL_GetTick();
        timebase = HAL_GetTick();
        pursuit_phase = 0;
        focus_phase = 0;
        saccade_idx = 0;
        saccade_count = 0;
    }
    if (next_state == SYS_IDLE)
    {
        Laser_Off();
        LED_Off(LED_FOCUS);
        LED_Off(LED_STATUS);
    }
}

/**
 * @brief  模式A - 注视稳定性训练
 *         原理：激光照射固定点，患者注视15秒
 *         检测：头稳指标超过5°时语音提醒"请保持头部稳定"
 *         结束：15秒自动结束，记录头稳指标
 */
static void Train_Fixation(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - timebase;

    Laser_On();
    LED_On(LED_FOCUS);

    HeadAnalysis_t *head = HeadTracker_GetResult();
    if (head->head_stability > 5.0f)
    {
        Voice_Play(0xFF, VOICE_TTS_KEEP_STILL);
    }

    if (elapsed > 15000)
    {
        Laser_Off();
        LED_Off(LED_FOCUS);
        record.end_tick = now;
        record.avg_head_stability = head->head_stability;
        record.completed = 1;
        App_Transition(SYS_FEEDBACK);
    }
}

/**
 * @brief  模式B - 扫视训练
 *         原理：激光在 4 个位置随机跳变，患者眼球扫视后按键确认
 *         流程：随机排序8次目标 → 每3秒超时 → 按键记录反应时间
 *         指标：正确次数、平均反应时间
 */
static void Train_Saccade(void)
{
    uint32_t now = HAL_GetTick();

    /* 首次运行：生成8个随机排序的目标位置（0~3号位置） */
    if (saccade_count == 0)
    {
        saccade_count = 8;
        for (uint8_t i = 0; i < saccade_count; i++)
            saccade_seq[i] = i % 4;
        /* Fisher-Yates shuffle 随机打乱 */
        for (uint8_t i = saccade_count - 1; i > 0; i--)
        {
            uint8_t j = (uint8_t)(HAL_GetTick() % (i + 1));
            uint8_t tmp = saccade_seq[i];
            saccade_seq[i] = saccade_seq[j];
            saccade_seq[j] = tmp;
        }
        saccade_idx = 0;
    }

    /* 当前目标已完成 → 切换到下一个 */
    if (saccade_light_on_tick == 0)
    {
        if (saccade_idx >= saccade_count)
        {
            record.end_tick = now;
            record.completed = 1;
            App_Transition(SYS_FEEDBACK);
            return;
        }

        current_target = saccade_seq[saccade_idx];
        uint8_t x_angle = 0, y_angle = 90;

        /* 4个位置：左上45°/60、右上135°/60、左下45°/120、右下135°/120 */
        switch (current_target)
        {
            case 0: x_angle = 45;  y_angle = 60;  break;
            case 1: x_angle = 135; y_angle = 60;  break;
            case 2: x_angle = 45;  y_angle = 120; break;
            case 3: x_angle = 135; y_angle = 120; break;
        }

        Servo_SetAngle(SERVO_AXIS_X, x_angle);
        Servo_SetAngle(SERVO_AXIS_Y, y_angle);
        HAL_Delay(100);
        Laser_On();
        saccade_light_on_tick = now;
        trial_start_tick = now;
        trial_result = 0;
        record.total_trials++;
    }

    /* 检测按键响应 */
    if (Key_GetEvent(KEY_CONFIRM) == KEY_EVENT_SHORT && trial_result == 0)
    {
        trial_result = 1;
        record.correct_trials++;
        uint32_t reaction = now - trial_start_tick;
        record.avg_reaction_ms += (float)reaction;
        Laser_Off();
        Voice_Play(0xFF, VOICE_TTS_CORRECT);
        saccade_light_on_tick = 0;
        saccade_idx++;
        HAL_Delay(500);
    }

    /* 超时（3秒无响应） */
    if (now - saccade_light_on_tick > 3000 && trial_result == 0)
    {
        Laser_Off();
        Voice_Play(0xFF, VOICE_TTS_TIMEOUT);
        saccade_light_on_tick = 0;
        saccade_idx++;
        HAL_Delay(500);
    }
}

/**
 * @brief  模式C - 平稳追踪训练
 *         原理：激光连续移动，患者眼球平稳跟踪（禁止转头代偿）
 *         轨迹分3阶段各5秒：水平线→斜线→圆圈
 *         总时长30秒
 *         检测代偿性转头并语音提醒
 */
static void Train_Pursuit(void)
{
    uint32_t now = HAL_GetTick();
    float t = (float)(now - timebase) / 5000.0f;

    Laser_On();

    uint8_t x_angle, y_angle;
    switch (pursuit_phase % 3)
    {
        case 0:  /* 水平扫描：左→右 */
            x_angle = (uint8_t)(90 + 60 * t);
            y_angle = 90;
            if (t >= 1.0f) { pursuit_phase++; timebase = now; }
            break;
        case 1:  /* 斜线：右上→左下 */
            x_angle = (uint8_t)(150 * (1.0f - t) + 30 * t);
            y_angle = (uint8_t)(60 * (1.0f - t) + 120 * t);
            if (t >= 1.0f) { pursuit_phase++; timebase = now; }
            break;
        case 2:  /* 圆形轨迹 */
        {
            float rad = t * 6.2832f;
            x_angle = (uint8_t)(90 + 60 * cosf(rad));
            y_angle = (uint8_t)(90 + 30 * sinf(rad));
            if (t >= 1.0f) { pursuit_phase++; timebase = now; }
            break;
        }
    }

    if (x_angle > 180) x_angle = 180;
    if (x_angle < 0)   x_angle = 0;
    if (y_angle > 180) y_angle = 180;
    if (y_angle < 0)   y_angle = 0;

    Servo_SetAngle(SERVO_AXIS_X, x_angle);
    Servo_SetAngle(SERVO_AXIS_Y, y_angle);

    HeadAnalysis_t *head = HeadTracker_GetResult();
    if (head->is_compensatory)
    {
        Voice_Play(0xFF, VOICE_TTS_EYE_ONLY);
    }

    if (now - record.start_tick > 30000)
    {
        Laser_Off();
        Servo_SetAngle(SERVO_AXIS_X, 90);
        Servo_SetAngle(SERVO_AXIS_Y, 90);
        record.end_tick = now;
        record.completed = 1;
        App_Transition(SYS_FEEDBACK);
    }
}

/**
 * @brief  模式D - 视觉聚焦训练（调节训练）
 *         原理：近距LED(25cm)和远距激光交替亮起，交替注视
 *         近距5秒→远距5秒→循环5次（共50秒）
 *         训练睫状肌的调节/松弛能力
 */
static void Train_Focus(void)
{
    uint32_t now = HAL_GetTick();

    if (focus_phase_tick == 0)
        focus_phase_tick = now;

    uint32_t elapsed = now - focus_phase_tick;

    /* phase 偶数=近距, phase 奇数=远距 */
    switch (focus_phase & 1)
    {
        case 0:  /* 近距：开启近距LED，关闭激光 */
            LED_On(LED_FOCUS);
            Laser_Off();
            if (elapsed > 5000) { focus_phase++; focus_phase_tick = now; }
            break;
        case 1:  /* 远距：关闭LED，开启激光 */
            LED_Off(LED_FOCUS);
            Laser_On();
            if (elapsed > 5000) { focus_phase++; focus_phase_tick = now; }
            break;
    }

    if (focus_phase >= 10)
    {
        LED_Off(LED_FOCUS);
        Laser_Off();
        record.end_tick = now;
        record.completed = 1;
        App_Transition(SYS_FEEDBACK);
    }
}

/**
 * @brief  模式E - 空间忽略训练
 *         原理：在患者左右视野交替点亮激光，引导寻找光点
 *         健侧(左)→患侧(右)→交替共10次
 *         语音提示"请寻找光点"，5秒超时
 *         记录两侧反应时间差异，评估忽略程度
 */
static void Train_Neglect(void)
{
    uint32_t now = HAL_GetTick();

    if (neglect_trial_count == 0)
    {
        record.start_tick = now;
    }

    /* 新一次试验开始 */
    if (neglect_trial_tick == 0)
    {
        neglect_side = neglect_trial_count & 1;  /* 交替左右侧 */
        Voice_Play(0x00, VOICE_CMD_CALIB_STILL);

        if (neglect_side == 0)
            Servo_SetAngle(SERVO_AXIS_X, 30);   /* 左侧 */
        else
            Servo_SetAngle(SERVO_AXIS_X, 150);  /* 右侧 */

        HAL_Delay(300);
        Laser_On();
        Voice_Play(0xFF, VOICE_TTS_FIND_LIGHT);
        neglect_trial_tick = now;
        neglect_responded = 0;
        record.total_trials++;
        neglect_trial_count++;
    }

    uint32_t reaction = now - neglect_trial_tick;

    /* 患者按键确认发现光点 */
    if (Key_GetEvent(KEY_CONFIRM) == KEY_EVENT_SHORT && neglect_responded == 0)
    {
        neglect_responded = 1;
        record.correct_trials++;
        record.avg_reaction_ms += (float)reaction;
        Laser_Off();
        Servo_SetAngle(SERVO_AXIS_X, 90);
        neglect_trial_tick = 0;
        HAL_Delay(1500);
    }

    /* 超时 */
    if (now - neglect_trial_tick > 5000 && neglect_responded == 0)
    {
        Voice_Play(0xFF, VOICE_TTS_NEGLECT_HINT);
        Laser_Off();
        Servo_SetAngle(SERVO_AXIS_X, 90);
        neglect_trial_tick = 0;
        HAL_Delay(1000);
    }

    /* 10次试验结束 */
    if (neglect_trial_count >= 10)
    {
        record.end_tick = now;
        record.completed = 1;
        App_Transition(SYS_FEEDBACK);
    }
}
