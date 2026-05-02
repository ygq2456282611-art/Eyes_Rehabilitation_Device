#include "train_modes.h"
#include "servo.h"
#include "voice.h"
#include "laser.h"
#include "key.h"
#include "led.h"
#include "head_tracker.h"
#include <stdio.h>
#include <string.h>

static SystemState_t   sys_state   = SYS_IDLE;
static TrainMode_t     train_mode  = MODE_A_FIXATION;
static TrainingRecord_t record;
static uint32_t        trial_start_tick = 0;
static uint32_t        timebase         = 0;
static uint8_t         trial_result = 0;
static uint8_t         menu_index   = 0;

static const char *mode_names[MODE_COUNT] = {
    "A-注视训练", "B-扫视训练", "C-追踪训练", "D-聚焦训练", "E-忽略训练"
};

static uint8_t saccade_seq[20];
static uint8_t saccade_idx = 0;
static uint8_t saccade_count = 0;
static uint8_t current_target = 0;
static uint32_t saccade_light_on_tick = 0;

static uint8_t  pursuit_phase = 0;
static float    pursuit_time = 0;
static uint8_t  focus_phase = 0;
static uint32_t focus_phase_tick = 0;

static uint8_t  neglect_side = 0;
static uint32_t neglect_trial_tick = 0;
static uint8_t  neglect_responded = 0;
static uint8_t  neglect_trial_count = 0;

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

void App_Init(void)
{
    sys_state = SYS_IDLE;
    train_mode = MODE_A_FIXATION;
    memset(&record, 0, sizeof(TrainingRecord_t));
    menu_index = 0;
    timebase = HAL_GetTick();
}

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

SystemState_t App_GetState(void) { return sys_state; }
TrainMode_t   App_GetMode(void)  { return train_mode; }
TrainingRecord_t App_GetRecord(void) { return record; }

void App_SetMode(TrainMode_t mode)
{
    if (mode < MODE_COUNT)
        train_mode = mode;
}

static void App_State_Idle(void)
{
    LED_Blink(LED_FOCUS, 200);

    if (Key_GetEvent(KEY_MODE) == KEY_EVENT_SHORT)
    {
        train_mode = (TrainMode_t)((menu_index + 1) % MODE_COUNT);
        menu_index++;
        Voice_PlayText(mode_names[train_mode]);
    }

    if (Key_GetEvent(KEY_CONFIRM) == KEY_EVENT_SHORT)
    {
        LED_On(LED_FOCUS);
        Voice_PlayText(mode_names[train_mode]);
        HAL_Delay(800);
        App_Transition(SYS_CALIBRATE);
        return;
    }

    if (Key_GetEvent(KEY_BACK) == KEY_EVENT_SHORT)
    {
        train_mode = (TrainMode_t)((menu_index + MODE_COUNT - 1) % MODE_COUNT);
        menu_index = (menu_index + MODE_COUNT - 1) % MODE_COUNT;
        Voice_PlayText(mode_names[train_mode]);
    }
}

static void App_State_Calibrate(void)
{
    Voice_PlayText("准备校准，请保持头部静止");
    HAL_Delay(1500);
    BMI088_euler_init();
    Voice_PlayText("校准完成");
    HAL_Delay(1000);
    App_Transition(SYS_TRAIN);
}

static void App_State_Train(void)
{
    if (HeadTracker_CheckSafety())
    {
        sys_state = SYS_PAUSE;
        Voice_PlayText("检测到异常姿态，训练暂停");
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

static void App_State_Feedback(void)
{
    char buf[64];
    sprintf(buf, "训练结束。正确率百分之%d", (int)(record.correct_trials * 100 / (record.total_trials ? record.total_trials : 1)));
    Voice_PlayText(buf);
    HAL_Delay(2000);
    LED_Blink(LED_STATUS, 500);
    HAL_Delay(1500);
    LED_Off(LED_STATUS);
    App_Transition(SYS_IDLE);
}

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

static void Train_Fixation(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - timebase;

    Laser_On();
    LED_On(LED_FOCUS);

    HeadAnalysis_t *head = HeadTracker_GetResult();
    if (head->head_stability > 5.0f)
    {
        Voice_PlayText("请保持头部稳定");
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

static void Train_Saccade(void)
{
    uint32_t now = HAL_GetTick();

    if (saccade_count == 0)
    {
        saccade_count = 8;
        for (uint8_t i = 0; i < saccade_count; i++)
            saccade_seq[i] = i % 4;
        for (uint8_t i = saccade_count - 1; i > 0; i--)
        {
            uint8_t j = (uint8_t)(HAL_GetTick() % (i + 1));
            uint8_t tmp = saccade_seq[i];
            saccade_seq[i] = saccade_seq[j];
            saccade_seq[j] = tmp;
        }
        saccade_idx = 0;
    }

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

    if (Key_GetEvent(KEY_CONFIRM) == KEY_EVENT_SHORT && trial_result == 0)
    {
        trial_result = 1;
        record.correct_trials++;
        uint32_t reaction = now - trial_start_tick;
        record.avg_reaction_ms += (float)reaction;
        Laser_Off();
        Voice_PlayText("正确");
        saccade_light_on_tick = 0;
        saccade_idx++;
        HAL_Delay(500);
    }

    if (now - saccade_light_on_tick > 3000 && trial_result == 0)
    {
        Laser_Off();
        Voice_PlayText("超时");
        saccade_light_on_tick = 0;
        saccade_idx++;
        HAL_Delay(500);
    }
}

static void Train_Pursuit(void)
{
    uint32_t now = HAL_GetTick();
    float t = (float)(now - timebase) / 5000.0f;

    Laser_On();

    uint8_t x_angle, y_angle;
    switch (pursuit_phase % 3)
    {
        case 0:
            x_angle = (uint8_t)(90 + 60 * t);
            y_angle = 90;
            if (t >= 1.0f) { pursuit_phase++; timebase = now; }
            break;
        case 1:
            x_angle = (uint8_t)(150 * (1.0f - t) + 30 * t);
            y_angle = (uint8_t)(60 * (1.0f - t) + 120 * t);
            if (t >= 1.0f) { pursuit_phase++; timebase = now; }
            break;
        case 2:
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
        Voice_PlayText("请用眼睛跟踪，不要转头");
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

static void Train_Focus(void)
{
    uint32_t now = HAL_GetTick();

    if (focus_phase_tick == 0)
        focus_phase_tick = now;

    uint32_t elapsed = now - focus_phase_tick;

    switch (focus_phase & 1)
    {
        case 0:
            LED_On(LED_FOCUS);
            Laser_Off();
            if (elapsed > 5000) { focus_phase++; focus_phase_tick = now; }
            break;
        case 1:
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

static void Train_Neglect(void)
{
    uint32_t now = HAL_GetTick();

    if (neglect_trial_count == 0)
    {
        record.start_tick = now;
    }

    if (neglect_trial_tick == 0)
    {
        neglect_side = neglect_trial_count & 1;
        Voice_PlayText("准备");

        if (neglect_side == 0)
            Servo_SetAngle(SERVO_AXIS_X, 30);
        else
            Servo_SetAngle(SERVO_AXIS_X, 150);

        HAL_Delay(300);
        Laser_On();
        Voice_PlayText("请寻找光点");
        neglect_trial_tick = now;
        neglect_responded = 0;
        record.total_trials++;
        neglect_trial_count++;
    }

    uint32_t reaction = now - neglect_trial_tick;

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

    if (now - neglect_trial_tick > 5000 && neglect_responded == 0)
    {
        Voice_PlayText("超时，请多注意该侧空间");
        Laser_Off();
        Servo_SetAngle(SERVO_AXIS_X, 90);
        neglect_trial_tick = 0;
        HAL_Delay(1000);
    }

    if (neglect_trial_count >= 10)
    {
        record.end_tick = now;
        record.completed = 1;
        App_Transition(SYS_FEEDBACK);
    }
}
