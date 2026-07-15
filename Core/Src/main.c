/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMI088driver.h"
#include "servo.h"
#include "voice.h"
#include "laser.h"
#include "key.h"
#include "led.h"
#include "head_tracker.h"
#include "train_modes.h"
#include "buzzer.h"
#include "ws2812.h"
#include "wireless_bridge.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define VOFA_SEND_PERIOD_MS       100U
#define VOFA_UART_BAUDRATE        115200U
#define VOFA_TX_MARGIN_MS         10U

bmi088_euler_data_t euler_angle;
float temp;
static float vofa_gyro[3];
static float vofa_accel[3];
static float vofa_bias[3];
static float vofa_roll_deg;
static float vofa_pitch_deg;
static float vofa_yaw_deg;
static float vofa_dt;
static uint32_t vofa_timestamp_us;
static uint8_t imu_calibrated;
static uint8_t imu_is_static;
static uint8_t wireless_online;
static uint32_t wireless_rx_valid;
static uint32_t wireless_rx_invalid;
static uint32_t wireless_tx_count;
static uint32_t wireless_last_sequence;
static uint32_t wireless_status_last_ms;
static char vofa_buf[384];
static char vofa_dma_buf[384];
static uint32_t vofa_last_send_ms;
static uint32_t vofa_tx_fail_count;
static uint32_t vofa_tx_drop_count;
static volatile uint8_t vofa_tx_busy;
static uint32_t ws2812_alert_tick = 0;
static uint8_t  ws2812_alert_on = 0;
static uint8_t  prev_alert_active = 0;

#pragma pack(push, 1)
typedef struct {
    uint32_t session_id;
    uint8_t flow_mode, mode_count, completed_modes, completed, overall_score;
    uint8_t mode_order[5];
    uint32_t total_duration_ms;
    uint8_t config_version;
    uint8_t reserved[9];
} ReportSummaryWire_t;

typedef struct {
    uint32_t session_id;
    uint8_t mode, completed, score, pause_count;
    uint16_t total_trials, correct_trials, timeout_trials;
    uint16_t duration_s, avg_reaction_ms, min_reaction_ms, max_reaction_ms;
    uint16_t avg_head_offset_tenths, max_head_offset_tenths, head_variation_tenths;
} ReportModeBasicWire_t;

typedef struct {
    uint32_t session_id;
    uint8_t mode, valid;
    uint8_t reserved0[2];
    uint32_t head_over_limit_ms;
    uint16_t compensation_permille;
    uint16_t left_total, left_correct, right_total, right_correct;
    uint8_t trial_count, affected_side;
    int16_t asymmetry_permille;
    uint8_t reserved1[2];
} ReportModeDetailWire_t;

typedef struct {
    uint32_t session_id;
    uint8_t mode, index, target, side, correct, timed_out;
    uint8_t reserved0[2];
    uint32_t reaction_ms;
    uint8_t reserved1[12];
} ReportTrialWire_t;
#pragma pack(pop)

typedef char ReportSummarySizeCheck[(sizeof(ReportSummaryWire_t) == 28U) ? 1 : -1];
typedef char ReportModeBasicSizeCheck[(sizeof(ReportModeBasicWire_t) == 28U) ? 1 : -1];
typedef char ReportModeDetailSizeCheck[(sizeof(ReportModeDetailWire_t) == 28U) ? 1 : -1];
typedef char ReportTrialSizeCheck[(sizeof(ReportTrialWire_t) == 28U) ? 1 : -1];

static TrainingConfig_t Main_ConvertWirelessConfig(const WirelessTrainingConfig_t *wire);
static WirelessTrainingConfig_t Main_ConvertTrainingConfig(const TrainingConfig_t *config);
static void Main_SendReportSection(const WirelessReportQuery_t *query);
static uint16_t Main_ClampU16(uint32_t value);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_UART7_Init();
  MX_USART10_UART_Init();
  /* USER CODE BEGIN 2 */
    
    while (BMI088_init() != BMI088_NO_ERROR)
    {
    }
    BMI088_AsyncStart();
    BMI088_euler_init();
    HAL_Delay(2000);

    /* 使能 DM-02 板载 5V 可控电源输出（舵机/PWM/串口/CAN 供电） */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef pwr = {0};
    pwr.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    pwr.Mode = GPIO_MODE_OUTPUT_PP;
    pwr.Pull = GPIO_NOPULL;
    pwr.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &pwr);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);

    Servo_Init();
    /* 成品上电保持双轴 90 度安全停放；需要检修时再手动调用 Servo_Test()。 */

    Laser_Init();
    Laser_Test();// 点亮激光并闪烁 3 次，验证 PA0 输出正常

    Key_Init();
    LED_Init();
    HeadTracker_Init();
    App_Init();
    Buzzer_Init();
    WS2812_Init();
    Voice_Init();
    WirelessBridge_Init();
    
    Buzzer_Alert(3, 200, 100);
    Laser_Blink(200, 3);
    HAL_Delay(4000);
    Voice_Play(0xFF, VOICE_TTS_INIT_OK); // 初始化完成播报
    HAL_Delay(4000);
    Voice_Play(0xFF, VOICE_TTS_WELCOME); // 欢迎
    HAL_Delay(2000);

    /* ==== PA2 按键测试（成功后删除）==== */
    /* ================================== */
    //Calibrate_ServoRange();  /* 标定模式已改为语音命令触发 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        uint32_t now_ms = HAL_GetTick();
        WirelessControlRequest_t control_request;
        WirelessConfigRequest_t config_request;
        WirelessConfigQuery_t config_query;
        WirelessReportQuery_t report_query;
        WirelessServoPreviewRequest_t servo_preview_request;

        if (WirelessBridge_TakeConfig(&config_request) != 0U)
        {
            AppControlResult_t config_result;
            if (config_request.precheck_result == APP_CONTROL_RESULT_ACCEPTED)
            {
                TrainingConfig_t config = Main_ConvertWirelessConfig(&config_request.config);
                config_result = App_SetTrainingConfig(&config);
            }
            else
            {
                config_result = (AppControlResult_t)config_request.precheck_result;
            }
            WirelessBridge_SendConfigAck(&config_request, (uint8_t)config_result,
                                         (uint8_t)App_GetState());
        }

        if (WirelessBridge_TakeConfigQuery(&config_query) != 0U)
        {
            TrainingConfig_t active_config = App_GetTrainingConfig();
            WirelessTrainingConfig_t wireless_config = Main_ConvertTrainingConfig(&active_config);
            WirelessBridge_SendConfigData(&config_query, &wireless_config);
        }

        if (WirelessBridge_TakeServoPreview(&servo_preview_request) != 0U)
        {
            AppControlResult_t preview_result;
            if (servo_preview_request.precheck_result == APP_CONTROL_RESULT_ACCEPTED)
            {
                preview_result = App_PreviewServo(
                    servo_preview_request.axis, servo_preview_request.angle,
                    (AppServoPreviewAction_t)servo_preview_request.action);
            }
            else
            {
                preview_result = (AppControlResult_t)servo_preview_request.precheck_result;
            }
            WirelessBridge_SendServoPreviewAck(&servo_preview_request,
                                               (uint8_t)preview_result,
                                               (uint8_t)App_GetState());
        }

        /* USART10中断只接收并入队，所有训练状态切换统一在主循环执行。 */
        if (WirelessBridge_TakeControl(&control_request) != 0U)
        {
            AppControlResult_t control_result;

            if (control_request.precheck_result == APP_CONTROL_RESULT_ACCEPTED)
            {
                if (control_request.command == APP_CONTROL_START)
                {
                    TrainingPlan_t plan;
                    memset(&plan, 0, sizeof(plan));
                    plan.flow_mode = control_request.flow_mode;
                    plan.mode_count = control_request.mode_count;
                    memcpy(plan.mode_order, control_request.mode_order, sizeof(plan.mode_order));
                    if (control_request.config_version != APP_CONFIG_VERSION)
                        control_result = APP_CONTROL_RESULT_VERSION_MISMATCH;
                    else
                        control_result = App_StartSession(&plan);
                }
                else
                {
                    control_result = App_ExecuteControl((AppControlCommand_t)control_request.command, 0U);
                }
            }
            else
            {
                control_result = (AppControlResult_t)control_request.precheck_result;
            }
            WirelessBridge_SendControlAck(&control_request, (uint8_t)control_result,
                                          (uint8_t)App_GetState(), (uint8_t)App_GetMode());
        }
        WirelessBridge_Update(now_ms);
        BMI088_Task();

        BMI088_read_euler(&euler_angle, &temp);
        HeadTracker_Update(&euler_angle, 0.01f);

        HeadAnalysis_t *head = HeadTracker_GetResult();
        (void)head;

        uint8_t alert_active = 0;
        uint8_t suppress_posture_alert =
            (App_GetState() == SYS_TRAIN && App_GetMode() == MODE_A_FIXATION);

        if (!suppress_posture_alert)
        {
            TrainingConfig_t active_config = App_GetTrainingConfig();
            /* Vertical mount semantics: roll=nod, pitch=head turn, yaw=lateral tilt. */
            if (fabsf(euler_angle.roll)  > active_config.safety_nod_deg) alert_active = 1;
            if (fabsf(euler_angle.pitch) > active_config.safety_turn_deg) alert_active = 1;
            if (fabsf(euler_angle.yaw)   > active_config.safety_tilt_deg) alert_active = 1;
        }

        /* 蜂鸣器：刚进入报警状态时响两声 */
        if (alert_active && !prev_alert_active)
        {
            Buzzer_Alert(2, 150, 100);
        }
        prev_alert_active = alert_active;

        /* WS2812：报警态红色闪烁，正常态绿色呼吸 */
        if (alert_active)
        {
            if (HAL_GetTick() - ws2812_alert_tick >= 1000)
            {
                ws2812_alert_tick = HAL_GetTick();
                ws2812_alert_on = !ws2812_alert_on;
            }
            WS2812_Set(ws2812_alert_on ? 255 : 0, 0, 0);
        }
        else
        {
            WS2812_BreathingGreen();
        }

        Key_Scan();
        App_Run(&euler_angle, temp);

        if (WirelessBridge_TakeReportQuery(&report_query) != 0U)
        {
            Main_SendReportSection(&report_query);
            /* Start a report reply immediately instead of waiting for the next 10 ms cycle. */
            WirelessBridge_Update(now_ms);
        }

        if ((now_ms - wireless_status_last_ms) >= 200U)
        {
            WirelessTrainingStatus_t status;
            AppLiveStatus_t live_status;

            wireless_status_last_ms = now_ms;
            App_GetLiveStatus(&live_status);
            memset(&status, 0, sizeof(status));
            status.state = live_status.state;
            status.mode = live_status.mode;
            status.flow_mode = live_status.flow_mode;
            status.flags = live_status.flags;
            status.mode_index = live_status.mode_index;
            status.mode_count = live_status.mode_count;
            status.total_trials = live_status.total_trials;
            status.correct_trials = live_status.correct_trials;
            status.timeout_trials = live_status.timeout_trials;
            status.elapsed_ms = live_status.elapsed_ms;
            status.avg_reaction_ms = live_status.avg_reaction_ms;
            status.head_offset_tenths = live_status.head_offset_tenths;
            status.head_variation_tenths = live_status.head_variation_tenths;
            status.session_id = live_status.session_id;
            status.calibration_phase = live_status.calibration_phase;
            status.calibration_points = live_status.calibration_points;
            WirelessBridge_SendTrainingStatus(&status);
        }

        if ((HAL_GetTick() - vofa_last_send_ms) >= VOFA_SEND_PERIOD_MS)
        {
            int len;

            vofa_last_send_ms = HAL_GetTick();
            BMI088_GetLatestFloat(vofa_gyro, vofa_accel, &temp, &vofa_timestamp_us);
            BMI088_GetEuler(&vofa_roll_deg, &vofa_pitch_deg, &vofa_yaw_deg, &vofa_dt, &vofa_timestamp_us);
            BMI088_GetGyroBias(vofa_bias);
            imu_calibrated = BMI088_IsCalibrated();
            imu_is_static = BMI088_IsStatic();
            wireless_online = WirelessBridge_IsOnline();
            wireless_rx_valid = WirelessBridge_GetValidRxCount();
            wireless_rx_invalid = WirelessBridge_GetInvalidRxCount();
            wireless_tx_count = WirelessBridge_GetTxCount();
            wireless_last_sequence = WirelessBridge_GetLastSequence();

            len = snprintf(vofa_buf, sizeof(vofa_buf),
                           "imu:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%u,%u,%.2f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
                           euler_angle.roll, euler_angle.pitch, euler_angle.yaw,
                           vofa_roll_deg, vofa_pitch_deg, vofa_yaw_deg,
                           vofa_gyro[0], vofa_gyro[1], vofa_gyro[2],
                           vofa_accel[0], vofa_accel[1], vofa_accel[2],
                           vofa_bias[0], vofa_bias[1], vofa_bias[2],
                           (unsigned int)imu_is_static,
                           (unsigned int)imu_calibrated,
                           temp,
                           (unsigned int)wireless_online,
                           (unsigned int)wireless_rx_valid,
                           (unsigned int)wireless_rx_invalid,
                           (unsigned int)wireless_tx_count,
                           (unsigned int)wireless_last_sequence,
                           (unsigned int)App_GetState(),
                           (unsigned int)App_GetMode(),
                           (unsigned int)App_GetLastVoiceCmd(),
                           (unsigned int)App_GetLastVoiceCmdTick(),
                           (unsigned int)Voice_GetLastTxType(),
                           (unsigned int)Voice_GetLastTxId(),
                           (unsigned int)Voice_GetLastTxTick(),
                           (unsigned int)App_GetLastEvent(),
                           (unsigned int)vofa_tx_fail_count,
                           (unsigned int)vofa_tx_drop_count);

            if ((len > 0) && (len < (int)sizeof(vofa_buf)))
            {
                if (vofa_tx_busy != 0U)
                {
                    vofa_tx_drop_count++;
                }
                else
                {
                    memcpy(vofa_dma_buf, vofa_buf, (size_t)len);
                    vofa_tx_busy = 1U;
                    if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)vofa_dma_buf, (uint16_t)len) != HAL_OK)
                    {
                        vofa_tx_busy = 0U;
                        vofa_tx_fail_count++;
                    }
                }
            }
        }

        BMI088_ClearNewSampleFlag();
        HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static uint16_t Main_ClampU16(uint32_t value)
{
  return (value > 65535U) ? 65535U : (uint16_t)value;
}

static TrainingConfig_t Main_ConvertWirelessConfig(const WirelessTrainingConfig_t *wire)
{
  TrainingConfig_t config;
  memset(&config, 0, sizeof(config));
  config.version = wire->version;
  config.x_min = wire->x_min; config.x_home = wire->x_home; config.x_max = wire->x_max;
  config.y_min = wire->y_min; config.y_home = wire->y_home; config.y_max = wire->y_max;
  config.servo_settle_ms = (uint16_t)wire->servo_settle_10ms * 10U;
  config.fixation_duration_s = wire->fixation_duration_s;
  config.saccade_trials = wire->saccade_trials;
  config.saccade_response_ms = (uint16_t)wire->saccade_timeout_100ms * 100U;
  config.saccade_interval_ms = (uint16_t)wire->saccade_interval_10ms * 10U;
  config.saccade_pattern = wire->saccade_pattern;
  config.pursuit_speed_deg_s = wire->pursuit_speed_deg_s;
  config.pursuit_path = wire->pursuit_path;
  config.pursuit_rounds = wire->pursuit_rounds;
  config.pursuit_checkpoint_ms = (uint16_t)wire->pursuit_checkpoint_100ms * 100U;
  config.focus_trials = wire->focus_trials;
  config.focus_timeout_s = wire->focus_timeout_s;
  config.focus_interval_s = wire->focus_interval_s;
  config.focus_feedback_required = wire->focus_feedback_required;
  config.neglect_trials = wire->neglect_trials;
  config.neglect_timeout_ms = (uint16_t)wire->neglect_timeout_100ms * 100U;
  config.neglect_affected_side = wire->neglect_affected_side;
  config.neglect_affected_ratio = wire->neglect_affected_ratio;
  config.neglect_pattern = wire->neglect_pattern;
  config.head_warning_tenths = wire->head_warning_tenths;
  config.safety_turn_deg = wire->safety_turn_deg;
  config.safety_nod_deg = wire->safety_nod_deg;
  config.safety_tilt_deg = wire->safety_tilt_deg;
  config.resume_stable_s = wire->resume_stable_s;
  config.head_variation_limit_deg = wire->head_variation_limit_deg;
  return config;
}

static WirelessTrainingConfig_t Main_ConvertTrainingConfig(const TrainingConfig_t *config)
{
  WirelessTrainingConfig_t wire;
  memset(&wire, 0, sizeof(wire));
  wire.version = config->version;
  wire.x_min = config->x_min; wire.x_home = config->x_home; wire.x_max = config->x_max;
  wire.y_min = config->y_min; wire.y_home = config->y_home; wire.y_max = config->y_max;
  wire.servo_settle_10ms = (uint8_t)(config->servo_settle_ms / 10U);
  wire.fixation_duration_s = config->fixation_duration_s;
  wire.saccade_trials = config->saccade_trials;
  wire.saccade_timeout_100ms = (uint8_t)(config->saccade_response_ms / 100U);
  wire.saccade_interval_10ms = (uint8_t)(config->saccade_interval_ms / 10U);
  wire.saccade_pattern = config->saccade_pattern;
  wire.pursuit_speed_deg_s = config->pursuit_speed_deg_s;
  wire.pursuit_path = config->pursuit_path;
  wire.pursuit_rounds = config->pursuit_rounds;
  wire.pursuit_checkpoint_100ms = (uint8_t)(config->pursuit_checkpoint_ms / 100U);
  wire.focus_trials = config->focus_trials;
  wire.focus_timeout_s = config->focus_timeout_s;
  wire.focus_interval_s = config->focus_interval_s;
  wire.focus_feedback_required = config->focus_feedback_required;
  wire.neglect_trials = config->neglect_trials;
  wire.neglect_timeout_100ms = (uint8_t)(config->neglect_timeout_ms / 100U);
  wire.neglect_affected_side = config->neglect_affected_side;
  wire.neglect_affected_ratio = config->neglect_affected_ratio;
  wire.neglect_pattern = config->neglect_pattern;
  wire.head_warning_tenths = (uint8_t)config->head_warning_tenths;
  wire.safety_turn_deg = config->safety_turn_deg;
  wire.safety_nod_deg = config->safety_nod_deg;
  wire.safety_tilt_deg = config->safety_tilt_deg;
  wire.resume_stable_s = config->resume_stable_s;
  wire.head_variation_limit_deg = config->head_variation_limit_deg;
  return wire;
}

static void Main_SendReportSection(const WirelessReportQuery_t *query)
{
  WirelessReportData_t response;
  TrainingSessionReport_t session = App_GetSessionReport();
  TrainingConfig_t config = App_GetTrainingConfig();
  memset(&response, 0, sizeof(response));
  response.section = query->section;
  response.index = query->index;
  if (query->session_id != 0U && query->session_id != session.session_id)
  {
    WirelessBridge_SendReportData(query, &response);
    return;
  }

  if (query->section == 0U)
  {
    ReportSummaryWire_t data;
    memset(&data, 0, sizeof(data));
    data.session_id = session.session_id;
    data.flow_mode = session.flow_mode;
    data.mode_count = session.mode_count;
    data.completed_modes = session.completed_modes;
    data.completed = session.completed;
    data.overall_score = session.overall_score;
    memcpy(data.mode_order, session.mode_order, sizeof(data.mode_order));
    data.total_duration_ms = session.total_duration_ms;
    data.config_version = config.version;
    memcpy(response.data, &data, sizeof(data));
    response.valid = (session.session_id != 0U);
  }
  else if (query->section == 1U)
  {
    ModeReport_t report;
    ReportModeBasicWire_t data;
    memset(&data, 0, sizeof(data));
    if (App_GetModeReport(query->mode, &report) != 0U)
    {
      data.session_id = session.session_id;
      data.mode = report.mode; data.completed = report.completed;
      data.score = report.score; data.pause_count = report.pause_count;
      data.total_trials = report.total_trials; data.correct_trials = report.correct_trials;
      data.timeout_trials = report.timeout_trials;
      data.duration_s = Main_ClampU16(report.duration_ms / 1000U);
      data.avg_reaction_ms = Main_ClampU16(report.avg_reaction_ms);
      data.min_reaction_ms = Main_ClampU16(report.min_reaction_ms);
      data.max_reaction_ms = Main_ClampU16(report.max_reaction_ms);
      data.avg_head_offset_tenths = report.avg_head_offset_tenths;
      data.max_head_offset_tenths = report.max_head_offset_tenths;
      data.head_variation_tenths = report.head_variation_tenths;
      memcpy(response.data, &data, sizeof(data));
      response.valid = report.completed;
    }
  }
  else if (query->section == 2U)
  {
    ModeReport_t report;
    ReportModeDetailWire_t data;
    memset(&data, 0, sizeof(data));
    if (App_GetModeReport(query->mode, &report) != 0U)
    {
      int32_t left_rate = report.left_total ? (int32_t)report.left_correct * 1000 / report.left_total : 0;
      int32_t right_rate = report.right_total ? (int32_t)report.right_correct * 1000 / report.right_total : 0;
      data.session_id = session.session_id; data.mode = report.mode; data.valid = report.completed;
      data.head_over_limit_ms = report.head_over_limit_ms;
      data.compensation_permille = report.compensation_permille;
      data.left_total = report.left_total; data.left_correct = report.left_correct;
      data.right_total = report.right_total; data.right_correct = report.right_correct;
      data.trial_count = App_GetTrialCount(query->mode);
      data.affected_side = config.neglect_affected_side;
      data.asymmetry_permille = (int16_t)(left_rate - right_rate);
      memcpy(response.data, &data, sizeof(data));
      response.valid = report.completed;
    }
  }
  else if (query->section == 3U)
  {
    TrialResult_t trial;
    ReportTrialWire_t data;
    memset(&data, 0, sizeof(data));
    if (App_GetTrialResult(query->mode, query->index, &trial) != 0U)
    {
      data.session_id = session.session_id; data.mode = query->mode; data.index = query->index;
      data.target = trial.target; data.side = trial.side; data.correct = trial.correct;
      data.timed_out = trial.timed_out; data.reaction_ms = trial.reaction_ms;
      memcpy(response.data, &data, sizeof(data));
      response.valid = 1U;
    }
  }
  WirelessBridge_SendReportData(query, &response);
}

void Vofa_USART1_ErrorNotify(void)
{
  vofa_tx_busy = 0U;
  vofa_tx_fail_count++;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    vofa_tx_busy = 0U;
  }
  else if (huart->Instance == USART10)
  {
    WirelessBridge_OnTxComplete();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART10)
  {
    WirelessBridge_OnError();
  }
  else if (huart->Instance == USART1)
  {
    Vofa_USART1_ErrorNotify();
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
