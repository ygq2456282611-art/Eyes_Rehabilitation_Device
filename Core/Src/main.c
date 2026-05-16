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
#include "sbus.h"
#include <stdio.h>
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
static uint8_t sbus_online;
static uint16_t sbus_swa;
static uint16_t sbus_swb;
static uint8_t sbus_swb_middle;
static uint8_t sbus_zero_event;
static uint8_t sbus_zero_event_report;
static bmi088_debug_data_t bmi088_debug;
static char vofa_buf[512];
static uint32_t vofa_last_send_ms;
static uint32_t ws2812_alert_tick = 0;
static uint8_t  ws2812_alert_on = 0;
static uint8_t  prev_alert_active = 0;
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
  MX_UART5_Init();
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
    Servo_Test();// 舵机自检：依次转到 0°、90°、180°，验证舵机控制和 PWM 输出正常

    Laser_Init();
    Laser_Test();// 点亮激光并闪烁 3 次，验证 PA0 输出正常

    Key_Init();
    LED_Init();
    HeadTracker_Init();
    App_Init();
    Buzzer_Init();
    WS2812_Init();
    Voice_Init();
    SBUS_Init();
    
    Buzzer_Alert(3, 200, 100);
    Laser_Blink(200, 3);
    HAL_Delay(4000);
    Voice_Play(0xFF, VOICE_TTS_INIT_OK); // 初始化完成播报
    HAL_Delay(4000);
    Voice_Play(0xFF, VOICE_TTS_WELCOME); // 欢迎
    HAL_Delay(2000);

    /* ==== PA2 按键测试（成功后删除）==== */
    Laser_Blink(200, 2);
    while (1)
    {
        Key_Scan();
        if (Key_GetEvent(KEY_PATIENT) == KEY_EVENT_SHORT)
        {
            Buzzer_Alert(2, 150, 100);
            break;
        }
        HAL_Delay(10);
    }
    /* ================================== */
    //Calibrate_ServoRange();  /* 标定模式已改为语音命令触发 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        SBUS_Update(HAL_GetTick());
        BMI088_Task();
        sbus_zero_event = SBUS_TakeSwbMiddleEvent();
        if (sbus_zero_event != 0U)
        {
            sbus_zero_event_report = 1U;
            BMI088_ResetReference();
            HeadTracker_Init();
        }

        BMI088_read_euler(&euler_angle, &temp);
        HeadTracker_Update(&euler_angle, 0.01f);

        HeadAnalysis_t *head = HeadTracker_GetResult();
        (void)head;

        uint8_t alert_active = 0;
        /* Vertical mount semantics: roll=nod, pitch=head turn, yaw=lateral tilt. */
        if (fabsf(euler_angle.roll)  > 20.0f) alert_active = 1;
        if (fabsf(euler_angle.pitch) > 20.0f) alert_active = 1;
        if (fabsf(euler_angle.yaw)   > 30.0f) alert_active = 1;

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

        if ((HAL_GetTick() - vofa_last_send_ms) >= 20U)
        {
            int len;

            vofa_last_send_ms = HAL_GetTick();
            BMI088_GetLatestFloat(vofa_gyro, vofa_accel, &temp, &vofa_timestamp_us);
            BMI088_GetEuler(&vofa_roll_deg, &vofa_pitch_deg, &vofa_yaw_deg, &vofa_dt, &vofa_timestamp_us);
            BMI088_GetGyroBias(vofa_bias);
            imu_calibrated = BMI088_IsCalibrated();
            imu_is_static = BMI088_IsStatic();
            sbus_online = SBUS_IsOnline();
            sbus_swa = SBUS_GetChannel(SBUS_SWA_CH_INDEX);
            sbus_swb = SBUS_GetChannel(SBUS_SWB_CH_INDEX);
            sbus_swb_middle = SBUS_IsSwbMiddle();
            BMI088_GetDebug(&bmi088_debug);

            len = snprintf(vofa_buf, sizeof(vofa_buf),
                           "imu:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%u,%u,%.2f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
                           euler_angle.roll, euler_angle.pitch, euler_angle.yaw,
                           vofa_roll_deg, vofa_pitch_deg, vofa_yaw_deg,
                           vofa_gyro[0], vofa_gyro[1], vofa_gyro[2],
                           vofa_accel[0], vofa_accel[1], vofa_accel[2],
                           vofa_bias[0], vofa_bias[1], vofa_bias[2],
                           (unsigned int)imu_is_static,
                           (unsigned int)imu_calibrated,
                           temp,
                           (unsigned int)sbus_online,
                           (unsigned int)sbus_swa,
                           (unsigned int)sbus_swb,
                           (unsigned int)sbus_swb_middle,
                           (unsigned int)sbus_zero_event_report,
                           (unsigned int)App_GetState(),
                           (unsigned int)App_GetMode(),
                           (unsigned int)bmi088_debug.acc_exti_count,
                           (unsigned int)bmi088_debug.gyro_exti_count,
                           (unsigned int)bmi088_debug.gyro_dma_count,
                           (unsigned int)bmi088_debug.accel_dma_count,
                           (unsigned int)bmi088_debug.temp_dma_count,
                           (unsigned int)bmi088_debug.dma_error_count,
                           (unsigned int)bmi088_debug.last_error,
                           (unsigned int)bmi088_debug.spi_busy,
                           (unsigned int)bmi088_debug.active_transaction,
                           (unsigned int)bmi088_debug.gyro_pending,
                           (unsigned int)bmi088_debug.accel_pending,
                           (unsigned int)bmi088_debug.temp_pending,
                           (unsigned int)bmi088_debug.gyro_timestamp_us,
                           (unsigned int)bmi088_debug.accel_timestamp_us,
                           (unsigned int)bmi088_debug.temp_timestamp_us);

            if ((len > 0) && (len < (int)sizeof(vofa_buf)))
            {
                HAL_UART_Transmit(&huart1, (uint8_t *)vofa_buf, (uint16_t)len, 20);
                sbus_zero_event_report = 0U;
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
