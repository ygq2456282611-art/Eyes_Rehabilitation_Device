/**
 * @file    buzzer.h
 * @brief   板载蜂鸣器驱动（TIM12_CH2, PB15）
 *          基于 TIM12 PWM 输出驱动蜂鸣器，发出报警提示音
 *          配置：Prescaler=23, Period=1999 → 5kHz PWM
 *          参考 DM-02 官方例程 CtrBoard-H7_BUZZER
 */
#ifndef BUZZER_H
#define BUZZER_H

#include "main.h"

/**
 * @brief  初始化蜂鸣器
 *         配置 PB15 为 TIM12_CH2 复用功能，启动 PWM 输出
 */
void Buzzer_Init(void);

/**
 * @brief  打开蜂鸣器（设置占空比）
 * @param  volume : 音量 0~1000，推荐 500（50% 占空比）
 */
void Buzzer_On(uint16_t volume);

/**
 * @brief  关闭蜂鸣器
 */
void Buzzer_Off(void);

/**
 * @brief  蜂鸣器模式化报警
 * @param  count  : 鸣叫次数
 * @param  on_ms  : 每次鸣叫持续时长（ms）
 * @param  off_ms : 间隔静音时长（ms）
 *         示例：Buzzer_Alert(3, 200, 100) → "哔-哔-哔"
 */
void Buzzer_Alert(uint8_t count, uint16_t on_ms, uint16_t off_ms);

#endif
