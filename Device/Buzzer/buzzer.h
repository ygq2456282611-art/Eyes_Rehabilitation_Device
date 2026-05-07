/**
 * @file    buzzer.h
 * @brief   板载蜂鸣器驱动（TIM12_CH2, PB15）
 *          基于 TIM12 PWM 输出驱动蜂鸣器，发出报警提示音
 *          配置：Prescaler=23, Period=1999 → 5kHz PWM
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
 * @brief  打开蜂鸣器
 * @param  volume : 音量 0~1000（占空比），推荐 500（50%）
 */
void Buzzer_On(uint16_t volume);

/**
 * @brief  关闭蜂鸣器
 */
void Buzzer_Off(void);

/**
 * @brief  蜂鸣器报警（特定模式：短促 N 次）
 * @param  count : 鸣叫次数
 * @param  on_ms  : 每次鸣叫持续时长（ms）
 * @param  off_ms : 间隔时长（ms）
 *         示例：Buzzer_Alert(3, 200, 100) → 哔-哔-哔
 */
void Buzzer_Alert(uint8_t count, uint16_t on_ms, uint16_t off_ms);

#endif
