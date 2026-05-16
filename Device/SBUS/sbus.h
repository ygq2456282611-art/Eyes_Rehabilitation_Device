#ifndef SBUS_H
#define SBUS_H

#include <stdint.h>

#define SBUS_CHANNEL_COUNT 16U
#define SBUS_SWA_CH_INDEX 4U
#define SBUS_SWB_CH_INDEX 5U

typedef enum
{
    SBUS_SWITCH_LOW = 0,
    SBUS_SWITCH_MIDDLE,
    SBUS_SWITCH_HIGH
} sbus_switch_pos_t;

void SBUS_Init(void);
void SBUS_Update(uint32_t now_ms);
uint8_t SBUS_IsOnline(void);
uint16_t SBUS_GetChannel(uint8_t ch_index);
sbus_switch_pos_t SBUS_GetSwbPosition(void);
uint8_t SBUS_IsSwbMiddle(void);
uint8_t SBUS_TakeSwbMiddleEvent(void);

#endif
