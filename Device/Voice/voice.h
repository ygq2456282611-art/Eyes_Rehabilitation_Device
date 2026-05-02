#ifndef VOICE_H
#define VOICE_H

#include "main.h"
#include "usart.h"

void Voice_Init(void);
void Voice_PlayText(const char *text);
void Voice_PlayPrompt(uint8_t id);
void Voice_SetVolume(uint8_t vol);
void Voice_Stop(void);
uint8_t Voice_IsBusy(void);

#endif
