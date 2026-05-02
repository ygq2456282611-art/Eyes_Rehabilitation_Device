#ifndef KEY_H
#define KEY_H

#include "main.h"

#define KEY_MODE    0
#define KEY_CONFIRM 1
#define KEY_BACK    2

typedef enum {
    KEY_EVENT_NONE  = 0,
    KEY_EVENT_SHORT = 1,
    KEY_EVENT_LONG  = 2,
} KeyEvent_t;

void Key_Init(void);
void Key_Scan(void);
KeyEvent_t Key_GetEvent(uint8_t key_id);

#endif
