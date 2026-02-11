#pragma once

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void Led_Init(void);
void Led_On(void);
void Led_Off(void);
void Led_Toggle(void);
void Led_BlinkLoop(uint32_t delay_ms);

#ifdef __cplusplus
}
#endif
