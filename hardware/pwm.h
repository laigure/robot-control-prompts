#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void PWM_Init(void);
void PWM_SetCompare1(uint16_t compare);
void PWM_SetCompare2(uint16_t compare);

#ifdef __cplusplus
}
#endif

