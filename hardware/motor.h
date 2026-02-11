#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Motor_Init(void);
void Motor_SetPWM(uint8_t n, int8_t pwm);

#ifdef __cplusplus
}
#endif

