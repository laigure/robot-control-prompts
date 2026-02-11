#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Key_Init(void);
uint8_t Key_GetNum(void);
uint8_t Key_GetState(void);
void Key_Tick(void);

#ifdef __cplusplus
}
#endif

