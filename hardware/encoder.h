#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Encoder_Init(void);
int16_t Encoder_Get(uint8_t n);

#ifdef __cplusplus
}
#endif

