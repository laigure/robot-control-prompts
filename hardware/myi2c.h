#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void MyI2C_W_SCL(uint8_t bit_value);
void MyI2C_W_SDA(uint8_t bit_value);
uint8_t MyI2C_R_SDA(void);

void MyI2C_Init(void);
void MyI2C_Start(void);
void MyI2C_Stop(void);

void MyI2C_SendByte(uint8_t byte);
uint8_t MyI2C_ReceiveByte(void);

void MyI2C_SendAck(uint8_t ack_bit);
uint8_t MyI2C_ReceiveAck(void);

#ifdef __cplusplus
}
#endif

