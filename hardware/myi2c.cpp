#include "myi2c.h"

#include "main.h"
#include "stm32f1xx_hal.h"

#ifndef MYI2C_SCL_GPIO_Port
#define MYI2C_SCL_GPIO_Port GPIOB
#endif
#ifndef MYI2C_SCL_Pin
#define MYI2C_SCL_Pin GPIO_PIN_10
#endif
#ifndef MYI2C_SDA_GPIO_Port
#define MYI2C_SDA_GPIO_Port GPIOB
#endif
#ifndef MYI2C_SDA_Pin
#define MYI2C_SDA_Pin GPIO_PIN_11
#endif

static inline void MyI2C_Delay(void)
{
  /* Rough software delay to keep bit-banged I2C timing stable on F1. */
  for (volatile uint32_t i = 0U; i < 32U; i++)
  {
    __NOP();
  }
}

extern "C" {

void MyI2C_W_SCL(uint8_t bit_value)
{
  HAL_GPIO_WritePin(MYI2C_SCL_GPIO_Port, MYI2C_SCL_Pin, bit_value ? GPIO_PIN_SET : GPIO_PIN_RESET);
  MyI2C_Delay();
}

void MyI2C_W_SDA(uint8_t bit_value)
{
  HAL_GPIO_WritePin(MYI2C_SDA_GPIO_Port, MYI2C_SDA_Pin, bit_value ? GPIO_PIN_SET : GPIO_PIN_RESET);
  MyI2C_Delay();
}

uint8_t MyI2C_R_SDA(void)
{
  MyI2C_Delay();
  return (HAL_GPIO_ReadPin(MYI2C_SDA_GPIO_Port, MYI2C_SDA_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

void MyI2C_Init(void)
{
  /* GPIO mode/clock are expected to be configured by CubeMX. */
  MyI2C_W_SCL(1U);
  MyI2C_W_SDA(1U);
}

void MyI2C_Start(void)
{
  MyI2C_W_SDA(1U);
  MyI2C_W_SCL(1U);
  MyI2C_W_SDA(0U);
  MyI2C_W_SCL(0U);
}

void MyI2C_Stop(void)
{
  MyI2C_W_SDA(0U);
  MyI2C_W_SCL(1U);
  MyI2C_W_SDA(1U);
}

void MyI2C_SendByte(uint8_t byte)
{
  uint8_t i;
  for (i = 0U; i < 8U; i++)
  {
    MyI2C_W_SDA((uint8_t)(!!(byte & (uint8_t)(0x80U >> i))));
    MyI2C_W_SCL(1U);
    MyI2C_W_SCL(0U);
  }
}

uint8_t MyI2C_ReceiveByte(void)
{
  uint8_t i;
  uint8_t byte = 0x00U;
  MyI2C_W_SDA(1U);
  for (i = 0U; i < 8U; i++)
  {
    MyI2C_W_SCL(1U);
    if (MyI2C_R_SDA())
    {
      byte |= (uint8_t)(0x80U >> i);
    }
    MyI2C_W_SCL(0U);
  }
  return byte;
}

void MyI2C_SendAck(uint8_t ack_bit)
{
  MyI2C_W_SDA(ack_bit);
  MyI2C_W_SCL(1U);
  MyI2C_W_SCL(0U);
}

uint8_t MyI2C_ReceiveAck(void)
{
  uint8_t ack_bit;
  MyI2C_W_SDA(1U);
  MyI2C_W_SCL(1U);
  ack_bit = MyI2C_R_SDA();
  MyI2C_W_SCL(0U);
  return ack_bit;
}

}  // extern "C"
