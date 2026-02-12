#include "myi2c.h"

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

/* Bind software-I2C SCL/SDA pins. */
SoftI2CBus::SoftI2CBus(GPIO_TypeDef *scl_port, uint16_t scl_pin, GPIO_TypeDef *sda_port, uint16_t sda_pin)
    : scl_port_(scl_port), scl_pin_(scl_pin), sda_port_(sda_port), sda_pin_(sda_pin)
{
}

/* Drive SCL and wait one short software timing slot. */
void SoftI2CBus::WriteSCL(uint8_t bit_value)
{
  HAL_GPIO_WritePin(scl_port_, scl_pin_, bit_value ? GPIO_PIN_SET : GPIO_PIN_RESET);
  Delay();
}

/* Drive SDA and wait one short software timing slot. */
void SoftI2CBus::WriteSDA(uint8_t bit_value)
{
  HAL_GPIO_WritePin(sda_port_, sda_pin_, bit_value ? GPIO_PIN_SET : GPIO_PIN_RESET);
  Delay();
}

/* Sample SDA input line. */
uint8_t SoftI2CBus::ReadSDA(void)
{
  Delay();
  return (HAL_GPIO_ReadPin(sda_port_, sda_pin_) == GPIO_PIN_SET) ? 1U : 0U;
}

/* Bus idle state (both lines high). */
void SoftI2CBus::Init(void)
{
  WriteSCL(1U);
  WriteSDA(1U);
}

/* Generate I2C START condition. */
void SoftI2CBus::Start(void)
{
  WriteSDA(1U);
  WriteSCL(1U);
  WriteSDA(0U);
  WriteSCL(0U);
}

/* Generate I2C STOP condition. */
void SoftI2CBus::Stop(void)
{
  WriteSDA(0U);
  WriteSCL(1U);
  WriteSDA(1U);
}

/* Send one byte MSB-first. */
void SoftI2CBus::SendByte(uint8_t byte)
{
  uint8_t i;
  for (i = 0U; i < 8U; i++)
  {
    WriteSDA((uint8_t)(!!(byte & (uint8_t)(0x80U >> i))));
    WriteSCL(1U);
    WriteSCL(0U);
  }
}

/* Receive one byte MSB-first. */
uint8_t SoftI2CBus::ReceiveByte(void)
{
  uint8_t i;
  uint8_t byte = 0U;
  WriteSDA(1U);
  for (i = 0U; i < 8U; i++)
  {
    WriteSCL(1U);
    if (ReadSDA())
    {
      byte |= (uint8_t)(0x80U >> i);
    }
    WriteSCL(0U);
  }
  return byte;
}

/* Send ACK/NACK bit. */
void SoftI2CBus::SendAck(uint8_t ack_bit)
{
  WriteSDA(ack_bit);
  WriteSCL(1U);
  WriteSCL(0U);
}

/* Read ACK bit from slave (0 means ACK). */
uint8_t SoftI2CBus::ReceiveAck(void)
{
  uint8_t ack_bit;
  WriteSDA(1U);
  WriteSCL(1U);
  ack_bit = ReadSDA();
  WriteSCL(0U);
  return ack_bit;
}

namespace {
/* Board default software-I2C bus (PB10/PB11). */
SoftI2CBus g_i2c(MYI2C_SCL_GPIO_Port, MYI2C_SCL_Pin, MYI2C_SDA_GPIO_Port, MYI2C_SDA_Pin);
}

/* Accessor for board software-I2C singleton. */
SoftI2CBus &BoardMyI2C(void)
{
  return g_i2c;
}

extern "C" {

/* C compatibility wrapper: drive SCL level. */
void MyI2C_W_SCL(uint8_t bit_value)
{
  BoardMyI2C().WriteSCL(bit_value);
}

/* C compatibility wrapper: drive SDA level. */
void MyI2C_W_SDA(uint8_t bit_value)
{
  BoardMyI2C().WriteSDA(bit_value);
}

/* C compatibility wrapper: read SDA level. */
uint8_t MyI2C_R_SDA(void)
{
  return BoardMyI2C().ReadSDA();
}

/* C compatibility wrapper: initialize software-I2C lines. */
void MyI2C_Init(void)
{
  BoardMyI2C().Init();
}

/* C compatibility wrapper: send START condition. */
void MyI2C_Start(void)
{
  BoardMyI2C().Start();
}

/* C compatibility wrapper: send STOP condition. */
void MyI2C_Stop(void)
{
  BoardMyI2C().Stop();
}

/* C compatibility wrapper: send one byte. */
void MyI2C_SendByte(uint8_t byte)
{
  BoardMyI2C().SendByte(byte);
}

/* C compatibility wrapper: receive one byte. */
uint8_t MyI2C_ReceiveByte(void)
{
  return BoardMyI2C().ReceiveByte();
}

/* C compatibility wrapper: send ACK/NACK bit. */
void MyI2C_SendAck(uint8_t ack_bit)
{
  BoardMyI2C().SendAck(ack_bit);
}

/* C compatibility wrapper: receive ACK bit. */
uint8_t MyI2C_ReceiveAck(void)
{
  return BoardMyI2C().ReceiveAck();
}

}  // extern "C"
