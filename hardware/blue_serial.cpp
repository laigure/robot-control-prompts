#include "blue_serial.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Bind one UART handle to one Bluetooth serial service instance. */
BlueSerialLink::BlueSerialLink(UART_HandleTypeDef *huart)
    : huart_(huart), rx_flag_(0U), rx_byte_(0U), rx_state_(0U), rx_index_(0U)
{
  rx_packet_[0] = '\0';
}

/* Integer power helper used by decimal number output. */
uint32_t BlueSerialLink::Pow10(uint32_t x, uint32_t y)
{
  uint32_t result = 1U;
  while (y--)
  {
    result *= x;
  }
  return result;
}

/* Arm 1-byte interrupt RX on the bound UART. */
void BlueSerialLink::StartReceiveIt(void)
{
  (void)HAL_UART_Receive_IT(huart_, &rx_byte_, 1U);
}

/* Initialize USART2 and reset protocol parser state. */
void BlueSerialLink::Init(void)
{
  MX_USART2_UART_Init();

  __disable_irq();
  rx_flag_ = 0U;
  rx_byte_ = 0U;
  rx_state_ = 0U;
  rx_index_ = 0U;
  rx_packet_[0] = '\0';
  __enable_irq();

  StartReceiveIt();
}

/* Blocking send of one byte. */
void BlueSerialLink::SendByte(uint8_t byte)
{
  (void)HAL_UART_Transmit(huart_, &byte, 1U, HAL_MAX_DELAY);
}

/* Blocking send of byte array. */
void BlueSerialLink::SendArray(const uint8_t *array, uint16_t length)
{
  if ((array == 0) || (length == 0U))
  {
    return;
  }
  (void)HAL_UART_Transmit(huart_, (uint8_t *)array, length, HAL_MAX_DELAY);
}

/* Blocking send of null-terminated C string. */
void BlueSerialLink::SendString(const char *string)
{
  if (string == 0)
  {
    return;
  }
  while (*string != '\0')
  {
    SendByte((uint8_t)(*string));
    string++;
  }
}

/* Send fixed-length unsigned decimal (zero-padded). */
void BlueSerialLink::SendNumber(uint32_t number, uint8_t length)
{
  uint8_t i;
  for (i = 0U; i < length; i++)
  {
    SendByte((uint8_t)(number / Pow10(10U, (uint32_t)length - i - 1U) % 10U + '0'));
  }
}

/* Format to stack buffer then send. */
void BlueSerialLink::VPrintf(const char *format, va_list arg)
{
  char buffer[kPacketBufferSize];
  (void)vsnprintf(buffer, sizeof(buffer), format, arg);
  SendString(buffer);
}

/* printf-like send wrapper. */
void BlueSerialLink::Printf(const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  VPrintf(format, arg);
  va_end(arg);
}

/* Check whether one framed packet is ready. */
uint8_t BlueSerialLink::HasPacket(void) const
{
  uint8_t flag;
  __disable_irq();
  flag = rx_flag_;
  __enable_irq();
  return flag;
}

/* Clear packet-ready flag. */
void BlueSerialLink::ClearPacketFlag(void)
{
  __disable_irq();
  rx_flag_ = 0U;
  __enable_irq();
}

/* Copy and consume one payload packet. */
uint8_t BlueSerialLink::ReadPacket(char *out, uint16_t out_size)
{
  uint8_t got = 0U;

  if ((out == 0) || (out_size == 0U))
  {
    return 0U;
  }

  __disable_irq();
  if (rx_flag_ != 0U)
  {
    size_t copy_len = strlen(rx_packet_);
    if (copy_len >= out_size)
    {
      copy_len = (size_t)(out_size - 1U);
    }
    (void)memcpy(out, rx_packet_, copy_len);
    out[copy_len] = '\0';
    rx_flag_ = 0U;
    got = 1U;
  }
  __enable_irq();

  return got;
}

/* RX byte parser for [payload] protocol, invoked from HAL callback. */
void BlueSerialLink::OnRxComplete(UART_HandleTypeDef *huart)
{
  if (huart->Instance != huart_->Instance)
  {
    return;
  }

  if (rx_state_ == 0U)
  {
    if ((rx_byte_ == (uint8_t)'[') && (rx_flag_ == 0U))
    {
      rx_state_ = 1U;
      rx_index_ = 0U;
    }
  }
  else
  {
    if (rx_byte_ == (uint8_t)']')
    {
      rx_packet_[rx_index_] = '\0';
      rx_flag_ = 1U;
      rx_state_ = 0U;
    }
    else if (rx_index_ < (uint8_t)(kPacketBufferSize - 1U))
    {
      rx_packet_[rx_index_] = (char)rx_byte_;
      rx_index_++;
    }
  }

  StartReceiveIt();
}

namespace {
/* Board default bluetooth serial instance on USART2. */
BlueSerialLink g_blue_serial(&huart2);
}

/* Accessor for board bluetooth serial singleton. */
BlueSerialLink &BoardBlueSerial(void)
{
  return g_blue_serial;
}

extern "C" {

/* C compatibility wrapper: initialize bluetooth serial service. */
void BlueSerial_Init(void)
{
  BoardBlueSerial().Init();
}

/* C compatibility wrapper: send one byte. */
void BlueSerial_SendByte(uint8_t byte)
{
  BoardBlueSerial().SendByte(byte);
}

/* C compatibility wrapper: send byte array. */
void BlueSerial_SendArray(const uint8_t *array, uint16_t length)
{
  BoardBlueSerial().SendArray(array, length);
}

/* C compatibility wrapper: send null-terminated string. */
void BlueSerial_SendString(const char *string)
{
  BoardBlueSerial().SendString(string);
}

/* C compatibility wrapper: send fixed-width decimal number. */
void BlueSerial_SendNumber(uint32_t number, uint8_t length)
{
  BoardBlueSerial().SendNumber(number, length);
}

/* C compatibility wrapper: printf-like send. */
void BlueSerial_Printf(const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  BoardBlueSerial().VPrintf(format, arg);
  va_end(arg);
}

/* C compatibility wrapper: return non-zero when one packet is ready. */
uint8_t BlueSerial_GetRxFlag(void)
{
  return BoardBlueSerial().HasPacket();
}

/* C compatibility wrapper: clear packet-ready flag. */
void BlueSerial_ClearRxFlag(void)
{
  BoardBlueSerial().ClearPacketFlag();
}

/* C compatibility wrapper: copy and consume one packet payload. */
uint8_t BlueSerial_ReadPacket(char *out, uint16_t out_size)
{
  return BoardBlueSerial().ReadPacket(out, out_size);
}

/* C compatibility wrapper: HAL RX complete callback entry. */
void BlueSerial_OnRxComplete(UART_HandleTypeDef *huart)
{
  BoardBlueSerial().OnRxComplete(huart);
}

}  // extern "C"
