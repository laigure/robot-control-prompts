#include "serial.h"
#include "blue_serial.h"

#include <stdarg.h>
#include <stdio.h>

/* Bind one UART handle to a serial service instance. */
UartSerial::UartSerial(UART_HandleTypeDef *huart)
    : huart_(huart), rx_byte_(0), rx_head_(0), rx_tail_(0)
{
}

/* Integer power helper used by decimal number output. */
uint32_t UartSerial::Pow10(uint32_t x, uint32_t y)
{
  uint32_t result = 1U;
  while (y--)
  {
    result *= x;
  }
  return result;
}

/* Reset RX ring buffer and arm interrupt-based 1-byte reception. */
void UartSerial::Init(void)
{
  __disable_irq();
  rx_head_ = 0U;
  rx_tail_ = 0U;
  rx_byte_ = 0U;
  __enable_irq();
  (void)HAL_UART_Receive_IT(huart_, &rx_byte_, 1U);
}

/* Blocking send of one byte. */
void UartSerial::SendByte(uint8_t byte)
{
  (void)HAL_UART_Transmit(huart_, &byte, 1U, HAL_MAX_DELAY);
}

/* Blocking send of a byte array. */
void UartSerial::SendArray(const uint8_t *array, uint16_t length)
{
  if ((array == 0) || (length == 0U))
  {
    return;
  }
  (void)HAL_UART_Transmit(huart_, (uint8_t *)array, length, HAL_MAX_DELAY);
}

/* Blocking send of a null-terminated C string. */
void UartSerial::SendString(const char *string)
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
void UartSerial::SendNumber(uint32_t number, uint8_t length)
{
  uint8_t i;
  for (i = 0U; i < length; i++)
  {
    SendByte((uint8_t)(number / Pow10(10U, (uint32_t)length - i - 1U) % 10U + '0'));
  }
}

/* Format to stack buffer then send. */
void UartSerial::VPrintf(const char *format, va_list arg)
{
  char string[128];
  (void)vsnprintf(string, sizeof(string), format, arg);
  SendString(string);
}

/* printf-like send wrapper. */
void UartSerial::Printf(const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  VPrintf(format, arg);
  va_end(arg);
}

/* Check whether RX ring buffer has unread data. */
uint8_t UartSerial::HasRxData(void) const
{
  uint8_t has_data;
  __disable_irq();
  has_data = (rx_head_ != rx_tail_) ? 1U : 0U;
  __enable_irq();
  return has_data;
}

/* Pop one byte from RX ring buffer, return 0 if empty. */
uint8_t UartSerial::PopRxData(void)
{
  uint8_t data = 0U;
  __disable_irq();
  if (rx_head_ != rx_tail_)
  {
    data = rx_buffer_[rx_tail_];
    rx_tail_ = (uint16_t)((rx_tail_ + 1U) % kRxBufferSize);
  }
  __enable_irq();
  return data;
}

/* RX complete callback handler:
 * push received byte into ring buffer and re-arm IRQ receive.
 */
void UartSerial::OnRxComplete(UART_HandleTypeDef *huart)
{
  if (huart->Instance == huart_->Instance)
  {
    const uint16_t next = (uint16_t)((rx_head_ + 1U) % kRxBufferSize);
    if (next != rx_tail_)
    {
      rx_buffer_[rx_head_] = rx_byte_;
      rx_head_ = next;
    }
    (void)HAL_UART_Receive_IT(huart_, &rx_byte_, 1U);
  }
}

/* Board default serial instance on USART1. */
UartSerial &BoardSerial(void)
{
  static UartSerial serial(&huart1);
  return serial;
}

extern "C" {

/* C compatibility wrapper: initialize serial service. */
void Serial_Init(void)
{
  BoardSerial().Init();
}

/* C compatibility wrapper: send one byte. */
void Serial_SendByte(uint8_t byte)
{
  BoardSerial().SendByte(byte);
}

/* C compatibility wrapper: send byte array. */
void Serial_SendArray(const uint8_t *array, uint16_t length)
{
  BoardSerial().SendArray(array, length);
}

/* C compatibility wrapper: send null-terminated string. */
void Serial_SendString(const char *string)
{
  BoardSerial().SendString(string);
}

/* C compatibility wrapper: send fixed-width decimal number. */
void Serial_SendNumber(uint32_t number, uint8_t length)
{
  BoardSerial().SendNumber(number, length);
}

/* Redirect stdio output (printf) to UART. */
int fputc(int ch, FILE *f)
{
  (void)f;
  BoardSerial().SendByte((uint8_t)ch);
  return ch;
}

/* C printf-like wrapper. */
void Serial_Printf(const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  BoardSerial().VPrintf(format, arg);
  va_end(arg);
}

/* Return non-zero when RX data is available. */
uint8_t Serial_GetRxFlag(void)
{
  return BoardSerial().HasRxData();
}

/* Read one byte from RX queue. */
uint8_t Serial_GetRxData(void)
{
  return BoardSerial().PopRxData();
}

/* HAL weak callback override for UART RX complete IRQ. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  BoardSerial().OnRxComplete(huart);
  BoardBlueSerial().OnRxComplete(huart);
}

}  // extern "C"
