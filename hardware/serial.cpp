#include "serial.h"

#include "usart.h"

#include <stdarg.h>
#include <stdio.h>

static uint8_t s_rx_byte = 0;
static volatile uint16_t s_rx_head = 0;
static volatile uint16_t s_rx_tail = 0;
static uint8_t s_rx_buf[128];

static uint32_t Serial_Pow(uint32_t x, uint32_t y)
{
  uint32_t result = 1;
  while (y--)
  {
    result *= x;
  }
  return result;
}

extern "C" {

void Serial_Init(void)
{
  s_rx_head = 0;
  s_rx_tail = 0;
  s_rx_byte = 0;
  (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
}

void Serial_SendByte(uint8_t byte)
{
  (void)HAL_UART_Transmit(&huart1, &byte, 1, HAL_MAX_DELAY);
}

void Serial_SendArray(const uint8_t *array, uint16_t length)
{
  if ((array == NULL) || (length == 0U))
  {
    return;
  }
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)array, length, HAL_MAX_DELAY);
}

void Serial_SendString(const char *string)
{
  if (string == NULL)
  {
    return;
  }

  while (*string != '\0')
  {
    Serial_SendByte((uint8_t)(*string));
    string++;
  }
}

void Serial_SendNumber(uint32_t number, uint8_t length)
{
  uint8_t i;
  for (i = 0; i < length; i++)
  {
    Serial_SendByte((uint8_t)(number / Serial_Pow(10U, length - i - 1U) % 10U + '0'));
  }
}

int fputc(int ch, FILE *f)
{
  (void)f;
  Serial_SendByte((uint8_t)ch);
  return ch;
}

void Serial_Printf(const char *format, ...)
{
  char string[128];
  va_list arg;
  va_start(arg, format);
  vsnprintf(string, sizeof(string), format, arg);
  va_end(arg);
  Serial_SendString(string);
}

uint8_t Serial_GetRxFlag(void)
{
  uint8_t has_data;
  __disable_irq();
  has_data = (s_rx_head != s_rx_tail) ? 1U : 0U;
  __enable_irq();
  return has_data;
}

uint8_t Serial_GetRxData(void)
{
  uint8_t data = 0;
  __disable_irq();
  if (s_rx_head != s_rx_tail)
  {
    data = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % (uint16_t)sizeof(s_rx_buf));
  }
  __enable_irq();
  return data;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uint16_t next = (uint16_t)((s_rx_head + 1U) % (uint16_t)sizeof(s_rx_buf));
    if (next != s_rx_tail)
    {
      s_rx_buf[s_rx_head] = s_rx_byte;
      s_rx_head = next;
    }
    (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
  }
}

}  // extern "C"
