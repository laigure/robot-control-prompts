#pragma once

#include <stdint.h>
#include <stdarg.h>

#include "usart.h"

#ifdef __cplusplus
/** @brief UART serial service with IRQ RX ring buffer. */
class UartSerial {
public:
  /** @brief Construct with a HAL UART handle. */
  explicit UartSerial(UART_HandleTypeDef *huart);

  /** @brief Initialize service and arm RX interrupt. */
  void Init(void);
  /** @brief Blocking send one byte. */
  void SendByte(uint8_t byte);
  /** @brief Blocking send byte array. */
  void SendArray(const uint8_t *array, uint16_t length);
  /** @brief Blocking send null-terminated string. */
  void SendString(const char *string);
  /** @brief Send fixed-width decimal with zero padding. */
  void SendNumber(uint32_t number, uint8_t length);
  /** @brief printf-like output. */
  void Printf(const char *format, ...);
  /** @brief va_list variant of printf-like output. */
  void VPrintf(const char *format, va_list arg);

  /** @brief Return non-zero when RX buffer has unread data. */
  uint8_t HasRxData(void) const;
  /** @brief Pop one byte from RX buffer, returns 0 when empty. */
  uint8_t PopRxData(void);
  /** @brief HAL RX complete callback handler (push + re-arm). */
  void OnRxComplete(UART_HandleTypeDef *huart);

private:
  static const uint16_t kRxBufferSize = 128U;
  static uint32_t Pow10(uint32_t x, uint32_t y);

private:
  UART_HandleTypeDef *huart_;
  uint8_t rx_byte_;
  volatile uint16_t rx_head_;
  volatile uint16_t rx_tail_;
  uint8_t rx_buffer_[kRxBufferSize];
};

/** @brief Get board default serial singleton (USART1). */
UartSerial &BoardSerial(void);
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @brief C compatibility: initialize serial service. */
void Serial_Init(void);
/** @brief C compatibility: send one byte. */
void Serial_SendByte(uint8_t byte);
/** @brief C compatibility: send byte array. */
void Serial_SendArray(const uint8_t *array, uint16_t length);
/** @brief C compatibility: send null-terminated string. */
void Serial_SendString(const char *string);
/** @brief C compatibility: send fixed-width decimal number. */
void Serial_SendNumber(uint32_t number, uint8_t length);
/** @brief C compatibility: printf-like send. */
void Serial_Printf(const char *format, ...);

/** @brief C compatibility: return non-zero if RX data is available. */
uint8_t Serial_GetRxFlag(void);
/** @brief C compatibility: pop one RX byte. */
uint8_t Serial_GetRxData(void);

#ifdef __cplusplus
}
#endif
