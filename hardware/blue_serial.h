#pragma once

#include <stdint.h>
#include <stdarg.h>

#include "usart.h"

#ifdef __cplusplus
/** @brief Bluetooth serial service on USART2 with framed RX parsing: [payload]. */
class BlueSerialLink {
public:
  /** @brief Construct service with target UART handle. */
  explicit BlueSerialLink(UART_HandleTypeDef *huart);

  /** @brief Initialize USART2 and arm interrupt-based RX. */
  void Init(void);
  /** @brief Blocking send one byte. */
  void SendByte(uint8_t byte);
  /** @brief Blocking send byte array. */
  void SendArray(const uint8_t *array, uint16_t length);
  /** @brief Blocking send null-terminated string. */
  void SendString(const char *string);
  /** @brief Send fixed-width decimal with zero padding. */
  void SendNumber(uint32_t number, uint8_t length);
  /** @brief printf-like output helper. */
  void Printf(const char *format, ...);
  /** @brief va_list variant of printf-like helper. */
  void VPrintf(const char *format, va_list arg);

  /** @brief Return non-zero when one framed packet is ready. */
  uint8_t HasPacket(void) const;
  /** @brief Clear packet-ready flag. */
  void ClearPacketFlag(void);
  /** @brief Copy and consume one packet payload into out buffer. */
  uint8_t ReadPacket(char *out, uint16_t out_size);
  /** @brief HAL RX complete callback entry (byte parser + re-arm). */
  void OnRxComplete(UART_HandleTypeDef *huart);

private:
  static uint32_t Pow10(uint32_t x, uint32_t y);
  void StartReceiveIt(void);

private:
  static const uint16_t kPacketBufferSize = 100U;
  UART_HandleTypeDef *huart_;
  volatile uint8_t rx_flag_;
  uint8_t rx_byte_;
  uint8_t rx_state_;
  uint8_t rx_index_;
  char rx_packet_[kPacketBufferSize];
};

/** @brief Get board default Bluetooth serial singleton (USART2). */
BlueSerialLink &BoardBlueSerial(void);
#endif

#ifdef __cplusplus
extern "C" {
#endif

void BlueSerial_Init(void);
void BlueSerial_SendByte(uint8_t byte);
void BlueSerial_SendArray(const uint8_t *array, uint16_t length);
void BlueSerial_SendString(const char *string);
void BlueSerial_SendNumber(uint32_t number, uint8_t length);
void BlueSerial_Printf(const char *format, ...);

uint8_t BlueSerial_GetRxFlag(void);
void BlueSerial_ClearRxFlag(void);
uint8_t BlueSerial_ReadPacket(char *out, uint16_t out_size);

void BlueSerial_OnRxComplete(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif
