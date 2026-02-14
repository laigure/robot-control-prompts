#pragma once

#include <stdint.h>
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
/** @brief Bit-banged software I2C bus driver. */
class SoftI2CBus {
public:
  /** @brief Construct with SCL/SDA GPIO definitions. */
  SoftI2CBus(GPIO_TypeDef *scl_port, uint16_t scl_pin, GPIO_TypeDef *sda_port, uint16_t sda_pin);

  /** @brief Set bus idle state. */
  void Init(void);
  /** @brief Generate I2C START condition. */
  void Start(void);
  /** @brief Generate I2C STOP condition. */
  void Stop(void);

  /** @brief Drive SCL level. */
  void WriteSCL(uint8_t bit_value);
  /** @brief Drive SDA level. */
  void WriteSDA(uint8_t bit_value);
  /** @brief Read SDA level. */
  uint8_t ReadSDA(void);

  /** @brief Send one byte (MSB first). */
  void SendByte(uint8_t byte);
  /** @brief Receive one byte (MSB first). */
  uint8_t ReceiveByte(void);
  /** @brief Send ACK/NACK bit. */
  void SendAck(uint8_t ack_bit);
  /** @brief Receive ACK bit from slave. */
  uint8_t ReceiveAck(void);

private:
  static inline void Delay(void)
  {
    for (volatile uint32_t i = 0U; i < 2U; i++)
    {
      __NOP();
    }
  }

private:
  GPIO_TypeDef *scl_port_;
  uint16_t scl_pin_;
  GPIO_TypeDef *sda_port_;
  uint16_t sda_pin_;
};

/** @brief Get board default software-I2C singleton. */
SoftI2CBus &BoardMyI2C(void);
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @brief C compatibility: drive SCL level. */
void MyI2C_W_SCL(uint8_t bit_value);
/** @brief C compatibility: drive SDA level. */
void MyI2C_W_SDA(uint8_t bit_value);
/** @brief C compatibility: read SDA level. */
uint8_t MyI2C_R_SDA(void);

/** @brief C compatibility: initialize software-I2C bus. */
void MyI2C_Init(void);
/** @brief C compatibility: generate START condition. */
void MyI2C_Start(void);
/** @brief C compatibility: generate STOP condition. */
void MyI2C_Stop(void);

/** @brief C compatibility: send one byte. */
void MyI2C_SendByte(uint8_t byte);
/** @brief C compatibility: receive one byte. */
uint8_t MyI2C_ReceiveByte(void);

/** @brief C compatibility: send ACK/NACK bit. */
void MyI2C_SendAck(uint8_t ack_bit);
/** @brief C compatibility: receive ACK bit. */
uint8_t MyI2C_ReceiveAck(void);

#ifdef __cplusplus
}
#endif
