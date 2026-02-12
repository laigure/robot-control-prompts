#pragma once

#include <stdint.h>
#include "myi2c.h"

#ifdef __cplusplus
/** @brief MPU6050 driver on top of software-I2C bus. */
class Mpu6050Device {
public:
  /** @brief Construct with target I2C bus and write address (default 0xD0). */
  explicit Mpu6050Device(SoftI2CBus &bus, uint8_t address_write = 0xD0U);

  /** @brief Initialize device registers and validate WHO_AM_I. */
  bool Init(void);
  /** @brief Read WHO_AM_I register. */
  uint8_t GetID(void);
  /** @brief Read accel/gyro raw data frame. */
  void GetData(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z,
               int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);

  /** @brief Write one register. */
  bool WriteReg(uint8_t reg_address, uint8_t data);
  /** @brief Read one register. */
  uint8_t ReadReg(uint8_t reg_address);
  /** @brief Read continuous register block. */
  bool ReadRegs(uint8_t reg_address, uint8_t *data_array, uint8_t count);

private:
  bool IsSupportedID(uint8_t id) const;

private:
  SoftI2CBus &bus_;
  uint8_t address_write_;
  uint8_t online_;
};

/** @brief Get board default MPU6050 singleton. */
Mpu6050Device &BoardMpu6050(void);
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @brief C compatibility: write one MPU register. */
void MPU6050_WriteReg(uint8_t reg_address, uint8_t data);
/** @brief C compatibility: read one MPU register. */
uint8_t MPU6050_ReadReg(uint8_t reg_address);
/** @brief C compatibility: read continuous register block. */
void MPU6050_ReadRegs(uint8_t reg_address, uint8_t *data_array, uint8_t count);

/** @brief C compatibility: initialize MPU device. */
void MPU6050_Init(void);
/** @brief C compatibility: read WHO_AM_I register. */
uint8_t MPU6050_GetID(void);

/** @brief C compatibility: read accel/gyro raw data. */
void MPU6050_GetData(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z,
                     int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);

#ifdef __cplusplus
}
#endif
