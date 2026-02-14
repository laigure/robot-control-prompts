#include "mpu6050.h"

#include "mpu6050_reg.h"

/* Bind one MPU6050 device to an I2C bus and write address. */
Mpu6050Device::Mpu6050Device(SoftI2CBus &bus, uint8_t address_write)
    : bus_(bus), address_write_(address_write), online_(0U)
{
}

/* Accept several WHO_AM_I values seen on common MPU6050 clones/modules. */
bool Mpu6050Device::IsSupportedID(uint8_t id) const
{
  return (id == 0x68U) || (id == 0x69U) || (id == 0x70U) || (id == 0x71U);
}

/* Write one register over software-I2C. */
bool Mpu6050Device::WriteReg(uint8_t reg_address, uint8_t data)
{
  bus_.Start();
  bus_.SendByte(address_write_);
  if (bus_.ReceiveAck() != 0U)
  {
    bus_.Stop();
    return false;
  }
  bus_.SendByte(reg_address);
  if (bus_.ReceiveAck() != 0U)
  {
    bus_.Stop();
    return false;
  }
  bus_.SendByte(data);
  if (bus_.ReceiveAck() != 0U)
  {
    bus_.Stop();
    return false;
  }
  bus_.Stop();
  return true;
}

/* Read one register over software-I2C. */
uint8_t Mpu6050Device::ReadReg(uint8_t reg_address)
{
  uint8_t data = 0xFFU;

  bus_.Start();
  bus_.SendByte(address_write_);
  if (bus_.ReceiveAck() != 0U)
  {
    bus_.Stop();
    return 0xFFU;
  }
  bus_.SendByte(reg_address);
  if (bus_.ReceiveAck() != 0U)
  {
    bus_.Stop();
    return 0xFFU;
  }

  bus_.Start();
  bus_.SendByte((uint8_t)(address_write_ | 0x01U));
  if (bus_.ReceiveAck() != 0U)
  {
    bus_.Stop();
    return 0xFFU;
  }
  data = bus_.ReceiveByte();
  bus_.SendAck(1U);
  bus_.Stop();
  return data;
}

/* Read a continuous register block over software-I2C. */
bool Mpu6050Device::ReadRegs(uint8_t reg_address, uint8_t *data_array, uint8_t count)
{
  uint8_t i;

  if ((data_array == 0) || (count == 0U))
  {
    return false;
  }

  bus_.Start();
  bus_.SendByte(address_write_);
  if (bus_.ReceiveAck() != 0U)
  {
    bus_.Stop();
    return false;
  }
  bus_.SendByte(reg_address);
  if (bus_.ReceiveAck() != 0U)
  {
    bus_.Stop();
    return false;
  }

  bus_.Start();
  bus_.SendByte((uint8_t)(address_write_ | 0x01U));
  if (bus_.ReceiveAck() != 0U)
  {
    bus_.Stop();
    return false;
  }

  for (i = 0U; i < count; i++)
  {
    data_array[i] = bus_.ReceiveByte();
    bus_.SendAck((i == (uint8_t)(count - 1U)) ? 1U : 0U);
  }
  bus_.Stop();
  return true;
}

/* Configure MPU6050 working mode and validate device ID. */
bool Mpu6050Device::Init(void)
{
  bus_.Init();
  online_ = 0U;

  (void)WriteReg(MPU6050_PWR_MGMT_1, 0x01U);
  (void)WriteReg(MPU6050_PWR_MGMT_2, 0x00U);
  (void)WriteReg(MPU6050_SMPLRT_DIV, 0x07U);
  (void)WriteReg(MPU6050_CONFIG, 0x00U);
  (void)WriteReg(MPU6050_GYRO_CONFIG, 0x18U);
  (void)WriteReg(MPU6050_ACCEL_CONFIG, 0x18U);

  online_ = IsSupportedID(GetID()) ? 1U : 0U;
  return online_ == 1U;
}

/* Return WHO_AM_I register value. */
uint8_t Mpu6050Device::GetID(void)
{
  return ReadReg(MPU6050_WHO_AM_I);
}

/* Read accel/gyro raw data frame (14 bytes) and unpack to int16_t. */
void Mpu6050Device::GetData(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z,
                            int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z)
{
  uint8_t data[14];

  if ((acc_x == 0) || (acc_y == 0) || (acc_z == 0) ||
      (gyro_x == 0) || (gyro_y == 0) || (gyro_z == 0))
  {
    return;
  }

  if ((online_ == 0U) || !ReadRegs(MPU6050_ACCEL_XOUT_H, data, 14U))
  {
    *acc_x = 0;
    *acc_y = 0;
    *acc_z = 0;
    *gyro_x = 0;
    *gyro_y = 0;
    *gyro_z = 0;
    return;
  }

  *acc_x = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  *acc_y = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  *acc_z = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
  *gyro_x = (int16_t)(((uint16_t)data[8] << 8) | data[9]);
  *gyro_y = (int16_t)(((uint16_t)data[10] << 8) | data[11]);
  *gyro_z = (int16_t)(((uint16_t)data[12] << 8) | data[13]);
}

/* Board default MPU6050 singleton on board software-I2C bus. */
Mpu6050Device &BoardMpu6050(void)
{
  static Mpu6050Device mpu(BoardMyI2C(), 0xD0U);
  return mpu;
}

extern "C" {

/* C compatibility wrapper: write one MPU register. */
void MPU6050_WriteReg(uint8_t reg_address, uint8_t data)
{
  (void)BoardMpu6050().WriteReg(reg_address, data);
}

/* C compatibility wrapper: read one MPU register. */
uint8_t MPU6050_ReadReg(uint8_t reg_address)
{
  return BoardMpu6050().ReadReg(reg_address);
}

/* C compatibility wrapper: read a register block. */
void MPU6050_ReadRegs(uint8_t reg_address, uint8_t *data_array, uint8_t count)
{
  (void)BoardMpu6050().ReadRegs(reg_address, data_array, count);
}

/* C compatibility wrapper: initialize MPU device. */
void MPU6050_Init(void)
{
  (void)BoardMpu6050().Init();
}

/* C compatibility wrapper: read WHO_AM_I. */
uint8_t MPU6050_GetID(void)
{
  return BoardMpu6050().GetID();
}

/* C compatibility wrapper: read accel/gyro frame. */
void MPU6050_GetData(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z,
                     int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z)
{
  BoardMpu6050().GetData(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
}

}  // extern "C"
