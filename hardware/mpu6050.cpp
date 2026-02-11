#include "mpu6050.h"

#include "myi2c.h"
#include "mpu6050_reg.h"

#define MPU6050_ADDRESS 0xD0U

static uint8_t s_mpu_online = 0U;

static uint8_t MPU6050_IsSupportedID(uint8_t id)
{
  return ((id == 0x68U) || (id == 0x69U) || (id == 0x70U) || (id == 0x71U)) ? 1U : 0U;
}

extern "C" {

void MPU6050_WriteReg(uint8_t reg_address, uint8_t data)
{
  MyI2C_Start();
  MyI2C_SendByte(MPU6050_ADDRESS);
  if (MyI2C_ReceiveAck() != 0U)
  {
    MyI2C_Stop();
    return;
  }
  MyI2C_SendByte(reg_address);
  if (MyI2C_ReceiveAck() != 0U)
  {
    MyI2C_Stop();
    return;
  }
  MyI2C_SendByte(data);
  (void)MyI2C_ReceiveAck();
  MyI2C_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t reg_address)
{
  uint8_t data = 0xFFU;

  MyI2C_Start();
  MyI2C_SendByte(MPU6050_ADDRESS);
  if (MyI2C_ReceiveAck() != 0U)
  {
    MyI2C_Stop();
    return 0xFFU;
  }
  MyI2C_SendByte(reg_address);
  if (MyI2C_ReceiveAck() != 0U)
  {
    MyI2C_Stop();
    return 0xFFU;
  }

  MyI2C_Start();
  MyI2C_SendByte((uint8_t)(MPU6050_ADDRESS | 0x01U));
  if (MyI2C_ReceiveAck() != 0U)
  {
    MyI2C_Stop();
    return 0xFFU;
  }
  data = MyI2C_ReceiveByte();
  MyI2C_SendAck(1U);
  MyI2C_Stop();

  return data;
}

void MPU6050_ReadRegs(uint8_t reg_address, uint8_t *data_array, uint8_t count)
{
  uint8_t i;

  if ((data_array == 0) || (count == 0U))
  {
    return;
  }

  MyI2C_Start();
  MyI2C_SendByte(MPU6050_ADDRESS);
  if (MyI2C_ReceiveAck() != 0U)
  {
    MyI2C_Stop();
    return;
  }
  MyI2C_SendByte(reg_address);
  if (MyI2C_ReceiveAck() != 0U)
  {
    MyI2C_Stop();
    return;
  }

  MyI2C_Start();
  MyI2C_SendByte((uint8_t)(MPU6050_ADDRESS | 0x01U));
  if (MyI2C_ReceiveAck() != 0U)
  {
    MyI2C_Stop();
    return;
  }
  for (i = 0U; i < count; i++)
  {
    data_array[i] = MyI2C_ReceiveByte();
    if (i < (uint8_t)(count - 1U))
    {
      MyI2C_SendAck(0U);
    }
    else
    {
      MyI2C_SendAck(1U);
    }
  }
  MyI2C_Stop();
}

void MPU6050_Init(void)
{
  MyI2C_Init();
  s_mpu_online = 0U;
  MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01U);
  MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00U);
  MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09U);
  MPU6050_WriteReg(MPU6050_CONFIG, 0x06U);
  MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18U);
  MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18U);
  s_mpu_online = MPU6050_IsSupportedID(MPU6050_GetID());
}

uint8_t MPU6050_GetID(void)
{
  return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z,
                     int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z)
{
  uint8_t data[14];

  if ((acc_x == 0) || (acc_y == 0) || (acc_z == 0) ||
      (gyro_x == 0) || (gyro_y == 0) || (gyro_z == 0))
  {
    return;
  }
  if (s_mpu_online == 0U)
  {
    *acc_x = 0;
    *acc_y = 0;
    *acc_z = 0;
    *gyro_x = 0;
    *gyro_y = 0;
    *gyro_z = 0;
    return;
  }

  MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, data, 14U);

  *acc_x = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  *acc_y = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  *acc_z = (int16_t)(((uint16_t)data[4] << 8) | data[5]);

  *gyro_x = (int16_t)(((uint16_t)data[8] << 8) | data[9]);
  *gyro_y = (int16_t)(((uint16_t)data[10] << 8) | data[11]);
  *gyro_z = (int16_t)(((uint16_t)data[12] << 8) | data[13]);
}

}  // extern "C"
