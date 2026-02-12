#pragma once

#include "serial.h"

namespace io {

class UartPlot {
public:
  explicit UartPlot(UartSerial &serial = BoardSerial())
      : serial_(serial)
  {
  }

  void Init(void)
  {
    serial_.Init();
  }

  void SendAngles(float angle_acc, float angle_gyro, float angle)
  {
    serial_.Printf("plot:%f,%f,%f\n", angle_acc, angle_gyro, angle);
  }

private:
  UartSerial &serial_;
};

}  // namespace io
