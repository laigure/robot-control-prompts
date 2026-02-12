#pragma once

#include <stdint.h>

#include "OLED.h"

namespace io {

struct OledTelemetry {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
  int32_t left_speed_centi_rps;
  int32_t right_speed_centi_rps;
};

class OledPanel {
public:
  void Init(void)
  {
    BoardOled().Init();
    BoardOled().Clear();
  }

  void ShowBootScreen(void)
  {
    BoardOled().ShowString(0, 0, "Hello", OLED_6X8);
    BoardOled().ShowString(0, 16, "STM32", OLED_8X16);
    BoardOled().Update();
  }

  void ShowError(const char *message)
  {
    BoardOled().Clear();
    BoardOled().ShowString(0, 0, message, OLED_8X16);
    BoardOled().Update();
  }

  void ShowTelemetry(const OledTelemetry &telemetry)
  {
    const int32_t left_centi = telemetry.left_speed_centi_rps;
    const int32_t right_centi = telemetry.right_speed_centi_rps;
    const char left_sign = (left_centi < 0) ? '-' : '+';
    const char right_sign = (right_centi < 0) ? '-' : '+';
    const long left_abs = (long)((left_centi < 0) ? -left_centi : left_centi);
    const long right_abs = (long)((right_centi < 0) ? -right_centi : right_centi);

    BoardOled().Printf(0, 0, OLED_8X16, "%+06d", telemetry.ax);
    BoardOled().Printf(0, 16, OLED_8X16, "%+06d", telemetry.ay);
    BoardOled().Printf(0, 32, OLED_8X16, "%+06d", telemetry.az);
    BoardOled().Printf(64, 0, OLED_8X16, "%+06d", telemetry.gx);
    BoardOled().Printf(64, 16, OLED_8X16, "%+06d", telemetry.gy);
    BoardOled().Printf(64, 32, OLED_8X16, "%+06d", telemetry.gz);
    BoardOled().Printf(0, 48, OLED_8X16, "L:%c%02ld.%02ld", left_sign, left_abs / 100L, left_abs % 100L);
    BoardOled().Printf(64, 48, OLED_8X16, "R:%c%02ld.%02ld", right_sign, right_abs / 100L, right_abs % 100L);
    BoardOled().Update();
  }
};

}  // namespace io
