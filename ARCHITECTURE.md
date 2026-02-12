# C++ Architecture (Embedded)

Application logic is organized into four C++ layers:

- `io/`: hardware adapters (IMU, OLED, UART plotting)
- `tasks/`: algorithm modules (attitude estimator)
- `tools/`: utility helpers (IRQ lock)
- `src/`: application orchestration (`BalanceCarApp`) and C bridge API

`Core/Src/main.c` is kept as a CubeMX-safe bootstrap only, and forwards control to C++:

- `AppCpp_Init()`
- `AppCpp_Loop()`
- `AppCpp_OnTimPeriodElapsed()`

This keeps HAL/CubeMX files stable while running app logic in C++ classes.
