#pragma once

class IrqLock {
public:
  IrqLock() { __disable_irq(); }
  ~IrqLock() { __enable_irq(); }
private:
  IrqLock(const IrqLock &);
  IrqLock &operator=(const IrqLock &);
};
