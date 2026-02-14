#include "led.h"
#include "main.h"

/* Bind one LED GPIO pin as a controllable LED device. */
LedDevice::LedDevice(GPIO_TypeDef *port, uint16_t pin)
    : port_(port), pin_(pin)
{
}

/* Initialize LED output default state (off). */
void LedDevice::Init(void)
{
  /* PC13 board LED is active-low on most STM32F103 boards. */
  HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
}

/* Drive LED pin high. */
void LedDevice::On(void)
{
  HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
}

/* Drive LED pin low. */
void LedDevice::Off(void)
{
  HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
}

/* Toggle LED pin output level. */
void LedDevice::Toggle(void)
{
  HAL_GPIO_TogglePin(port_, pin_);
}

/* Legacy blocking blink loop (kept only for compatibility). */
void LedDevice::BlinkLoop(uint32_t delay_ms)
{
  for (;;)
  {
    Toggle();
    HAL_Delay(delay_ms);
  }
}

/* Construct flow runner with an LED channel table. */
LedFlowRunner::LedFlowRunner(const LedChannel *channels, uint8_t count)
    : channels_(channels), count_(count), index_(0U), elapsed_ms_(0U), period_ms_(120U),
      enable_(true), forward_(true), blink_state_(false)
{
}

/* Reset flow runner state and apply initial output pattern. */
void LedFlowRunner::Init(void)
{
  index_ = 0U;
  elapsed_ms_ = 0U;
  period_ms_ = 120U;
  enable_ = true;
  forward_ = true;
  blink_state_ = false;
  ApplyState();
}

/* Update flow period with range clamp to keep behavior stable. */
void LedFlowRunner::SetPeriodMs(uint16_t period_ms)
{
  if (period_ms < 10U)
  {
    period_ms = 10U;
  }
  if (period_ms > 2000U)
  {
    period_ms = 2000U;
  }
  period_ms_ = period_ms;
}

/* Enable/disable output and refresh physical LED state immediately. */
void LedFlowRunner::SetEnable(bool enable)
{
  enable_ = enable;
  ApplyState();
}

/* Set forward/backward direction for multi-channel flow. */
void LedFlowRunner::SetForward(bool forward)
{
  forward_ = forward;
}

/* Current direction flag. */
bool LedFlowRunner::IsForward(void) const
{
  return forward_;
}

/* Current enable flag. */
bool LedFlowRunner::IsEnabled(void) const
{
  return enable_;
}

/* Current flow period in ms. */
uint16_t LedFlowRunner::PeriodMs(void) const
{
  return period_ms_;
}

/* 1 ms scheduler tick:
 * accumulate elapsed time and advance sequence when period is reached.
 */
void LedFlowRunner::Tick1ms(void)
{
  if (!enable_)
  {
    return;
  }

  elapsed_ms_++;
  if (elapsed_ms_ >= period_ms_)
  {
    elapsed_ms_ = 0U;
    Advance();
  }
}

/* Advance output sequence by one step. */
void LedFlowRunner::Advance(void)
{
  if (count_ == 0U)
  {
    return;
  }

  if (count_ == 1U)
  {
    blink_state_ = !blink_state_;
  }
  else if (forward_)
  {
    index_++;
    if (index_ >= count_)
    {
      index_ = 0U;
    }
  }
  else
  {
    if (index_ == 0U)
    {
      index_ = (uint8_t)(count_ - 1U);
    }
    else
    {
      index_--;
    }
  }

  ApplyState();
}

/* Apply logical sequence state to all physical LED channels. */
void LedFlowRunner::ApplyState(void)
{
  uint8_t i;
  for (i = 0U; i < count_; i++)
  {
    GPIO_PinState state = GPIO_PIN_RESET;
    if (enable_)
    {
      if (count_ == 1U)
      {
        state = blink_state_ ? GPIO_PIN_SET : GPIO_PIN_RESET;
      }
      else
      {
        state = (i == index_) ? GPIO_PIN_SET : GPIO_PIN_RESET;
      }
    }
    HAL_GPIO_WritePin(channels_[i].port, channels_[i].pin, state);
  }
}

namespace {
/* Single-board LED device for compatibility APIs. */
LedDevice g_led(LED_GPIO_Port, LED_Pin);

/* Default flow channels:
 * - PC13 board LED only.
 * PB12~PB15 are used by motor direction control, so they must not be toggled here.
 */
const LedChannel k_flow_channels[] = {
    {LED_GPIO_Port, LED_Pin},
};
LedFlowRunner g_flow(k_flow_channels, (uint8_t)(sizeof(k_flow_channels) / sizeof(k_flow_channels[0])));
}

/* Accessor for board LED singleton. */
LedDevice &BoardLed(void)
{
  return g_led;
}

/* Accessor for board flow-runner singleton. */
LedFlowRunner &BoardLedFlow(void)
{
  return g_flow;
}

extern "C" {

/* C compatibility wrapper: initialize single LED output. */
void Led_Init(void)
{
  g_led.Init();
}

/* C compatibility wrapper: set LED on. */
void Led_On(void)
{
  g_led.On();
}

/* C compatibility wrapper: set LED off. */
void Led_Off(void)
{
  g_led.Off();
}

/* C compatibility wrapper: toggle LED state. */
void Led_Toggle(void)
{
  g_led.Toggle();
}

/* C compatibility wrapper: blocking blink loop (legacy). */
void Led_BlinkLoop(uint32_t delay_ms)
{
  g_led.BlinkLoop(delay_ms);
}

/* C compatibility wrapper: initialize flow runner. */
void LedFlow_Init(void)
{
  g_flow.Init();
}

/* C compatibility wrapper: 1 ms flow runner tick. */
void LedFlow_Tick(void)
{
  g_flow.Tick1ms();
}

/* C compatibility wrapper: set flow period in ms. */
void LedFlow_SetPeriodMs(uint16_t period_ms)
{
  g_flow.SetPeriodMs(period_ms);
}

/* C compatibility wrapper: enable/disable flow output. */
void LedFlow_SetEnable(uint8_t enable)
{
  g_flow.SetEnable(enable != 0U);
}

/* C compatibility wrapper: set flow direction flag. */
void LedFlow_SetForward(uint8_t forward)
{
  g_flow.SetForward(forward != 0U);
}

}  // extern "C"
