#include "sg90_pwm.h"

static TIM_HandleTypeDef s_tim1;
static uint32_t s_shake_begin, s_center_begin;
static uint8_t s_active, s_segment, s_initialized;

static void SetPulse(uint16_t pulse_us)
{
  if (s_initialized) __HAL_TIM_SET_COMPARE(&s_tim1, TIM_CHANNEL_4, pulse_us);
}

uint8_t SG90_Init(void)
{
  TIM_OC_InitTypeDef config = {0};
  s_tim1.Instance = TIM1;
  /* APB2=84MHz且定时器时钟=168MHz：分频后1MHz，20000计数即50Hz。 */
  s_tim1.Init.Prescaler = 167U;
  s_tim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  s_tim1.Init.Period = 19999U;
  s_tim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  s_tim1.Init.RepetitionCounter = 0U;
  s_tim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&s_tim1) != HAL_OK) return 0U;

  config.OCMode = TIM_OCMODE_PWM1;
  config.Pulse = SG90_CENTER_US;
  config.OCPolarity = TIM_OCPOLARITY_HIGH;
  config.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  config.OCFastMode = TIM_OCFAST_DISABLE;
  config.OCIdleState = TIM_OCIDLESTATE_RESET;
  config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&s_tim1, &config, TIM_CHANNEL_4) != HAL_OK) return 0U;
  s_initialized = 1U;
  SetPulse(SG90_CENTER_US);
  /* 空闲时不输出PWM，避免SG90持续纠正中位而抖动。 */
  s_active = 0U;
  return 1U;
}

void SG90_StartShake(uint32_t now)
{
  s_shake_begin = now;
  s_segment = 0U;
  if (s_initialized) {
    /* 允许j反复测试：先把CH4和HAL通道状态统一复位，再从左端重新启动。 */
    (void)HAL_TIM_PWM_Stop(&s_tim1, TIM_CHANNEL_4);
    __HAL_TIM_SET_COUNTER(&s_tim1, 0U);
    SetPulse(SG90_LEFT_50_US);
    if (HAL_TIM_PWM_Start(&s_tim1, TIM_CHANNEL_4) != HAL_OK) {
      s_active = 0U;
      return;
    }
  }
  s_active = 1U;
}

uint8_t SG90_Task(uint32_t now)
{
  uint32_t elapsed, segment;
  if (!s_active) return 0U;
  if (s_active == 2U) {
    if ((uint32_t)(now - s_center_begin) < SG90_CENTER_SETTLE_MS) return 1U;
    /* 中位到达后继续输出1500us，避免关闭PWM后停在偏位。 */
    s_active = 0U;
    return 0U;
  }
  elapsed = now - s_shake_begin;
  if (elapsed >= SG90_SHAKE_TOTAL_MS) {
    SetPulse(SG90_CENTER_US);
    s_center_begin = now;
    s_active = 2U;
    return 1U;
  }
  segment = elapsed / SG90_SHAKE_HALF_MS;
  if (segment != s_segment) {
    s_segment = (uint8_t)segment;
    SetPulse((segment & 1U) ? SG90_RIGHT_50_US : SG90_LEFT_50_US);
  }
  return 1U;
}

void SG90_Center(void)
{
  if (s_initialized) {
    (void)HAL_TIM_PWM_Stop(&s_tim1, TIM_CHANNEL_4);
    __HAL_TIM_SET_COUNTER(&s_tim1, 0U);
    SetPulse(SG90_CENTER_US);
    (void)HAL_TIM_PWM_Start(&s_tim1, TIM_CHANNEL_4);
  }
  s_active = 0U;
}

uint8_t SG90_IsActive(void)
{
  return s_active;
}
