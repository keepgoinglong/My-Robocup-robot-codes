#include "hc_sr04.h"

#define HC_SR04_TRIG_PORT       GPIOA
#define HC_SR04_TRIG_PIN        GPIO_PIN_2
#define HC_SR04_ECHO_PORT       GPIOA
#define HC_SR04_ECHO_PIN        GPIO_PIN_3
#define HC_SR04_WAIT_LOW_US     2000U
#define HC_SR04_ECHO_TIMEOUT_US 30000U
#define HC_SR04_MIN_MM          20U
#define HC_SR04_MAX_MM          4000U

static uint8_t s_async_state, s_async_ready;
static uint32_t s_async_mark, s_async_pulse;
static uint16_t s_async_distance;
static HC_SR04_Result s_async_result;

static uint32_t CyclesPerUs(void)
{
  uint32_t value = SystemCoreClock / 1000000U;
  return value ? value : 1U;
}

static void DelayUs(uint32_t delay_us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t wait_cycles = delay_us * CyclesPerUs();
  while ((uint32_t)(DWT->CYCCNT - start) < wait_cycles) { }
}

static uint8_t WaitPin(GPIO_PinState state, uint32_t timeout_us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t timeout_cycles = timeout_us * CyclesPerUs();
  while (HAL_GPIO_ReadPin(HC_SR04_ECHO_PORT, HC_SR04_ECHO_PIN) != state) {
    if ((uint32_t)(DWT->CYCCNT - start) >= timeout_cycles) return 0U;
  }
  return 1U;
}

void HC_SR04_Init(void)
{
  GPIO_InitTypeDef gpio;

  gpio.Pin = 0U;
  gpio.Mode = 0U;
  gpio.Pull = 0U;
  gpio.Speed = 0U;
  gpio.Alternate = 0U;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_WritePin(HC_SR04_TRIG_PORT, HC_SR04_TRIG_PIN, GPIO_PIN_RESET);

  gpio.Pin = HC_SR04_TRIG_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HC_SR04_TRIG_PORT, &gpio);

  gpio.Pin = HC_SR04_ECHO_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(HC_SR04_ECHO_PORT, &gpio);

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  s_async_state = 0U; s_async_ready = 0U;
}

uint8_t HC_SR04_Start(void)
{
  if (s_async_state != 0U || s_async_ready) return 0U;
  if (HAL_GPIO_ReadPin(HC_SR04_ECHO_PORT, HC_SR04_ECHO_PIN) != GPIO_PIN_RESET) {
    s_async_result = HC_SR04_ECHO_STUCK_HIGH; s_async_ready = 1U; return 0U;
  }
  HAL_GPIO_WritePin(HC_SR04_TRIG_PORT, HC_SR04_TRIG_PIN, GPIO_PIN_SET);
  s_async_mark = DWT->CYCCNT; s_async_state = 1U;
  return 1U;
}

void HC_SR04_Task(void)
{
  uint32_t elapsed, us, mm;
  if (s_async_state == 0U) return;
  elapsed = DWT->CYCCNT - s_async_mark;
  if (s_async_state == 1U) {
    if (elapsed < 10U * CyclesPerUs()) return;
    HAL_GPIO_WritePin(HC_SR04_TRIG_PORT, HC_SR04_TRIG_PIN, GPIO_PIN_RESET);
    s_async_mark = DWT->CYCCNT; s_async_state = 2U; return;
  }
  if (s_async_state == 2U) {
    if (HAL_GPIO_ReadPin(HC_SR04_ECHO_PORT, HC_SR04_ECHO_PIN) == GPIO_PIN_SET) {
      s_async_mark = DWT->CYCCNT; s_async_state = 3U; return;
    }
    if (elapsed < HC_SR04_ECHO_TIMEOUT_US * CyclesPerUs()) return;
    s_async_result = HC_SR04_RISE_TIMEOUT;
  } else {
    if (HAL_GPIO_ReadPin(HC_SR04_ECHO_PORT, HC_SR04_ECHO_PIN) == GPIO_PIN_SET &&
        elapsed < HC_SR04_ECHO_TIMEOUT_US * CyclesPerUs()) return;
    if (HAL_GPIO_ReadPin(HC_SR04_ECHO_PORT, HC_SR04_ECHO_PIN) == GPIO_PIN_SET)
      s_async_result = HC_SR04_FALL_TIMEOUT;
    else {
      us = elapsed / CyclesPerUs(); mm = (us * 10U + 29U) / 58U;
      s_async_pulse = us; s_async_distance = (uint16_t)mm;
      s_async_result = (mm < HC_SR04_MIN_MM || mm > HC_SR04_MAX_MM) ? HC_SR04_OUT_OF_RANGE : HC_SR04_OK;
    }
  }
  s_async_state = 0U; s_async_ready = 1U;
}

uint8_t HC_SR04_Ready(HC_SR04_Result *result, uint32_t *pulse_us, uint16_t *distance_mm)
{
  if (!s_async_ready) return 0U;
  if (result) *result = s_async_result;
  if (pulse_us) *pulse_us = s_async_pulse;
  if (distance_mm) *distance_mm = s_async_distance;
  s_async_ready = 0U; return 1U;
}

uint8_t HC_SR04_IsBusy(void) { return s_async_state != 0U; }

HC_SR04_Result HC_SR04_Measure(uint32_t *pulse_us, uint16_t *distance_mm)
{
  uint32_t start, elapsed_us, measured_mm;

  if (pulse_us == NULL || distance_mm == NULL) return HC_SR04_OUT_OF_RANGE;
  *pulse_us = 0U;
  *distance_mm = 0U;

  if (!WaitPin(GPIO_PIN_RESET, HC_SR04_WAIT_LOW_US)) return HC_SR04_ECHO_STUCK_HIGH;

  HAL_GPIO_WritePin(HC_SR04_TRIG_PORT, HC_SR04_TRIG_PIN, GPIO_PIN_RESET);
  DelayUs(2U);
  HAL_GPIO_WritePin(HC_SR04_TRIG_PORT, HC_SR04_TRIG_PIN, GPIO_PIN_SET);
  DelayUs(10U);
  HAL_GPIO_WritePin(HC_SR04_TRIG_PORT, HC_SR04_TRIG_PIN, GPIO_PIN_RESET);

  if (!WaitPin(GPIO_PIN_SET, HC_SR04_ECHO_TIMEOUT_US)) return HC_SR04_RISE_TIMEOUT;
  start = DWT->CYCCNT;
  if (!WaitPin(GPIO_PIN_RESET, HC_SR04_ECHO_TIMEOUT_US)) return HC_SR04_FALL_TIMEOUT;

  elapsed_us = (uint32_t)(DWT->CYCCNT - start) / CyclesPerUs();
  measured_mm = (elapsed_us * 10U + 29U) / 58U;
  *pulse_us = elapsed_us;
  *distance_mm = (uint16_t)measured_mm;
  if (measured_mm < HC_SR04_MIN_MM || measured_mm > HC_SR04_MAX_MM)
    return HC_SR04_OUT_OF_RANGE;
  return HC_SR04_OK;
}

const char *HC_SR04_ResultText(HC_SR04_Result result)
{
  switch (result) {
    case HC_SR04_OK: return "OK";
    case HC_SR04_ECHO_STUCK_HIGH: return "ECHO_STUCK_HIGH";
    case HC_SR04_RISE_TIMEOUT: return "NO_ECHO_RISE";
    case HC_SR04_FALL_TIMEOUT: return "ECHO_TOO_LONG";
    case HC_SR04_OUT_OF_RANGE: return "OUT_OF_RANGE";
  }
  return "UNKNOWN";
}
