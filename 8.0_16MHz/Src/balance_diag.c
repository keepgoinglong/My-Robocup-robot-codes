#include "balance_diag.h"
#include "robot_motion.h"
#include "sts3215.h"
#include <string.h>

#define BALANCE_DIAG_COUNTS  40 /* 诊断幅度约3.5度；只用于判断骨盆前后方向。 */
#define BALANCE_DIAG_SPEED    60 /* 使用较低速度，便于扶稳机器人观察。 */
#define BALANCE_DIAG_ACC       4 /* 使用较低加速度，避免突然晃动。 */

static const uint8_t k_diag_ids[2] = {8U, 10U};
static uint16_t s_saved_position[2];
static uint8_t s_saved_valid;
static char s_error[64];

static uint16_t ClampRawPosition(int32_t value)
{
  if (value < 0) return 0U;
  if (value > 4095) return 4095U;
  return (uint16_t)value;
}

uint8_t BalanceDiag_Start(uint8_t pattern)
{
  int16_t id8_delta = 0;
  int16_t id10_delta = 0;
  uint16_t target[2];

  if (Robot_GetState() != ROBOT_STANDING) {
    strcpy(s_error, "enter stable stand first");
    return 0U;
  }
  /* 同一轮诊断始终以第一次保存的位置为基准，反复发送a/b不会继续累加到机械限位。 */
  if (s_saved_valid == 0U) {
    if (STS_ReadPosition(8U, &s_saved_position[0]) != HAL_OK ||
        STS_ReadPosition(10U, &s_saved_position[1]) != HAL_OK) {
      strcpy(s_error, "cannot read ID8/ID10");
      return 0U;
    }
  }

  switch (pattern) {
    case 0U: /* A：当前假设，左右髋前后镜像联动。 */
      id8_delta = -BALANCE_DIAG_COUNTS;
      id10_delta = BALANCE_DIAG_COUNTS;
      break;
    case 1U: /* B：与A完全相反。 */
      id8_delta = BALANCE_DIAG_COUNTS;
      id10_delta = -BALANCE_DIAG_COUNTS;
      break;
    case 2U: /* D：只测试左髋前后ID8。 */
      id8_delta = -BALANCE_DIAG_COUNTS;
      break;
    case 3U: /* E：只测试右髋前后ID10。 */
      id10_delta = BALANCE_DIAG_COUNTS;
      break;
    default:
      strcpy(s_error, "unknown diagnostic pattern");
      return 0U;
  }

  target[0] = ClampRawPosition((int32_t)s_saved_position[0] + id8_delta);
  target[1] = ClampRawPosition((int32_t)s_saved_position[1] + id10_delta);
  if (STS_SyncWritePosition(k_diag_ids, target, 2U,
                            BALANCE_DIAG_SPEED, BALANCE_DIAG_ACC) != HAL_OK) {
    strcpy(s_error, "servo bus transmit failed");
    return 0U;
  }
  s_saved_valid = 1U;
  strcpy(s_error, "ok");
  return 1U;
}

uint8_t BalanceDiag_Return(void)
{
  if (s_saved_valid == 0U) {
    strcpy(s_error, "no diagnostic pose saved");
    return 0U;
  }
  if (STS_SyncWritePosition(k_diag_ids, s_saved_position, 2U,
                            BALANCE_DIAG_SPEED, BALANCE_DIAG_ACC) != HAL_OK) {
    strcpy(s_error, "servo bus transmit failed");
    return 0U;
  }
  s_saved_valid = 0U;
  strcpy(s_error, "ok");
  return 1U;
}

void BalanceDiag_Cancel(void)
{
  s_saved_valid = 0U;
}

const char *BalanceDiag_GetLastError(void)
{
  return s_error;
}
