#include "sts3215.h"
#include "robot_config.h"

#define STS_INST_READ        0x02U
#define STS_INST_WRITE       0x03U
#define STS_INST_SYNC_WRITE  0x83U
#define STS_ADDR_TORQUE      40U
#define STS_ADDR_ACC         41U
#define STS_ADDR_POSITION    56U

static UART_HandleTypeDef *s_uart;

static void STS_SetTransmit(void)
{
#if ROBOT_USE_EXTERNAL_ADAPTER == 0
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, SERVO_RX_DISABLE_LEVEL);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, SERVO_TX_ENABLE_LEVEL);
#endif
}

static void STS_SetReceive(void)
{
#if ROBOT_USE_EXTERNAL_ADAPTER == 0
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, SERVO_TX_DISABLE_LEVEL);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, SERVO_RX_ENABLE_LEVEL);
#endif
}

static uint8_t STS_Checksum(const uint8_t *data, uint16_t start, uint16_t end)
{
  uint16_t i;
  uint8_t sum = 0U;
  for (i = start; i < end; ++i) sum = (uint8_t)(sum + data[i]);
  return (uint8_t)(~sum);
}

static HAL_StatusTypeDef STS_Send(uint8_t *packet, uint16_t length)
{
  HAL_StatusTypeDef status;
  if (s_uart == NULL) return HAL_ERROR;
  __HAL_UART_CLEAR_OREFLAG(s_uart);
  while (__HAL_UART_GET_FLAG(s_uart, UART_FLAG_RXNE) != RESET) {
    volatile uint32_t discard = s_uart->Instance->DR;
    (void)discard;
  }
  STS_SetTransmit();
  status = HAL_UART_Transmit(s_uart, packet, length, 20U);
  if (status == HAL_OK) {
    while (__HAL_UART_GET_FLAG(s_uart, UART_FLAG_TC) == RESET) { }
  }
  STS_SetReceive();
  return status;
}

void STS_Init(UART_HandleTypeDef *huart)
{
  s_uart = huart;
  STS_SetReceive();
}

HAL_StatusTypeDef STS_TorqueEnable(uint8_t id, uint8_t enable)
{
  uint8_t p[8];
  p[0] = 0xFFU; p[1] = 0xFFU; p[2] = id; p[3] = 4U;
  p[4] = STS_INST_WRITE; p[5] = STS_ADDR_TORQUE; p[6] = enable ? 1U : 0U;
  p[7] = STS_Checksum(p, 2U, 7U);
  return STS_Send(p, sizeof(p));
}

HAL_StatusTypeDef STS_ReadPosition(uint8_t id, uint16_t *position)
{
  uint8_t tx[8];
  uint8_t rx[8];
  HAL_StatusTypeDef status;
  if (position == NULL) return HAL_ERROR;
  tx[0] = 0xFFU; tx[1] = 0xFFU; tx[2] = id; tx[3] = 4U;
  tx[4] = STS_INST_READ; tx[5] = STS_ADDR_POSITION; tx[6] = 2U;
  tx[7] = STS_Checksum(tx, 2U, 7U);
  status = STS_Send(tx, sizeof(tx));
  if (status != HAL_OK) return status;
  status = HAL_UART_Receive(s_uart, rx, sizeof(rx), 8U);
  if (status != HAL_OK) return status;
  if (rx[0] != 0xFFU || rx[1] != 0xFFU || rx[2] != id || rx[3] != 4U) return HAL_ERROR;
  if (STS_Checksum(rx, 2U, 7U) != rx[7]) return HAL_ERROR;
  if (rx[4] != 0U) return HAL_ERROR;
  *position = (uint16_t)((uint16_t)rx[5] | ((uint16_t)rx[6] << 8));
  *position &= 0x0FFFU;
  return HAL_OK;
}

HAL_StatusTypeDef STS_SyncWritePosition(const uint8_t *ids,
                                        const uint16_t *positions,
                                        uint8_t count,
                                        uint16_t speed,
                                        uint8_t acc)
{
  uint8_t packet[8U + ROBOT_MAX_SERVO_COUNT * 8U];
  uint16_t i;
  uint16_t n = 0U;
  if (ids == NULL || positions == NULL || count == 0U || count > ROBOT_MAX_SERVO_COUNT) return HAL_ERROR;
  /* 与fddebug Bus.write相同：拒绝越界值，不夹紧或改写。 */
  if (speed < 1U || speed > 32766U || acc > 254U) return HAL_ERROR;
  for (i = 0U; i < count; ++i) if (positions[i] > 4095U) return HAL_ERROR;
  packet[n++] = 0xFFU; packet[n++] = 0xFFU; packet[n++] = STS_BROADCAST_ID;
  packet[n++] = (uint8_t)(4U + count * 8U);
  packet[n++] = STS_INST_SYNC_WRITE;
  packet[n++] = STS_ADDR_ACC;
  packet[n++] = 7U;
  for (i = 0U; i < count; ++i) {
    packet[n++] = ids[i];
    packet[n++] = acc;
    packet[n++] = (uint8_t)(positions[i] & 0xFFU);
    packet[n++] = (uint8_t)(positions[i] >> 8);
    packet[n++] = 0U; packet[n++] = 0U;
    packet[n++] = (uint8_t)(speed & 0xFFU);
    packet[n++] = (uint8_t)(speed >> 8);
  }
  packet[n] = STS_Checksum(packet, 2U, n);
  ++n;
  return STS_Send(packet, n);
}
