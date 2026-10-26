#include "vision_uart.h"
#include <string.h>

static UART_HandleTypeDef *s_vision_uart;
static uint8_t s_rx_byte, s_packet[8], s_fill;
static volatile uint8_t s_restart_rx, s_have_packet;
static volatile VisionStatus s_status;
static volatile VisionAngle s_angle, s_control;
static volatile VisionObstacle s_obstacle;
static volatile uint8_t s_control_gate, s_control_pending;
static uint8_t s_last_control_seq, s_have_control_seq;
static uint32_t s_last_control_ms;
static uint8_t s_tx_packet[6];
static volatile uint8_t s_tx_busy;
static volatile VisionShapeEvent s_events[VISION_SHAPE_QUEUE_SIZE];
static volatile uint8_t s_head, s_tail;

static uint32_t Lock(void) { uint32_t key = __get_PRIMASK(); __disable_irq(); return key; }
static void Unlock(uint32_t key) { __set_PRIMASK(key); }

__attribute__((noinline)) uint16_t Vision_CRC16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU, i;
  uint8_t bit;
  for (i = 0U; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (bit = 0U; bit < 8U; ++bit)
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
  }
  return crc; /* CCITT-FALSE：不反射，xorout=0，按包型喂前5/6/4字节。 */
}

static void AcceptPacket(uint32_t received_ms)
{
  uint8_t sequence = s_packet[2], shape = s_packet[4];
  uint8_t duplicate = s_have_packet && sequence == s_status.sequence;
  if (s_have_packet) {
    uint8_t advance = (uint8_t)(sequence - s_status.sequence);
    if (duplicate) ++s_status.duplicate_packets;
    else if (advance < 128U) s_status.estimated_lost += advance - 1U;
    else ++s_status.sequence_jumps; /* 重启/乱序仅统计，不拒收；255→0的advance是1。 */
  }
  ++s_status.valid_packets;
  s_status.last_valid_ms = received_ms;
  s_status.sequence = sequence;
  s_status.action = s_packet[3]; /* action是最新有效持续状态，不累计LEFT/RIGHT次数。 */
  s_status.shape = shape;
  s_have_packet = 1U;
  if (shape != 0U && !duplicate) {
    uint8_t next = (uint8_t)((s_head + 1U) % VISION_SHAPE_QUEUE_SIZE);
    if (next == s_tail) {
      ++s_status.queue_overflows;
      s_status.overflow_latched = 1U; /* 不覆盖旧事件；显式记录溢出，不因视觉问题停步。 */
    } else {
      s_events[s_head].sequence = sequence;
      s_events[s_head].shape = shape;
      s_events[s_head].received_ms = received_ms;
      __DMB();
      s_head = next;
    }
  }
  /* shape=0永远不会清队列；action=0也不会撤销已锁存事件。 */
}

void Vision_GetAngle(VisionAngle *angle)
{
  uint32_t key;
  if (angle == NULL) return;
  key = Lock(); *angle = s_angle; Unlock(key);
}

void Vision_GetObstacle(VisionObstacle *obstacle)
{
  uint32_t key;
  if (obstacle == NULL) return;
  key = Lock(); *obstacle = s_obstacle; Unlock(key);
}

void Vision_FeedByte(uint8_t byte, uint32_t received_ms)
{
  uint8_t length;
  int16_t angle;
  s_packet[s_fill++] = byte;
  /* 两种帧头共用滑窗，CRC失败只移一字节，支持混合粘包和拆包。 */
  while (s_fill != 0U) {
    if (s_packet[0] != 0xAAU || (s_fill >= 2U && s_packet[1] != 0x55U && s_packet[1] != 0x56U && s_packet[1] != 0x58U)) {
      --s_fill; memmove(s_packet, s_packet + 1U, s_fill); continue;
    }
    if (s_fill < 2U) return;
    length = s_packet[1] == 0x55U ? 7U : 8U;
    if (s_fill < length) return;
    angle = (int16_t)((uint16_t)s_packet[4] | ((uint16_t)s_packet[5] << 8));
    if (Vision_CRC16(s_packet, length - 2U) !=
        ((uint16_t)s_packet[length-2U] | ((uint16_t)s_packet[length-1U] << 8))) {
      ++s_status.crc_errors;
    /* 5.3：status4右避边同样携带位置误差；旧版误拒绝非零右纠包。 */
    } else if ((s_packet[1] == 0x55U && (s_packet[3] > 3U || s_packet[4] > 6U)) ||
               (s_packet[1] == 0x56U && (s_packet[3] > 5U || angle < -900 || angle > 900 ||
                                (s_packet[3] != 2U && s_packet[3] != 3U && s_packet[3] != 4U && angle != 0)))) {
      ++s_status.field_errors;
    } else {
      if (s_packet[1] == 0x55U) AcceptPacket(received_ms);
      else if (s_packet[1] == 0x58U) {
        if (s_packet[3] > 1U || angle < -450 || angle > 450 || (!s_packet[3] && angle != 0)) {
          ++s_status.field_errors;
          --s_fill; memmove(s_packet, s_packet + 1U, s_fill); continue;
        }
        s_obstacle.deci_degrees = angle; s_obstacle.present = s_packet[3] & 1U;
        s_obstacle.sequence = s_packet[2]; s_obstacle.received_ms = received_ms;
        ++s_obstacle.valid_packets;
      } else {
        s_angle.deci_degrees = angle;
        s_angle.status = s_packet[3];
        s_angle.sequence = s_packet[2]; s_angle.received_ms = received_ms;
        ++s_angle.valid_packets;
        /* AA56是最新持续方向状态：即使上一条反馈尚未完成，也用新序号覆盖单槽，
         * 避免旧LEFT因BUSY/AA57丢失而永久锁住。 */
        if (s_angle.status >= 1U && s_angle.status <= 4U &&
            !(s_have_control_seq && s_last_control_seq == s_angle.sequence &&
              (uint32_t)(received_ms - s_last_control_ms) < 10000U)) {
          s_control = s_angle;
          s_control_pending = 1U;
          s_last_control_seq = s_angle.sequence; s_last_control_ms = received_ms; s_have_control_seq = 1U;
        }
      }
      s_fill = 0U; return;
    }
    --s_fill; memmove(s_packet, s_packet + 1U, s_fill);
  }
}

void Vision_Init(UART_HandleTypeDef *uart)
{
  uint32_t key = Lock();
  s_vision_uart = uart;
  memset((void *)&s_status, 0, sizeof(s_status));
  memset((void *)&s_angle, 0, sizeof(s_angle));
  memset((void *)&s_obstacle, 0, sizeof(s_obstacle));
  s_fill = 0U; s_have_packet = 0U; s_head = 0U; s_tail = 0U;
  s_control_gate = 0U; s_control_pending = 0U; s_have_control_seq = 0U; s_tx_busy = 0U;
  s_restart_rx = uart != NULL;
  Unlock(key);
  Vision_Service();
}

void Vision_Service(void)
{
  /* 异常重启只操作接收寄存器；无DMA、无轮询等7字节、无HAL_Delay。 */
  uint32_t key = Lock();
  if (s_vision_uart != NULL && s_restart_rx) {
    if (HAL_UART_Receive_IT(s_vision_uart, &s_rx_byte, 1U) == HAL_OK) s_restart_rx = 0U;
  }
  Unlock(key);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
  uint8_t byte;
  if (uart != s_vision_uart) return;
  if (uart->ErrorCode != HAL_UART_ERROR_NONE) {
    s_fill = 0U; /* 带硬件错误的字节不进入协议。交给ErrorCallback恢复接收。 */
    return;
  }
  byte = s_rx_byte;
  s_restart_rx = HAL_UART_Receive_IT(uart, &s_rx_byte, 1U) != HAL_OK;
  Vision_FeedByte(byte, HAL_GetTick());
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (uart != s_vision_uart) return;
  ++s_status.uart_errors;
  s_fill = 0U;
  /* 本通道没有DMA，AbortReceive只有寄存器操作，不等待数据到达。 */
  (void)HAL_UART_AbortReceive(uart);
  __HAL_UART_CLEAR_OREFLAG(uart);
  s_restart_rx = HAL_UART_Receive_IT(uart, &s_rx_byte, 1U) != HAL_OK;
}

void Vision_GetStatus(VisionStatus *status)
{
  uint32_t key, now;
  if (status == NULL) return;
  key = Lock();
  *status = s_status;
  now = HAL_GetTick();
  status->online = s_have_packet && (uint32_t)(now - s_status.last_valid_ms) <= VISION_TIMEOUT_MS;
  status->queued_shapes = (uint8_t)((s_head + VISION_SHAPE_QUEUE_SIZE - s_tail) % VISION_SHAPE_QUEUE_SIZE);
  Unlock(key);
}

uint8_t Vision_GetAction(void)
{
  uint8_t action;
  uint32_t key = Lock();
  action = s_status.action; /* 最新有效状态保持；250ms离线仅诊断，不再合成STOP。 */
  Unlock(key);
  return action;
}

uint8_t Vision_PeekShape(VisionShapeEvent *event)
{
  uint8_t found;
  uint32_t key = Lock();
  found = s_tail != s_head;
  if (found && event != NULL) *event = s_events[s_tail];
  Unlock(key);
  return found;
}

uint8_t Vision_PopShape(VisionShapeEvent *event)
{
  uint8_t found;
  uint32_t key = Lock();
  found = s_tail != s_head;
  if (found) {
    if (event != NULL) *event = s_events[s_tail];
    s_tail = (uint8_t)((s_tail + 1U) % VISION_SHAPE_QUEUE_SIZE);
  }
  Unlock(key);
  return found;
}

void Vision_ClearOverflow(void)
{
  uint32_t key = Lock();
  if ((s_head + 1U) % VISION_SHAPE_QUEUE_SIZE != s_tail) s_status.overflow_latched = 0U;
  Unlock(key);
}

uint8_t Vision_TakeControl(VisionAngle *request)
{
  uint32_t key = Lock();
  uint8_t found = s_control_pending;
  if (found) { *request = s_control; s_control_pending = 0U; }
  Unlock(key); return found;
}
void Vision_SetControlGate(uint8_t enabled)
{
  uint32_t key = Lock();
  s_control_gate = enabled && !s_control_pending;
  Unlock(key);
}
void Vision_DiscardControl(void)
{
  uint32_t key = Lock(); s_control_gate = 0U; s_control_pending = 0U; Unlock(key);
}
/* 视觉串口全双工中断TX，静态缓存直到发送完成；不阻塞步态或RX。 */
uint8_t Vision_SendFeedback(uint8_t sequence, uint8_t state)
{
  uint16_t crc;
  if (s_vision_uart == NULL || s_tx_busy || state > 3U) return 0U;
  s_tx_packet[0] = 0xAAU; s_tx_packet[1] = 0x57U;
  s_tx_packet[2] = sequence; s_tx_packet[3] = state;
  crc = Vision_CRC16(s_tx_packet, 4U);
  s_tx_packet[4] = (uint8_t)crc; s_tx_packet[5] = (uint8_t)(crc >> 8);
  s_tx_busy = 1U;
  if (HAL_UART_Transmit_IT(s_vision_uart, s_tx_packet, 6U) != HAL_OK) {
    s_tx_busy = 0U; return 0U;
  }
  return 1U;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart == s_vision_uart) s_tx_busy = 0U;
}
