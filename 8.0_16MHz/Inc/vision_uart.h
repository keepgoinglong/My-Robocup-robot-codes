#ifndef VISION_UART_H
#define VISION_UART_H
#include "main.h"

#define VISION_TIMEOUT_MS 250U
#define VISION_SHAPE_QUEUE_SIZE 64U /* 环形队列可保存63个待处理事件。 */
typedef struct { uint8_t sequence, shape; uint32_t received_ms; } VisionShapeEvent;
typedef struct {
  uint32_t valid_packets, crc_errors, field_errors, duplicate_packets;
  uint32_t estimated_lost, sequence_jumps, uart_errors, queue_overflows;
  uint32_t last_valid_ms;
  uint8_t sequence, action, shape, online, queued_shapes, overflow_latched;
} VisionStatus;

/* AA56: seq/status/int16小端0.1度/CRC16前6字节；与AA55序号独立。 */
typedef struct {
  int16_t deci_degrees; /* 当前包角度，status=2直走微调或status=3左转调幅有效。 */
  uint8_t status, sequence;
  uint32_t received_ms, valid_packets;
} VisionAngle;
/* AA58: seq/flags/int16横杆角度(0.1度)/CRC；flags bit0=稳定障碍。 */
typedef struct {
  int16_t deci_degrees;
  uint8_t present, sequence;
  uint32_t received_ms, valid_packets;
} VisionObstacle;
void Vision_GetObstacle(VisionObstacle *obstacle);
void Vision_GetAngle(VisionAngle *angle);
/* 单槽请求锁存：WAIT/无效包不覆盖，忙时不排队转向。 */
uint8_t Vision_TakeControl(VisionAngle *request);
void Vision_SetControlGate(uint8_t enabled);
void Vision_DiscardControl(void);
uint8_t Vision_SendFeedback(uint8_t sequence, uint8_t state);
void Vision_Init(UART_HandleTypeDef *uart);
void Vision_Service(void);
uint16_t Vision_CRC16(const uint8_t *data, uint16_t length);
/* 仅接收中断调用；独立入口也便于离线协议测试。主循环不重复喂同一包。 */
void Vision_FeedByte(uint8_t byte, uint32_t received_ms);
uint8_t Vision_GetAction(void); /* 最新有效action，不因离线变成0。 */
void Vision_GetStatus(VisionStatus *status);
uint8_t Vision_PeekShape(VisionShapeEvent *event);
uint8_t Vision_PopShape(VisionShapeEvent *event);
void Vision_ClearOverflow(void);
#endif
