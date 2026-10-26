#ifndef HC_SR04_H
#define HC_SR04_H

#include "main.h"

typedef enum {
  HC_SR04_OK = 0,
  HC_SR04_ECHO_STUCK_HIGH,
  HC_SR04_RISE_TIMEOUT,
  HC_SR04_FALL_TIMEOUT,
  HC_SR04_OUT_OF_RANGE
} HC_SR04_Result;

/* 7.0接线：PA2=TRIG，PA3=ECHO直连。
 * 本控制板主控为STM32F407VET6，原理图确认PA3直达扩展排针；
 * PA3是FT数字输入。此结论不能套用到其他GPIO或其他控制板。 */
void HC_SR04_Init(void);
HC_SR04_Result HC_SR04_Measure(uint32_t *pulse_us, uint16_t *distance_mm);
/* 8.0行走用非阻塞接口：Task每轮调用，Ready只消费一次完成结果。 */
uint8_t HC_SR04_Start(void);
void HC_SR04_Task(void);
uint8_t HC_SR04_Ready(HC_SR04_Result *result, uint32_t *pulse_us, uint16_t *distance_mm);
uint8_t HC_SR04_IsBusy(void);
const char *HC_SR04_ResultText(HC_SR04_Result result);

#endif
