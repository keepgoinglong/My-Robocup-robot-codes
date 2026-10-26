#ifndef SG90_PWM_H
#define SG90_PWM_H

#include "main.h"

/* J1信号脚PA11：TIM1_CH4，50Hz。实机若机械零位有偏差，只改以下三个脉宽。 */
#define SG90_CENTER_US       1500U
#define SG90_LEFT_50_US       944U
#define SG90_RIGHT_50_US     2056U
#define SG90_SHAKE_TOTAL_MS  3000U
#define SG90_SHAKE_HALF_MS    500U
#define SG90_CENTER_SETTLE_MS  400U

uint8_t SG90_Init(void);
void SG90_StartShake(uint32_t now);
uint8_t SG90_Task(uint32_t now);
void SG90_Center(void);
uint8_t SG90_IsActive(void);

#endif
