#ifndef __STS3215_H
#define __STS3215_H

#include "main.h"

#define STS_BROADCAST_ID 0xFEU

void STS_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef STS_ReadPosition(uint8_t id, uint16_t *position);
HAL_StatusTypeDef STS_TorqueEnable(uint8_t id, uint8_t enable);
HAL_StatusTypeDef STS_SyncWritePosition(const uint8_t *ids,
                                        const uint16_t *positions,
                                        uint8_t count,
                                        uint16_t speed,
                                        uint8_t acc);

#endif
