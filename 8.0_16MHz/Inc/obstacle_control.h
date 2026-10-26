#ifndef OBSTACLE_CONTROL_H
#define OBSTACLE_CONTROL_H
#include "main.h"

void ObstacleControl_Init(void);
void ObstacleControl_Task(void);
uint16_t ObstacleControl_GetDistance(void);
uint8_t ObstacleControl_GetNearCount(void);

#endif
