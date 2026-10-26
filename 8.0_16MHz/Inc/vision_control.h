#ifndef VISION_CONTROL_H
#define VISION_CONTROL_H

#include "main.h"
void VisionControl_Init(void);
void VisionControl_Task(void);
uint8_t VisionControl_StartWalk(void);
void VisionControl_SetEnabled(uint8_t enabled);
uint8_t VisionControl_IsEnabled(void);
uint8_t VisionControl_GetRequestedAction(void);
uint8_t VisionControl_TurnUnavailable(void);
uint8_t VisionControl_GetFeedbackState(void);
uint8_t VisionControl_GetFeedbackSequence(void);
uint32_t VisionControl_GetAcceptedShapes(void);
uint8_t VisionControl_GetLastAcceptedShape(void);
#endif
