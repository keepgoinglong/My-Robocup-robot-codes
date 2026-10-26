#ifndef __ROBOT_MOTION_H
#define __ROBOT_MOTION_H
#include "main.h"
#include "robot_config.h"
typedef enum {
  ROBOT_IDLE = 0, ROBOT_ENTERING_STAND, ROBOT_STANDING,
  ROBOT_MARCHING, ROBOT_FAULT, ROBOT_HOLDING
} RobotState;
/* 保留旧调用编号兼容；两种模式均严格使用动作组原始参数及时序。 */
typedef enum { ACTION_REFERENCE = 0, ACTION_SPEED_GUARDED = 1 } ActionProfile;
void Robot_Init(void);
void Robot_ReportSafetyFault(void);
uint8_t Robot_Scan(uint16_t positions[ROBOT_MAX_SERVO_COUNT]);
uint8_t Robot_EnterStand(void);
/* single=1：仅执行第3/4步循环，到第4步落地帧停，不停在离地帧。 */
uint8_t Robot_StartActions(ActionProfile profile, uint8_t single);
void Robot_StopMarch(void);
/* 仅请求在下一个原落地帧进入反馈空闲，不取消已接受的控制。 */
void Robot_RequestFeedbackHold(void);
uint8_t Robot_ResumeFeedbackStep(void);
uint8_t Robot_IsFeedbackPaused(void);
void Robot_StopAtStartPose(void);
uint8_t Robot_RequestControl(uint8_t status, int16_t angle);
uint8_t Robot_GetControlPhase(void);
void Robot_ClearControl(void);
void Robot_CancelControl(void);
void Robot_SetLineAngle(int16_t deci_degrees);
/* 障碍接近：只在原落地边界切换小步幅；angle为横杆相对水平角，单位0.1度。 */
void Robot_SetObstacleApproach(uint8_t active, int16_t angle);
/* 越障请求只在小步幅计划落地后启动，完整执行用户JSON的15帧。 */
uint8_t Robot_RequestObstacleOver(void);
uint8_t Robot_GetObstaclePhase(void); /* 0普通，1小步幅，2越障。 */
void Robot_EmergencyOff(void);
void Robot_Task(void);
RobotState Robot_GetState(void);
const char *Robot_GetLastError(void);
uint8_t Robot_GetFrame(void);
uint32_t Robot_GetFrameDuration(void);
/* 主循环事件接口：成功表示锁存；不是立即打断当前步态。1..6有效。 */
uint8_t Robot_RequestShape(uint8_t shape);
uint8_t Robot_HasStationaryShape(void);
uint8_t Robot_GetActiveShape(void);
uint8_t Robot_GetPendingShape(void);
uint32_t Robot_GetCompletedShapes(void);
uint32_t Robot_GetInterruptedShapes(void);
#endif
