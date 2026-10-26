#include "obstacle_control.h"
#include "hc_sr04.h"
#include "vision_uart.h"
#include "robot_motion.h"
#include "robot_config.h"

#define ULTRASONIC_PERIOD_MS       60U
#define VISION_OBSTACLE_MAX_AGE_MS 300U
#define APPROACH_ENTER_MM          190U
#define OVER_MIN_MM                65U
#define OVER_MAX_MM                65U
#define EMERGENCY_OVER_MM          65U
#define ALIGN_LIMIT_DECI_DEG       80

static uint32_t s_next_ping;
static uint16_t s_distance;
static uint8_t s_near_count, s_over_count, s_over_latched;

void ObstacleControl_Init(void)
{
  s_next_ping = HAL_GetTick() + 100U;
  s_distance = 0U;
  s_near_count = 0U; s_over_count = 0U; s_over_latched = 0U;
}

void ObstacleControl_Task(void)
{
  HC_SR04_Result result;
  VisionObstacle obstacle;
  uint32_t now = HAL_GetTick(), pulse;
  uint16_t distance;
  uint8_t vision_fresh;
  HC_SR04_Task();
  Vision_GetObstacle(&obstacle);
  vision_fresh = obstacle.present &&
                 (uint32_t)(now - obstacle.received_ms) <= VISION_OBSTACLE_MAX_AGE_MS;

  /* 正常步态和小碎步期间持续测距；越障动作中不重复触发。 */
  if ((Robot_GetState() != ROBOT_MARCHING && Robot_GetState() != ROBOT_HOLDING) ||
      Robot_GetObstaclePhase() == 2U) {
    (void)HC_SR04_Ready(&result, &pulse, &distance);
    return;
  }

  if (!HC_SR04_IsBusy() && (int32_t)(now - s_next_ping) >= 0) {
    (void)HC_SR04_Start();
    s_next_ping = now + ULTRASONIC_PERIOD_MS;
  }
  if (!HC_SR04_Ready(&result, &pulse, &distance)) return;
  (void)pulse;
  /* 连续确认必须全部有效；无回波或超量程会中断本轮确认。 */
  if (result != HC_SR04_OK) {
    s_near_count = 0U;
    s_over_count = 0U;
    return;
  }
  s_distance = distance;
  if (distance < APPROACH_ENTER_MM) {
    if (s_near_count < OBSTACLE_APPROACH_CONFIRM_SAMPLES) ++s_near_count;
  } else {
    s_near_count = 0U;
    s_over_count = 0U;
    /* 一旦进入小碎步就锁存到越障；超声瞬时变远不恢复大步幅。 */
    if (Robot_GetObstaclePhase() == 0U) s_over_latched = 0U;
  }
  if (s_near_count < OBSTACLE_APPROACH_CONFIRM_SAMPLES) return;

  /* 小于100mm只依赖连续超声；视觉有效时才叠加左右髋朝向修正。 */
  Robot_SetObstacleApproach(1U, vision_fresh ? obstacle.deci_degrees : 0);
  if (s_over_latched) return;
  if ((vision_fresh && distance >= OVER_MIN_MM && distance <= OVER_MAX_MM &&
       obstacle.deci_degrees >= -ALIGN_LIMIT_DECI_DEG &&
       obstacle.deci_degrees <= ALIGN_LIMIT_DECI_DEG) ||
      distance < EMERGENCY_OVER_MM) {
    if (s_over_count < OBSTACLE_OVER_CONFIRM_SAMPLES) ++s_over_count;
  } else {
    s_over_count = 0U;
  }
  if (s_over_count >= OBSTACLE_OVER_CONFIRM_SAMPLES && Robot_RequestObstacleOver()) {
    s_over_latched = 1U;
    s_over_count = 0U;
  }
}

uint16_t ObstacleControl_GetDistance(void) { return s_distance; }
uint8_t ObstacleControl_GetNearCount(void) { return s_near_count; }
