#include "vision_control.h"
#include "vision_uart.h"
#include "robot_motion.h"

static uint8_t s_enabled, s_requested_action, s_forward_latched;
static uint32_t s_retry_after;
static uint32_t s_accepted_shapes;
static uint8_t s_last_accepted_shape;
/* AA57: 0 READY, 1 BUSY, 2 DONE, 3 FAULT。CRC/包格式保持不变。 */
static uint8_t s_feedback_state, s_feedback_seq, s_done_sent, s_was_settled;
static uint8_t s_link_seen, s_was_online, s_handshake_sent, s_busy_sent, s_fault_sent;
static uint32_t s_last_sequence_jumps;
static uint32_t s_feedback_due;
volatile uint32_t g_vision_shape6_received;
volatile uint8_t g_vision_last_shape_sequence;

static uint8_t IsSettled(void)
{
  RobotState state = Robot_GetState();
  return (state == ROBOT_STANDING || state == ROBOT_HOLDING) && !Robot_GetActiveShape();
}

void VisionControl_Init(void)
{
  s_enabled = 1U; s_requested_action = 0U; s_forward_latched = 0U;
  s_retry_after = HAL_GetTick(); s_feedback_due = HAL_GetTick();
  s_feedback_state = 0U; s_feedback_seq = 0U; s_done_sent = 0U;
  s_was_settled = 0U;
  s_link_seen = 0U; s_was_online = 0U; s_handshake_sent = 0U; s_busy_sent = 0U; s_fault_sent = 0U;
  s_last_sequence_jumps = 0U;
  s_accepted_shapes = 0U; s_last_accepted_shape = 0U;
  g_vision_shape6_received = 0U; g_vision_last_shape_sequence = 0U;
}

void VisionControl_SetEnabled(uint8_t enabled)
{
  s_enabled = enabled != 0U;
  s_forward_latched = 0U; s_retry_after = HAL_GetTick();
  Vision_DiscardControl();
  if (!s_enabled) {
    /* 人工停止是取消，不伪造DONE，也不当作动作故障。 */
    Robot_StopMarch();
  } else {
    Robot_CancelControl(); Vision_ClearOverflow();
  }
  Robot_ClearControl();
  s_feedback_state = Robot_GetState() == ROBOT_FAULT ? 3U : 0U;
  s_done_sent = 0U; s_was_settled = 0U;
  s_feedback_due = HAL_GetTick();
}

uint8_t VisionControl_StartWalk(void)
{
  if (!Robot_StartActions(ACTION_REFERENCE, 0U)) return 0U;
  VisionControl_SetEnabled(1U); s_forward_latched = 1U;
  return 1U;
}
uint8_t VisionControl_IsEnabled(void) { return s_enabled; }
uint8_t VisionControl_GetRequestedAction(void) { return s_requested_action; }
uint8_t VisionControl_TurnUnavailable(void) { return 0U; }
uint8_t VisionControl_GetFeedbackState(void) { return s_feedback_state; }
uint8_t VisionControl_GetFeedbackSequence(void) { return s_feedback_seq; }

static void FeedbackTask(uint32_t now)
{
  RobotState state = Robot_GetState();
  uint8_t settled = IsSettled();
  uint8_t can_accept = state == ROBOT_MARCHING || state == ROBOT_STANDING || state == ROBOT_HOLDING;
  if (settled && !s_was_settled) {
    /* 丢线/WAIT不重放旧直走；落地后的新包才可开始下一动作。 */
    s_forward_latched = 0U;
    s_feedback_due = now;
  }
  s_was_settled = settled;
  if (state == ROBOT_FAULT) {
    if (s_feedback_state != 3U) { s_feedback_due = now; s_fault_sent = 0U; }
    s_feedback_state = 3U; s_forward_latched = 0U;
  } else if (s_feedback_state == 3U && settled) {
    /* 真故障经人工恢复到落地空闲后解除，不在错误状态直接发READY。 */
    Robot_ClearControl(); s_feedback_state = 0U; s_feedback_due = now;
  } else if (s_feedback_state == 1U) {
    if (Robot_GetControlPhase() == 4U) {
      /* DONE只表示已按计划时序执行完这一步，不等待舵机机械位置精准到位。 */
      s_feedback_state = 2U; s_done_sent = 0U; s_feedback_due = now;
    } else if (Robot_GetControlPhase() == 5U || state == ROBOT_IDLE) {
      /* 取消/主动卸力不等于真实FAULT；不发未完成的DONE。 */
      Robot_ClearControl(); s_feedback_state = 0U; s_feedback_due = now;
    }
  }
  /* 连续行走时也必须接收下一条AA56；运动层会等到12/16步界再应用，
   * 因此不会在单脚支撑的半步中途突然修改目标。 */
#if TEMP_FORCE_CONTINUOUS_LEFT
  /* 临时连续左转期间不锁存AA56，避免控制事务在帧11/15进入反馈HOLD。 */
  Vision_SetControlGate(0U);
#else
  Vision_SetControlGate(s_enabled && can_accept && s_feedback_state == 0U);
#endif
  /* READY在空闲接收窗口每200ms重发，避免单个反馈包丢失后视觉端永久WAIT。 */
  /* READY是通信握手，不要求机器人先停下；否则连续行走取消自动HOLD后，
   * 启用反馈的视觉端会永远停在status=5 WAIT_BOARD。 */
  if (s_feedback_state == 0U && s_link_seen && can_accept &&
      (!s_handshake_sent || (int32_t)(now - s_feedback_due) >= 0) &&
      Vision_SendFeedback(0U, 0U)) {
    s_handshake_sent = 1U; s_feedback_due = now + 200U;
  } else if (s_feedback_state == 1U && !s_busy_sent && Vision_SendFeedback(s_feedback_seq, 1U)) {
    s_busy_sent = 1U;
  } else if (s_feedback_state == 2U && (int32_t)(now - s_feedback_due) >= 0 &&
             Vision_SendFeedback(s_feedback_seq, 2U)) {
    s_feedback_due = now + 50U;
    if (++s_done_sent == 3U) {
      Robot_ClearControl(); s_feedback_state = 0U;
      s_busy_sent = 0U;
    }
  } else if (s_feedback_state == 3U && !s_fault_sent && Vision_SendFeedback(s_feedback_seq, 3U)) {
    s_fault_sent = 1U;
  }
}

void VisionControl_Task(void)
{
  VisionShapeEvent event;
  VisionAngle request;
  VisionStatus status;
  uint32_t now = HAL_GetTick();
  Vision_Service();
  Vision_GetStatus(&status);
  if (status.online && !s_was_online && s_link_seen) s_handshake_sent = 0U;
  s_was_online = status.online;
  if (status.valid_packets != 0U) s_link_seen = 1U;
  if (status.sequence_jumps != s_last_sequence_jumps) {
    /* K230D重启/序号回跳后允许重新握手；正常255->0不会计为jump。 */
    s_last_sequence_jumps = status.sequence_jumps;
    s_handshake_sent = 0U;
  }
  s_requested_action = Vision_GetAction(); /* AA55独立图形；AA56唯一运动决策。 */
  if (s_enabled && Robot_GetState() != ROBOT_FAULT) {
    if (Vision_PeekShape(&event) && Robot_RequestShape(event.shape)) {
      (void)Vision_PopShape(&event);
      ++s_accepted_shapes; s_last_accepted_shape = event.shape;
      if (event.shape == 6U) ++g_vision_shape6_received;
      g_vision_last_shape_sequence = event.sequence;
    }
    if (Vision_TakeControl(&request)) {
      if (Robot_RequestControl(request.status, request.deci_degrees)) {
        /* 方向状态始终更新；只有反馈空闲时才新建一条辅助BUSY/DONE事务。 */
        if (s_feedback_state == 0U) {
          s_feedback_seq = request.sequence; s_feedback_state = 1U;
          s_forward_latched = 1U; s_feedback_due = now; s_busy_sent = 0U;
        }
      }
      /* 普通忙碌/拒收不映射FAULT；真正故障统一由ROBOT_FAULT判断。 */
    }
    if (s_forward_latched && IsSettled() && !Robot_HasStationaryShape() &&
        (int32_t)(now - s_retry_after) >= 0) {
      uint8_t started = Robot_IsFeedbackPaused() ? Robot_ResumeFeedbackStep() : Robot_StartActions(ACTION_REFERENCE, 0U);
      if (!started) {
        s_retry_after = now + 250U;
        /* 扫描/实际姿态校验失败是安全故障，交给运动状态机明确记录。 */
        Robot_ReportSafetyFault();
      } else s_forward_latched = 0U;
    }
    /* 没有已接受的AA56控制事务时保持连续行走；视觉短时丢失或action=0
     * 不得在帧11/15自动停靠。真正的控制事务仍由运动层在完成帧置反馈停靠。 */
  }
  FeedbackTask(now);
}

uint32_t VisionControl_GetAcceptedShapes(void) { return s_accepted_shapes; }
uint8_t VisionControl_GetLastAcceptedShape(void) { return s_last_accepted_shape; }
