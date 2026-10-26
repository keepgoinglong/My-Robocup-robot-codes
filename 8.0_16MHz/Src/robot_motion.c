#include "robot_motion.h"
#include "sts3215.h"
#include "action_frames.h"
#include "right_turn_frames.h"
#include "obstacle_action_frames.h"
#include "shape_frames.h"
#include "sg90_pwm.h"
#include <string.h>
#include <stdio.h>

static const uint8_t k_ids[ROBOT_SERVO_COUNT] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14};
static RobotState s_state;
static const char *s_error;
static char s_pose_error[128];
static uint16_t s_now[ROBOT_SERVO_COUNT], s_from[ROBOT_SERVO_COUNT];
static uint8_t s_frame, s_resume, s_stop, s_single, s_stage, s_check_id;
static uint32_t s_begin, s_duration, s_stage_begin, s_last_update;
static uint16_t s_joint_pending;
static uint32_t s_feedback_poll_due;
static const uint32_t *s_active_delay, *s_active_joint_duration;
static uint8_t s_obstacle_small, s_obstacle_small_requested;
static uint8_t s_obstacle_over_request, s_obstacle_over_active, s_obstacle_index;
static int16_t s_obstacle_hip_correction;
/* 保留原步态状态机；事件仅在已有帧入口/落地出口插入，不另建一套行走。 */
static ActionFrame s_motion;
static uint8_t s_use_frame_timing, s_default_left_frame, s_start_pose_stop, s_feedback_hold, s_feedback_paused;
static uint8_t s_cycle_direction;
/* 一次控制事务：0空闲，1排定，2执行一步，4指令序列完成，5拒绝/取消。 */
static uint8_t s_control_phase, s_control_kind, s_control_end;
static int16_t s_requested_correction, s_step_correction;
static uint8_t s_pending_shape, s_active_shape, s_shape_phase, s_shape_return_hold;
static uint8_t s_shape_check_id, s_shape_stable_frame, s_direct_start;
static uint32_t s_shape_prepare_begin, s_completed_shapes, s_interrupted_shapes;
/* shape_phase: 0普通，1举手，2收手，3原前摆帧附加抬脚停留，4落地反馈检查，5摇头。 */
/* stage 0运动，1等待反馈到位，2停留。只有站姿原帧要求反馈到位。 */

static uint8_t Fail(const char *reason)
{
  s_error = reason;
  s_state = ROBOT_FAULT;
  if (s_control_phase >= 1U && s_control_phase <= 3U) s_control_phase = 5U;
  /* 保留最后目标和扭矩，避免通信/反馈故障自动卸力；x可主动紧急卸力。 */
  return 0U;
}

void Robot_ReportSafetyFault(void)
{
  (void)Fail(s_error); /* 保留真实拒绝原因：读取失败或人工起步姿态误差。 */
}

static uint8_t InLimits(uint8_t i, uint16_t position)
{
  return position >= k_action_limits[i][0] && position <= k_action_limits[i][1];
}

static uint8_t InMotionLimits(uint8_t i, uint16_t position)
{
  if (s_obstacle_over_active)
    return position >= k_obstacle_limits[i][0] && position <= k_obstacle_limits[i][1];
  return InLimits(i, position);
}

/* 与fddebug一致：延迟期间不发该ID，终点发送后不重复写该ID。 */
static HAL_StatusTypeDef SendPending(uint32_t elapsed)
{
  uint8_t ids[ROBOT_SERVO_COUNT], count = 0U, i;
  uint16_t positions[ROBOT_SERVO_COUNT], finished = 0U;
  for (i = 0U; i < ROBOT_SERVO_COUNT; ++i) {
    uint32_t delay = 0U, duration = s_duration;
    uint16_t bit = (uint16_t)(1U << i);
    if (!(s_joint_pending & bit)) continue;
    if (s_use_frame_timing) {
      delay = s_active_delay[i];
      duration = s_active_joint_duration[i];
    }
    if (elapsed < delay) continue;
    ids[count] = k_ids[i]; positions[count++] = s_now[i];
    if (elapsed >= delay + duration) finished |= bit;
  }
  if (count && STS_SyncWritePosition(ids, positions, count, s_motion.speed, s_motion.acc) != HAL_OK)
    return HAL_ERROR;
  s_joint_pending &= (uint16_t)~finished;
  return HAL_OK;
}

/* 对齐fddebug的绝对帧时钟：迟到跳过旧采样，不逐次累积偏移；
 * 单舵机延迟/终点即使不在10ms网格上，也在主循环首次经过时处理。 */
static uint8_t MotionSampleDue(uint32_t now)
{
  uint32_t elapsed = now - s_begin, previous = s_last_update - s_begin;
  uint8_t i;
  if (elapsed >= s_duration ||
      elapsed / ROBOT_CONTROL_PERIOD_MS > previous / ROBOT_CONTROL_PERIOD_MS) return 1U;
  if (s_use_frame_timing) {
    for (i = 0U; i < ROBOT_SERVO_COUNT; ++i) {
      uint32_t delay, end;
      if (!(s_joint_pending & (uint16_t)(1U << i))) continue;
      delay = s_active_delay[i];
      end = delay + s_active_joint_duration[i];
      if ((previous < delay && elapsed >= delay) || (previous < end && elapsed >= end)) return 1U;
    }
  }
  return 0U;
}

static uint8_t BeginMotion(uint32_t now)
{
  uint8_t i;
  for (i = 0U; i < ROBOT_SERVO_COUNT; ++i) {
    if (!InMotionLimits(i, s_now[i]) || !InMotionLimits(i, s_motion.position[i]))
      return Fail("action/start outside JSON limits; no clipping");
  }
  /* 速度、ACC及运动时间全部取自动作组，不做速度预算或延时补偿。 */
  s_duration = s_motion.duration_ms;
  memcpy(s_from, s_now, sizeof(s_now));
  s_begin = now;
  s_last_update = now;
  s_stage = 0U;
  s_joint_pending = (uint16_t)((1U << ROBOT_SERVO_COUNT) - 1U);
  if (s_direct_start) {
    /* 仅首次第5->第6帧模拟舵机调试软件：最终目标只下发一次，由舵机内部完成运动。 */
    s_direct_start = 0U;
    if (STS_SyncWritePosition(k_ids, s_motion.position, ROBOT_SERVO_COUNT,
                             s_motion.speed, s_motion.acc) != HAL_OK)
      return Fail("direct start transmit failed; last target retained");
    memcpy(s_now, s_motion.position, sizeof(s_now));
    s_stage = 3U; /* 等待原duration后再进入原hold，不每10ms重写目标。 */
    return 1U;
  }
  /* fddebug的零时长关节在启动时刻直接发送终点；其他关节沿用起点。 */
  for (i = 0U; i < ROBOT_SERVO_COUNT; ++i) {
    if ((s_use_frame_timing && k_action_delay[s_frame][i] == 0U &&
         k_action_joint_duration[s_frame][i] == 0U) || (!s_use_frame_timing && s_duration == 0U))
      s_now[i] = s_motion.position[i];
  }
  /* 与调试软件一样在t=0设置本帧ACC/速度。 */
  if (SendPending(0U) != HAL_OK)
    return Fail("servo transmit failed; last target retained");
  return 1U;
}

/* 数组下标1=左髋ID2，下标2=右髋ID3；保持当前步态的髋选择顺序。 */
static int16_t LeftTurnHipBase(uint8_t hip)
{
  return hip == 1U ? LEFT_TURN_LEFT_HIP_BASE_OFFSET : LEFT_TURN_RIGHT_HIP_BASE_OFFSET;
}

static uint8_t BeginFrame(uint8_t frame, uint32_t now)
{
  uint8_t hip, small_index = 0U, right_index = 0U, use_small = 0U, use_right = 0U;
  if (frame >= ACTION_FRAME_COUNT) return Fail("invalid action frame");
  s_cycle_direction = 1U;
  /* 在计划落地后接入，直走和转向均为3帧一步：13,14,15或17,10,11。
   * 12/16仅作为入口编号，不发送动作；不从单脚支撑中间插入请求。 */
  if (s_control_phase == 1U && (frame == 12U || frame == 16U) &&
      !s_pending_shape && !s_active_shape && !s_stop) {
    s_control_phase = 2U;
    s_control_end = frame == 12U ? 15U : 11U;
    s_step_correction = s_requested_correction;
  }
  /* 所有行走均使用新直走6帧，仅修改髋目标，不引入旧腿部动作和时序。 */
  if (frame == 12U || frame == 16U) ++frame;
  s_frame = frame; s_use_frame_timing = 1U; s_default_left_frame = 0U;
  s_motion = k_action_frames[frame];
  s_active_delay = k_action_delay[frame];
  s_active_joint_duration = k_action_joint_duration[frame];
  /* 右转完整参照my_actionsfinalTURNRIGHT；不同角度只改当前支撑侧髋旋转。 */
  if (!s_obstacle_small &&
      (s_control_kind == 4U || (s_control_kind == 2U && s_requested_correction < 0))) {
    if (frame == 10U) { right_index = 0U; use_right = 1U; }
    else if (frame == 11U) { right_index = 1U; use_right = 1U; }
    else if (frame == 13U) { right_index = 2U; use_right = 1U; }
    else if (frame == 14U) { right_index = 3U; use_right = 1U; }
    else if (frame == 15U) { right_index = 4U; use_right = 1U; }
    else if (frame == 17U) { right_index = 5U; use_right = 1U; }
    if (use_right) {
      s_motion = k_right_turn_frames[right_index];
      s_active_delay = k_right_turn_delay[right_index];
      s_active_joint_duration = k_right_turn_joint_duration[right_index];
    }
  }
  if (s_obstacle_small && frame >= ACTION_LOOP_FIRST) {
    if (frame == 10U) { small_index = 0U; use_small = 1U; }
    else if (frame == 11U) { small_index = 1U; use_small = 1U; }
    else if (frame == 13U) { small_index = 2U; use_small = 1U; }
    else if (frame == 14U) { small_index = 3U; use_small = 1U; }
    else if (frame == 15U) { small_index = 4U; use_small = 1U; }
    else if (frame == 17U) { small_index = 5U; use_small = 1U; }
    if (use_small) {
      s_motion = k_small_frames[small_index];
      s_active_delay = k_small_delay[small_index];
      s_active_joint_duration = k_small_joint_duration[small_index];
      /* 正角向右纠：当前支撑侧髋减小；负角向左纠：当前支撑侧髋增大。 */
      hip = small_index < 3U ? 2U : 1U;
      s_motion.position[hip] = (uint16_t)((int32_t)s_motion.position[hip] + s_obstacle_hip_correction);
    }
  }
  /* 只优化正常直走前摆停留，不压缩运动或换重心/落地时间。
   * 图形在下方覆盖整帧，保留其原始保持时间。 */
  if ((frame == 10U || frame == 14U) && s_control_kind <= 1U &&
      !TEMP_FORCE_CONTINUOUS_LEFT && s_motion.hold_ms >= WALK_SWING_HOLD_TRIM_MS) {
    s_motion.hold_ms -= WALK_SWING_HOLD_TRIM_MS;
  }
  /* AA56是持续方向状态：最近一次1/2/3/4持续作用于后续每个步态帧，
   * 不因单步反馈事务结束而自动恢复直走。 */
  if (s_control_kind >= 1U && s_control_kind <= 4U && frame >= ACTION_LOOP_FIRST) {
    hip = frame < 14U ? 2U : 1U; /* 用户确认左右髋互换：原ID2阶段用ID3，原ID3阶段用ID2。 */
    if (s_control_kind == 3U) {
      /* 两侧基准独立，s_requested_correction仅保存共同的视觉微调量。 */
      s_motion.position[hip] = (uint16_t)(2048 + LeftTurnHipBase(hip) + s_requested_correction);
      s_default_left_frame = 1U;
    } else if (use_right) {
      /* 基准文件是-100计数；实时角度只替换ID2/ID3，其余位置与时序不动。 */
      s_motion.position[hip] = (uint16_t)(2048 + s_requested_correction);
    } else {
      /* 直走左右微调/右避边只叠加髋偏移，不再叠加旧转向腿部差值。 */
      s_motion.position[hip] = (uint16_t)((int32_t)s_motion.position[hip] + s_requested_correction);
    }
  }
#if TEMP_FORCE_CONTINUOUS_LEFT
  /* 临时连续左转：覆盖直走/视觉修正，只改当前支撑侧髋，保留JSON全部其余参数。 */
  if (frame >= ACTION_LOOP_FIRST) {
    hip = frame < 14U ? 2U : 1U; /* 临时左转也沿用交换后的髋映射。 */
    s_motion.position[hip] = (uint16_t)(2048 + LeftTurnHipBase(hip));
    s_default_left_frame = 1U;
  }
#endif

  /* 图形一次性事件独立锁存；纠偏一步完成后才允许启动图形，不打断三秒保持。 */
  if (!s_stop && s_control_phase != 2U && (s_pending_shape == 3U || s_pending_shape == 4U) &&
      frame == k_shape_anchor[s_pending_shape - 1U]) {
    s_active_shape = s_pending_shape; s_pending_shape = 0U;
    /* 图形只覆盖表演姿态/保持时间，不覆盖本帧已经计算的左右髋转向。 */
    uint16_t left_hip = s_motion.position[1], right_hip = s_motion.position[2];
    s_shape_phase = 3U; s_motion = k_shape_frames[s_active_shape - 1U];
    s_motion.position[1] = left_hip; s_motion.position[2] = right_hip;
  }
  return BeginMotion(now);
}

static uint8_t BeginObstacleFrame(uint8_t index, uint32_t now)
{
  if (index >= OVER_FRAME_COUNT) return Fail("invalid obstacle frame");
  s_obstacle_over_active = 1U; s_obstacle_index = index;
  s_motion = k_over_frames[index];
  s_use_frame_timing = 1U;
  s_active_delay = k_over_delay[index];
  s_active_joint_duration = k_over_joint_duration[index];
  return BeginMotion(now);
}

void Robot_SetObstacleApproach(uint8_t active, int16_t angle)
{
  int32_t correction;
  /* 小碎步一旦请求/生效就锁存到越障，禁止距离抖动切回大步幅。 */
  if (!active) {
    if (!s_obstacle_small_requested && !s_obstacle_small) s_obstacle_hip_correction = 0;
    return;
  }
  if (angle < -900) angle = -900; else if (angle > 900) angle = 900;
  /* 横杆接近水平时保留原小碎步，不做左右髋修正。 */
  if (angle >= -OBSTACLE_ALIGN_DEADBAND_DECI_DEG &&
      angle <= OBSTACLE_ALIGN_DEADBAND_DECI_DEG) {
    s_obstacle_hip_correction = 0;
    s_obstacle_small_requested = 1U;
    return;
  }
  correction = -(int32_t)angle * OBSTACLE_HIP_COUNTS_PER_DEGREE;
  /* angle单位为0.1度；X10=15表示视觉角先除以1.5，并对称四舍五入。 */
  correction = correction >= 0 ?
      (correction + OBSTACLE_ANGLE_DIVISOR_X10 / 2) / OBSTACLE_ANGLE_DIVISOR_X10 :
      -((-correction + OBSTACLE_ANGLE_DIVISOR_X10 / 2) / OBSTACLE_ANGLE_DIVISOR_X10);
  if (correction > OBSTACLE_HIP_MAX_COUNTS) correction = OBSTACLE_HIP_MAX_COUNTS;
  if (correction < -OBSTACLE_HIP_MAX_COUNTS) correction = -OBSTACLE_HIP_MAX_COUNTS;
  s_obstacle_hip_correction = (int16_t)correction;
  /* 不在半步中途换动作表；先走完当前动作组的落地->离地，再切换。 */
  s_obstacle_small_requested = 1U;
}

uint8_t Robot_RequestObstacleOver(void)
{
  if (s_obstacle_over_active || s_obstacle_over_request ||
      (s_state != ROBOT_MARCHING && s_state != ROBOT_HOLDING)) return 0U;
  s_obstacle_over_request = 1U; s_obstacle_small_requested = 1U;
  return 1U;
}

uint8_t Robot_GetObstaclePhase(void)
{
  return s_obstacle_over_active ? 2U : s_obstacle_small ? 1U : 0U;
}

/* AA56正角要求右纠（减位置），负角要求左纠（加位置）。
 * 输入单位0.1度：4096/360*(deci_degrees/10/LINE_ANGLE_DIVISOR)，四舍五入。
 * 始终相对直走原目标，绝不把多次接收累加成更大的旋转。 */
void Robot_SetLineAngle(int16_t deci_degrees)
{
  int32_t scaled;
  const int32_t denominator = 3600L * LINE_ANGLE_DIVISOR;
  if (deci_degrees < -900 || deci_degrees > 900) return;
  scaled = -(int32_t)deci_degrees * 4096;
  s_requested_correction = (int16_t)(scaled >= 0 ? (scaled + denominator / 2) / denominator :
                                                          -((-scaled + denominator / 2) / denominator));
}

uint8_t Robot_RequestControl(uint8_t status, int16_t angle)
{
  int32_t adjust;
  if (status < 1U || status > 4U ||
      (s_state != ROBOT_MARCHING && s_state != ROBOT_STANDING && s_state != ROBOT_HOLDING) || s_stop) return 0U;
  if (angle < -900 || angle > 900) return 0U;
  s_control_kind = status;
  if (status == 1U) s_requested_correction = 0; /* 直走也回传同序号BUSY/DONE。 */
  else if (status == 2U) Robot_SetLineAngle(angle);
  else if (status == 3U) {
    /* AA56左弯角先按配置缩小；基准髋目标保持不变，只减弱视觉附加量。 */
    adjust = -(int32_t)angle * LEFT_ADJUST_COUNTS_PER_DEGREE;
    adjust = adjust >= 0 ?
      (adjust + 5 * LEFT_CURVE_ANGLE_DIVISOR) / (10 * LEFT_CURVE_ANGLE_DIVISOR) :
      -((-adjust + 5 * LEFT_CURVE_ANGLE_DIVISOR) / (10 * LEFT_CURVE_ANGLE_DIVISOR));
    if (adjust > LEFT_ADJUST_MAX_COUNTS) adjust = LEFT_ADJUST_MAX_COUNTS;
    if (adjust < -LEFT_ADJUST_MAX_COUNTS) adjust = -LEFT_ADJUST_MAX_COUNTS;
    s_requested_correction = (int16_t)adjust;
  } else {
    /* status=4已经明确要求向右修正；视觉实机可能下发负偏角，
     * 因此取角度绝对值增强修正，避免-5~-17度始终只用基础力度。 */
    adjust = angle >= 0 ? (int32_t)angle : -(int32_t)angle;
    adjust = (adjust * RIGHT_RECOVER_COUNTS_PER_DEGREE + 5) / 10;
    adjust += RIGHT_RECOVER_BASE_COUNTS;
    if (adjust > RIGHT_RECOVER_MAX_COUNTS) adjust = RIGHT_RECOVER_MAX_COUNTS;
    s_requested_correction = (int16_t)-adjust;
  }
  /* 忙时立即更新持续方向，但不重启当前反馈事务；空闲时再建立新事务。 */
  if (s_control_phase == 0U) s_control_phase = 1U;
  return 1U;
}
uint8_t Robot_GetControlPhase(void) { return s_control_phase; }
void Robot_ClearControl(void)
{
  if (s_control_phase == 4U || s_control_phase == 5U) s_control_phase = 0U;
  /* 只清反馈事务阶段；保留最近AA56方向，直到视觉发送新的状态。 */
}
void Robot_CancelControl(void)
{
  if (s_control_phase >= 1U && s_control_phase <= 3U) s_control_phase = 5U;
  s_requested_correction = 0;
}

uint8_t Robot_RequestShape(uint8_t shape)
{
  if (shape < 1U || shape > 6U || s_pending_shape != 0U) return 0U;
  s_pending_shape = shape;
  return 1U;
}

uint8_t Robot_HasStationaryShape(void)
{
  return s_state == ROBOT_HOLDING &&
         s_frame == ACTION_STEP3_STABLE &&
         s_cycle_direction == 1U &&
         (s_pending_shape == 1U || s_pending_shape == 2U || s_pending_shape == 5U);
}

static void PrepareArmShape(uint32_t now)
{
  s_active_shape = s_pending_shape;
  s_pending_shape = 0U;
  s_shape_phase = 4U;
  s_shape_stable_frame = s_frame;
  s_shape_return_hold = s_state == ROBOT_HOLDING || s_stop || s_single;
  s_shape_check_id = 4U; /* ID5..14确认保持当前第3/6帧稳定腿姿，手臂此时不改变。 */
  s_shape_prepare_begin = now;
  s_last_update = now;
  s_state = ROBOT_MARCHING;
}

static void CheckArmLanding(uint32_t now)
{
  if (now - s_shape_prepare_begin > ACTION_REACHED_TIMEOUT_MS) {
    (void)Fail("shape landing feedback timeout"); return;
  }
  if (s_shape_check_id < ROBOT_SERVO_COUNT) {
    uint16_t measured;
    int32_t error;
    if (STS_ReadPosition(k_ids[s_shape_check_id], &measured) != HAL_OK) {
      (void)Fail("shape landing feedback read failed"); return;
    }
    error = (int32_t)measured - k_action_frames[s_shape_stable_frame].position[s_shape_check_id];
    if (error < -(int32_t)ACTION_REACHED_TOLERANCE || error > (int32_t)ACTION_REACHED_TOLERANCE) s_shape_check_id = 4U;
    else ++s_shape_check_id;
  }
  /* 反馈只确认关节到位，不冒充足底触地检测。最少100ms给落脚过渡。 */
  if (s_shape_check_id == ROBOT_SERVO_COUNT && now - s_shape_prepare_begin >= 100U) {
    s_motion = k_shape_frames[s_active_shape - 1U];
    /* 抬手时保持刚落地的髋目标，不回到图形模板的直走2048。 */
    s_motion.position[1] = s_now[1]; s_motion.position[2] = s_now[2];
    memcpy(&s_motion.position[4], &k_action_frames[s_shape_stable_frame].position[4],
           (ROBOT_SERVO_COUNT - 4U) * sizeof(s_motion.position[0]));
    s_use_frame_timing = 0U;
    s_shape_phase = 1U;
    (void)BeginMotion(HAL_GetTick());
  }
}

static uint16_t Interpolate(uint16_t a, uint16_t b, uint32_t elapsed, uint32_t duration)
{
  double x, q, value, fraction;
  uint16_t rounded;
  if (elapsed >= duration || duration == 0U) return b;
  if (elapsed == 0U) return a;
  x = (double)elapsed / duration;
  q = x*x*x*(10.0+x*(-15.0+6.0*x));
  value = a + ((double)b - a)*q;
  /* 与fddebug的Python round一致：最近整数，恰好半整数时取偶数。 */
  rounded = (uint16_t)value;
  fraction = value - rounded;
  if (fraction > 0.5 || (fraction == 0.5 && (rounded & 1U))) ++rounded;
  return rounded;
}

void Robot_Init(void)
{
  s_state = ROBOT_IDLE; s_start_pose_stop = 0U; s_feedback_hold = 0U; s_feedback_paused = 0U;
  s_use_frame_timing = 0U; s_default_left_frame = 0U;
  s_active_delay = k_action_delay[0]; s_active_joint_duration = k_action_joint_duration[0];
  s_obstacle_small = 0U; s_obstacle_small_requested = 0U;
  s_obstacle_over_request = 0U; s_obstacle_over_active = 0U;
  s_obstacle_index = 0U; s_obstacle_hip_correction = 0;
  s_requested_correction = 0; s_step_correction = 0; s_cycle_direction = 1U;
  s_control_phase = 0U; s_control_kind = 0U;
  
  s_error = "none";
  s_frame = 0U; s_resume = ACTION_WALK_START_NEXT;
  s_stop = 0U; s_single = 0U; s_stage = 0U;
  s_duration = 0U; s_last_update = HAL_GetTick();
  s_pending_shape = 0U; s_active_shape = 0U; s_shape_phase = 0U;
  s_shape_stable_frame = ACTION_STEP4_STABLE; s_direct_start = 0U;
  s_completed_shapes = 0U; s_interrupted_shapes = 0U;
  memcpy(s_now, k_action_frames[0].position, sizeof(s_now));
}

uint8_t Robot_Scan(uint16_t positions[ROBOT_MAX_SERVO_COUNT])
{
  uint8_t i;
  /* 防止14次阻塞读取和串口打印打断步态调度。 */
  if (s_state == ROBOT_MARCHING || s_state == ROBOT_ENTERING_STAND) {
    s_error = "stop and hold before scan"; return 0U;
  }
  for (i = 0U; i < ROBOT_SERVO_COUNT; ++i) {
    uint16_t p;
    if (STS_ReadPosition(k_ids[i], &p) != HAL_OK) { s_error = "missing servo/read timeout"; return 0U; }
    if (positions != NULL) positions[i] = p;
  }
  s_error = "none";
  return 1U;
}

uint8_t Robot_EnterStand(void)
{
  uint8_t i;
  uint16_t measured[ROBOT_SERVO_COUNT];
  if (s_state == ROBOT_MARCHING || s_state == ROBOT_ENTERING_STAND) {
    s_error = "h to land and hold before s"; return 0U;
  }
  if (!Robot_Scan(measured)) return 0U;
  for (i = 0U; i < ROBOT_SERVO_COUNT; ++i) {
    if (!InLimits(i, measured[i])) { s_error = "start outside JSON limits"; return 0U; }
  }
  memcpy(s_now, measured, sizeof(s_now));
  s_stop = 0U; s_start_pose_stop = 0U; s_single = 0U; s_resume = ACTION_WALK_START_NEXT;
  
  /* 先预装当前反馈位置，再开扭矩，防止重新使能时追逐旧目标。 */
  s_active_shape = 0U; s_shape_phase = 0U;
  if (!BeginFrame(0U, HAL_GetTick())) return 0U;
  if (STS_TorqueEnable(STS_BROADCAST_ID, 1U) != HAL_OK) return Fail("torque enable transmit failed");
  s_state = ROBOT_ENTERING_STAND;
  s_error = "none";
  return 1U;
}

uint8_t Robot_StartActions(ActionProfile profile, uint8_t single)
{
  uint8_t i, next;
  uint16_t measured[ROBOT_SERVO_COUNT];
  if (profile != ACTION_REFERENCE && profile != ACTION_SPEED_GUARDED) { s_error = "unknown profile"; return 0U; }
  if (s_state != ROBOT_STANDING && s_state != ROBOT_HOLDING) { s_error = "s and wait for stand first"; return 0U; }
  if (!Robot_Scan(measured)) return 0U;
  /* s完成后允许舵机存在少量静差；明显被抱动时仍拒绝启动。 */
  for (i = 0U; i < ROBOT_SERVO_COUNT; ++i) {
    int32_t error = (int32_t)measured[i] - s_now[i];
    if (error < -(int32_t)ACTION_START_TOLERANCE || error > (int32_t)ACTION_START_TOLERANCE) {
      sprintf(s_pose_error,
               "held pose not reached: ID%u actual=%u target=%u diff=%ld; support robot and use s",
               (unsigned)k_ids[i], (unsigned)measured[i], (unsigned)s_now[i], (long)error);
      s_error = s_pose_error; return 0U;
    }
  }
  /* 从实测姿态开始插值，避免把允许的静差当作已经到达目标。 */
  memcpy(s_now, measured, sizeof(s_now));
  /* 起始动作是原第6帧（左脚离地）；首次行走从第1帧继续，落地锚点仍为第2/5帧。 */
  next = s_state == ROBOT_STANDING ? ACTION_WALK_START_NEXT : s_resume;
  s_single = single; s_stop = 0U; s_feedback_hold = 0U; s_feedback_paused = 0U;
  s_direct_start = 0U; /* fddebug首次起步也执行五次插值，不直接跳终点。 */
  if (!BeginFrame(next, HAL_GetTick())) return 0U;
  s_state = ROBOT_MARCHING; s_error = "none";
  return 1U;
}

uint8_t Robot_ResumeFeedbackStep(void)
{
  /* 反馈停靠没有卸力/重新定位，继续同一条轨迹；不重扫14只舵机。
   * 仅此路径可用，人工停止和首次起步仍走原StartActions检查。 */
  if (s_state != ROBOT_HOLDING || !s_feedback_paused || s_stop || s_active_shape) return 0U;
  s_feedback_paused = 0U;
  s_state = ROBOT_MARCHING;
  return BeginFrame(s_resume, HAL_GetTick());
}
uint8_t Robot_IsFeedbackPaused(void) { return s_feedback_paused; }

void Robot_RequestFeedbackHold(void)
{
  if (s_state == ROBOT_MARCHING) s_feedback_hold = 1U;
}

void Robot_StopMarch(void)
{
  s_feedback_hold = 0U; s_feedback_paused = 0U;
  s_start_pose_stop = 0U;
  Robot_CancelControl();
  if (s_state == ROBOT_MARCHING) s_stop = 1U;
}

void Robot_StopAtStartPose(void)
{
  Robot_StopMarch();
  if (s_state == ROBOT_MARCHING) s_start_pose_stop = 1U;
}

/* s最终停在指定左前落地姿态；转向残留髋偏移用起步过渡收回。 */
static void FinishStop(uint8_t next, uint32_t now)
{
  s_feedback_paused = s_feedback_hold && !s_stop && !s_single;
  s_resume = next; s_stop = 0U; s_feedback_hold = 0U;
  if (s_start_pose_stop) {
    s_start_pose_stop = 0U; s_resume = ACTION_WALK_START_NEXT;
    if (memcmp(s_now, k_action_frames[0].position, sizeof(s_now)) != 0) {
      if (BeginFrame(0U, now)) s_state = ROBOT_ENTERING_STAND;
    } else { s_frame = ACTION_WALK_START_FRAME; s_state = ROBOT_STANDING; }
  } else s_state = ROBOT_HOLDING;
}

void Robot_EmergencyOff(void)
{
  Robot_CancelControl();
  SG90_Center();
  if (STS_TorqueEnable(STS_BROADCAST_ID, 0U) != HAL_OK) { (void)Fail("emergency OFF transmit failed"); return; }
  s_state = ROBOT_IDLE; s_stop = 0U; s_resume = ACTION_WALK_START_NEXT;
  if (s_active_shape != 0U) ++s_interrupted_shapes;
  s_active_shape = 0U; s_shape_phase = 0U;
  s_obstacle_small = 0U; s_obstacle_small_requested = 0U;
  s_obstacle_over_request = 0U; s_obstacle_over_active = 0U;
  s_error = "emergency torque off sent";
}

void Robot_Task(void)
{
  uint32_t now = HAL_GetTick(), elapsed;
  const ActionFrame *f;
  uint8_t i, next;
  /* 图形6只在右脚前摆后的落地稳定帧11停住；PA11摇头三秒后回中续走。 */
  if (s_shape_phase == 5U) {
    s_last_update = now;
    /* SG90_Task统一由main主循环调用，与手动j走完全相同的PWM更新路径。 */
    if (SG90_IsActive()) return;
    ++s_completed_shapes;
    s_active_shape = 0U; s_shape_phase = 0U;
    next = s_frame + 1U;
    if (next == ACTION_FRAME_COUNT) next = ACTION_LOOP_FIRST;
    if (s_feedback_hold || (s_stop && (!s_start_pose_stop || s_frame == ACTION_STEP4_STABLE)) || (s_single && s_frame == ACTION_FINAL_LANDING)) {
      FinishStop(next, now); return;
    }
    (void)BeginFrame(next, now);
    return;
  }
  if (Robot_HasStationaryShape()) PrepareArmShape(now);
  if (s_state != ROBOT_ENTERING_STAND && s_state != ROBOT_MARCHING) return;
  if ((uint32_t)(now - s_last_update) > ACTION_SCHEDULER_TIMEOUT_MS) { (void)Fail("scheduler delayed >250ms; held last target"); return; }
  /* 运动按帧绝对时间及关节边界调度；停留仍使用JSON原始时长。 */
  if (s_stage == 0U) {
    if (!MotionSampleDue(now)) return;
  } else if ((uint32_t)(now - s_last_update) < ROBOT_CONTROL_PERIOD_MS &&
             !(s_stage == 2U && (uint32_t)(now - s_stage_begin) >= s_motion.hold_ms)) return;
  s_last_update = now;
  if (s_shape_phase == 4U) { CheckArmLanding(now); return; }
  f = &s_motion;
  if (s_stage == 3U) {
    if ((uint32_t)(now - s_begin) < s_duration) return;
    s_stage = 2U;
    s_stage_begin = now;
  }
  if (s_stage == 0U) {
    elapsed = now - s_begin;
    for (i = 0U; i < ROBOT_SERVO_COUNT; ++i) {
      uint32_t delay = 0U, joint_duration = s_duration;
      if (s_use_frame_timing) {
        delay = s_active_delay[i];
        joint_duration = s_active_joint_duration[i];
      }
      s_now[i] = Interpolate(s_from[i], f->position[i], elapsed > delay ? elapsed - delay : 0U, joint_duration);
    }
    if (SendPending(elapsed) != HAL_OK) {
      (void)Fail("servo transmit failed; last target retained"); return;
    }
    if (elapsed < s_duration) return;
    s_stage = f->wait_reached ? 1U : 2U;
    s_stage_begin = HAL_GetTick();
    s_check_id = 0U;
    s_feedback_poll_due = s_stage_begin;
  }
  if (s_stage == 1U) {
    uint16_t measured;
    uint8_t reached = 1U;
    int32_t error;
    /* 与fddebug一致：每轮读取全部目标，误差<=20；未到位50ms后重试。 */
    if ((uint32_t)(HAL_GetTick() - s_feedback_poll_due) >= 0x80000000U) return;
    for (i = 0U; i < ROBOT_SERVO_COUNT; ++i) {
      if (STS_ReadPosition(k_ids[i], &measured) != HAL_OK) { (void)Fail("stand feedback read failed"); return; }
      error = (int32_t)measured - f->position[i];
      if (error < -(int32_t)ACTION_REACHED_TOLERANCE || error > (int32_t)ACTION_REACHED_TOLERANCE) reached = 0U;
    }
    if (!reached) {
      now = HAL_GetTick();
      if (now - s_stage_begin >= ACTION_REACHED_TIMEOUT_MS) { (void)Fail("stand feedback timeout; check load/power"); return; }
      s_feedback_poll_due = now + 50U;
      return;
    }
    s_stage = 2U;
    s_stage_begin = HAL_GetTick();
  }
  /* 停留从最后端点包发完/到位确认后开始；停留期间不反复重发位置。 */
  now = HAL_GetTick();
  /* 只有显式手动停止可提前结束单脚等待；视觉表演丢线由上层屏蔽，保持完整三秒。 */
  if (!(s_shape_phase == 3U && s_stop) && now - s_stage_begin < f->hold_ms) return;
  if (s_shape_phase == 1U) {
    s_motion = k_shape_frames[s_active_shape - 1U];
    /* 三秒后双臂回到当前第2/5帧，腿部始终不离开该稳定姿态。 */
    memcpy(s_motion.position, k_action_frames[s_shape_stable_frame].position,
           sizeof(s_motion.position));
    /* 收手同样保持转向髋姿态；下一步由最新有效巡线指令决定。 */
    s_motion.position[1] = s_now[1]; s_motion.position[2] = s_now[2];
    s_motion.hold_ms = 0U;
    s_shape_phase = 2U;
    (void)BeginMotion(now); /* 两腿不动，手臂收回后再继续行走。 */
    return;
  }
  if (s_shape_phase == 2U || s_shape_phase == 3U) {
    if (s_shape_phase == 3U && s_stop && now - s_stage_begin < f->hold_ms) ++s_interrupted_shapes;
    else ++s_completed_shapes;
    if (s_shape_phase == 2U && s_shape_return_hold) s_stop = 1U;
    s_active_shape = 0U; s_shape_phase = 0U;
  }
  if (s_frame == 0U) {
    /* 帧0仅为进入站姿的过渡副本；其目标与my_actionsfinalorigin2第6帧完全相同。 */
    s_frame = ACTION_WALK_START_FRAME;
    s_state = ROBOT_STANDING;
    return;
  }
  if (s_obstacle_over_active) {
    if (++s_obstacle_index < OVER_FRAME_COUNT) {
      (void)BeginObstacleFrame(s_obstacle_index, now);
    } else {
      /* 完整越障动作结束后回到普通巡线；不重复触发本次障碍。 */
      s_obstacle_over_active = 0U; s_obstacle_over_request = 0U;
      s_obstacle_small = 0U; s_obstacle_small_requested = 0U;
      s_obstacle_hip_correction = 0;
      /* 新越障第16帧已经完成左脚前摆；下一帧直接接正常“左脚落地”。 */
      (void)BeginFrame(ACTION_STEP4_STABLE, now);
    }
    return;
  }
  if (s_control_phase == 2U && s_frame == s_control_end && f->landing) {
    /* 本步原运动/停留结束后结束反馈事务，但连续方向状态继续作用；
     * AA57仅辅助，不能让机器人停下或自动恢复直走。 */
    s_control_phase = 4U;
  }
  if (s_frame == ACTION_STEP3_STABLE &&
      s_cycle_direction == 1U &&
      (s_pending_shape == 1U || s_pending_shape == 2U || s_pending_shape == 5U)) {
    PrepareArmShape(now);
    return;
  }
  /* 正常步态<->小碎步：保留当前动作组完整的落地->离地衔接，
   * 在离地帧结束后切换，下一帧用另一动作组的800ms前摆，避免200ms内硬切腿姿。 */
  if (s_frame == 13U || s_frame == 17U)
    s_obstacle_small = s_obstacle_small_requested;
  if (f->landing && s_obstacle_over_request && !s_active_shape && !s_pending_shape && !s_stop) {
    s_obstacle_over_request = 0U;
    (void)BeginObstacleFrame(0U, now);
    return;
  }
  next = s_frame + 1U;
  if (next == ACTION_FRAME_COUNT) next = ACTION_LOOP_FIRST;
  /* 按文件落地标签停在3/7/11/15，不把离地帧当双支撑。
   * 无触地传感器：这是计划落地帧，实际触地仍由实机验证。 */
  if (s_frame == ACTION_STEP3_STABLE && s_cycle_direction == 1U &&
      s_pending_shape == 6U && !s_stop) {
    s_active_shape = 6U; s_pending_shape = 0U; s_shape_phase = 5U;
    SG90_StartShake(now);
    return;
  }
  if (f->landing && (s_feedback_hold || (s_stop && (!s_start_pose_stop || s_frame == ACTION_STEP4_STABLE)) || (s_single && s_frame == ACTION_FINAL_LANDING))) {
    FinishStop(next, now); return;
  }
  (void)BeginFrame(next, now);
}

RobotState Robot_GetState(void) { return s_state; }
const char *Robot_GetLastError(void) { return s_error; }
uint8_t Robot_GetFrame(void) { return s_frame; }
uint32_t Robot_GetFrameDuration(void) { return s_duration; }
uint8_t Robot_GetActiveShape(void) { return s_active_shape; }
uint8_t Robot_GetPendingShape(void) { return s_pending_shape; }
uint32_t Robot_GetCompletedShapes(void) { return s_completed_shapes; }
uint32_t Robot_GetInterruptedShapes(void) { return s_interrupted_shapes; }
