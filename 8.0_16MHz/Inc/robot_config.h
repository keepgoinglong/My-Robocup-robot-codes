#ifndef __ROBOT_CONFIG_H
#define __ROBOT_CONFIG_H

/* ===== 越障左右修正死区：现场优先改这里 =====
 * 单位0.1度；50表示横杆角在-5.0度到+5.0度以内时不修正左右。 */
#define OBSTACLE_ALIGN_DEADBAND_DECI_DEG 50
#if OBSTACLE_ALIGN_DEADBAND_DECI_DEG < 0 || OBSTACLE_ALIGN_DEADBAND_DECI_DEG > 900
#error "Obstacle alignment deadband must be within 0..900 deci-degrees"
#endif

/* ===== 7.0主机收到的左弯角缩小倍数：现场优先改这里 =====
 * 4=视觉左弯角先除以4再修正髋；1=不缩小。左转基准值不受影响。 */
#define LEFT_CURVE_ANGLE_DIVISOR          4
#if LEFT_CURVE_ANGLE_DIVISOR < 1
#error "Left curve angle divisor must be positive"
#endif

/* ===== 7.0障碍连续确认次数：现场优先改这里 =====
 * 超声每60ms测量一次；条件中断或无效测距都会重新计数。
 * 8次约480ms后进入小碎步；越障距离及对正条件再连续5次（约300ms）才越障。 */
#define OBSTACLE_APPROACH_CONFIRM_SAMPLES  7U
#define OBSTACLE_OVER_CONFIRM_SAMPLES      5U
#if OBSTACLE_APPROACH_CONFIRM_SAMPLES < 3U || OBSTACLE_OVER_CONFIRM_SAMPLES < 2U
#error "Obstacle confirmation samples are too small"
#endif

/* ===== 7.0小碎步左右髋对正：现场优先改这里 =====
 * 视觉横杆角先除以1.5，再按每度12计数修正当前支撑侧髋。
 * 最大240计数（约21度）；正横杆角向右纠，负横杆角向左纠。 */
#define OBSTACLE_ANGLE_DIVISOR_X10      15 /* 15=1.5；10=1.0；20=2.0。 */
#define OBSTACLE_HIP_COUNTS_PER_DEGREE 12
#define OBSTACLE_HIP_MAX_COUNTS       240
#if OBSTACLE_ANGLE_DIVISOR_X10 < 10
#error "Obstacle angle divisor must be at least 1.0"
#endif

/* 左转时左右髋独立基准：目标=2048+对应基准+视觉微调。
 * 左髋ID2、右髋ID3；增大对应数值会增大该髋的左转目标偏移。
 * 当前两侧均保留原值59（零修正目标2107），只拆分参数，不改变原动作。
 * 左髋转动偏小：只增大LEFT_TURN_LEFT_HIP_BASE_OFFSET，右髋参数不动。 */
#define LEFT_TURN_LEFT_HIP_BASE_OFFSET   45 /* 左髋旋转 ID2 */
#define LEFT_TURN_RIGHT_HIP_BASE_OFFSET  45 /* 右髋旋转 ID3 */

/* 5.1：暂时关闭TOF初始化、测距及报告，XSHUT保持低电平。 */
#define ROBOT_ENABLE_TOF 0

/* 2.3：起步取my_actionsfinalorigin2第6帧左脚离地姿势，从第1帧衔接循环六帧及单舵机时序。
 * 旧版调参及中文注释原样保存在 Reference_1.0；不叠加旧步态补偿。 */
#define ROBOT_SERVO_COUNT               14U
#define ROBOT_MAX_SERVO_COUNT           14U
#define ROBOT_CONTROL_PERIOD_MS         10U
/* 直走小幅提速：只减前摆结束的停留；换重心、落地及所有运动时序不变。
 * 50：200ms -> 150ms，两步2800ms -> 2700ms；设0恢复原始步频。
 * 转向/纠偏及图形保持原时序；实机稳定性须重新验证。 */
/* 2.3严格保留新JSON各帧运动和停留时长，不额外压缩。 */
#define WALK_SWING_HOLD_TRIM_MS          0U
#if WALK_SWING_HOLD_TRIM_MS > 50U || WALK_SWING_HOLD_TRIM_MS % ROBOT_CONTROL_PERIOD_MS != 0U
#error "Walk hold trim must be 0..50 ms in control-period increments"
#endif
/* 临时实机试验：1=忽略AA56方向并在每个行走帧强制默认左转；改回0即可恢复视觉控制。 */
#define TEMP_FORCE_CONTINUOUS_LEFT       0
/* 巡线角度缩小倍数：2表示角度/2，3表示角度/3。 */
#define LINE_ANGLE_DIVISOR               1
#if LINE_ANGLE_DIVISOR < 1
#error "LINE_ANGLE_DIVISOR must be positive"
#endif
/* 默认左转由文件顶部左右髋独立基准设置。正视觉角减少左转，负角增加左转。
 * 每差1度修正5计数，最多±30；严格保持左转，不允许反向。 */
#define LEFT_ADJUST_COUNTS_PER_DEGREE     5
#define LEFT_ADJUST_MAX_COUNTS          30
#if LEFT_ADJUST_COUNTS_PER_DEGREE < 0 || LEFT_ADJUST_MAX_COUNTS < 0 || LEFT_TURN_LEFT_HIP_BASE_OFFSET <= LEFT_ADJUST_MAX_COUNTS || LEFT_TURN_LEFT_HIP_BASE_OFFSET + LEFT_ADJUST_MAX_COUNTS > 116 || LEFT_TURN_RIGHT_HIP_BASE_OFFSET <= LEFT_ADJUST_MAX_COUNTS || LEFT_TURN_RIGHT_HIP_BASE_OFFSET + LEFT_ADJUST_MAX_COUNTS > 116
#error "Left hip range must remain positive and within old left-turn amplitude"
#endif
/* 5.3右纠偏：保留原62计数起始量，随位置误差增强，最多100计数。
 * 正误差每度增加4计数；上限为试机参数，并非实机稳定性结论。 */
#define RIGHT_RECOVER_BASE_COUNTS       62
#define RIGHT_RECOVER_COUNTS_PER_DEGREE  8
#define RIGHT_RECOVER_MAX_COUNTS       100
#if RIGHT_RECOVER_BASE_COUNTS < 0 || RIGHT_RECOVER_COUNTS_PER_DEGREE < 0 || RIGHT_RECOVER_MAX_COUNTS < RIGHT_RECOVER_BASE_COUNTS || RIGHT_RECOVER_MAX_COUNTS > 116
#error "Right recovery must be within 0..116 counts"
#endif
#define ROBOT_USE_EXTERNAL_ADAPTER       1 /* 与1.0一致：USART2，PD5/PD6，1Mbps。 */
#define SERVO_TX_ENABLE_LEVEL  GPIO_PIN_RESET
#define SERVO_TX_DISABLE_LEVEL GPIO_PIN_SET
#define SERVO_RX_ENABLE_LEVEL  GPIO_PIN_RESET
#define SERVO_RX_DISABLE_LEVEL GPIO_PIN_SET

/* 5.1：速度、ACC直接使用动作组原值，运动时间不按速度预算延长。 */
/* 与fddebug一致：等待到位误差20计数，超时5秒。 */
#define ACTION_REACHED_TOLERANCE        20U
/* 已执行s并保持后，g/t允许较小的舵机静差；只用于再次启动，不放宽动作限位。 */
#define ACTION_START_TOLERANCE          50U
#define ACTION_REACHED_TIMEOUT_MS     5000U
#define ACTION_SCHEDULER_TIMEOUT_MS    250U
#endif
