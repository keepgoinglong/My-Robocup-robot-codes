/* Frame-6 left-foot-lift start pose and six-frame loop from my_actionsfinalorigin4; old startup removed. */
/* SHA256: f4ce8b3f07bd7be766a94b65ea1a159c1f3c0e3bf5f9ea46b4880be35eb22a51 */
#ifndef ACTION_FRAMES_H
#define ACTION_FRAMES_H
#define ACTION_FRAME_COUNT 18U
#define ACTION_LOOP_FIRST 10U
/* my_actionsfinalorigin4原文件：第2/5帧是双脚着地稳定点，第6帧（左脚离地）是直走起始动作。 */
#define ACTION_STEP3_STABLE 11U
#define ACTION_STEP4_STABLE 15U
#define ACTION_WALK_START_FRAME 17U
#define ACTION_WALK_START_NEXT ACTION_LOOP_FIRST
#define ACTION_FINAL_LANDING ACTION_STEP4_STABLE
typedef struct {
  uint16_t position[ROBOT_SERVO_COUNT];
  uint32_t duration_ms, hold_ms;
  uint16_t speed;
  uint8_t acc, wait_reached, landing;
} ActionFrame;
static const uint16_t k_action_limits[ROBOT_SERVO_COUNT][2] = {
  {48, 4048}, /* ID1 */
  {48, 4048}, /* ID2 */
  {48, 4048}, /* ID3 */
  {48, 4048}, /* ID4 */
  {1800, 4048}, /* ID5 */
  {48, 2258}, /* ID6 */
  {1308, 2962}, /* ID7 */
  {1360, 2786}, /* ID8 */
  {1140, 2768}, /* ID9 */
  {1355, 2800}, /* ID10 */
  {1600, 3080}, /* ID11 */
  {977, 2500}, /* ID12 */
  {1124, 2800}, /* ID13 */
  {1500, 2972}, /* ID14 */
};
static const ActionFrame k_action_frames[ACTION_FRAME_COUNT] = {
  /* 0: 第2步·左脚·离地 */
  {{2048, 2048, 2048, 2048, 1950, 2020, 2248, 1920, 2148, 1800, 2180, 2188, 1750, 1720}, 200U, 50U, 1600U, 16U, 0U, 0U},
  /* 1: Unused removed startup */
  {{2048, 2048, 2048, 2048, 1950, 2020, 2248, 1920, 2148, 1800, 2180, 2188, 1750, 1720}, 0U, 0U, 1600U, 16U, 0U, 0U},
  /* 2: Unused removed startup */
  {{2048, 2048, 2048, 2048, 1950, 2020, 2248, 1920, 2148, 1800, 2180, 2188, 1750, 1720}, 0U, 0U, 1600U, 16U, 0U, 0U},
  /* 3: Unused removed startup */
  {{2048, 2048, 2048, 2048, 1950, 2020, 2248, 1920, 2148, 1800, 2180, 2188, 1750, 1720}, 0U, 0U, 1600U, 16U, 0U, 0U},
  /* 4: Unused removed startup */
  {{2048, 2048, 2048, 2048, 1950, 2020, 2248, 1920, 2148, 1800, 2180, 2188, 1750, 1720}, 0U, 0U, 1600U, 16U, 0U, 0U},
  /* 5: Unused removed startup */
  {{2048, 2048, 2048, 2048, 1950, 2020, 2248, 1920, 2148, 1800, 2180, 2188, 1750, 1720}, 0U, 0U, 1600U, 16U, 0U, 0U},
  /* 6: Unused removed startup */
  {{2048, 2048, 2048, 2048, 1950, 2020, 2248, 1920, 2148, 1800, 2180, 2188, 1750, 1720}, 0U, 0U, 1600U, 16U, 0U, 0U},
  /* 7: Unused removed startup */
  {{2048, 2048, 2048, 2048, 1950, 2020, 2248, 1920, 2148, 1800, 2180, 2188, 1750, 1720}, 0U, 0U, 1600U, 16U, 0U, 0U},
  /* 8: Unused removed startup */
  {{2048, 2048, 2048, 2048, 1950, 2020, 2248, 1920, 2148, 1800, 2180, 2188, 1750, 1720}, 0U, 0U, 1600U, 16U, 0U, 0U},
  /* 9: Unused removed startup */
  {{2048, 2048, 2048, 2048, 1950, 2020, 2248, 1920, 2148, 1800, 2180, 2188, 1750, 1720}, 0U, 0U, 1600U, 16U, 0U, 0U},
  /* 10: 第1步·右脚·前摆 */
  {{2448, 2048, 2048, 2448, 1920, 2000, 1848, 2148, 1848, 2180, 2280, 2188, 2248, 2410}, 800U, 50U, 1800U, 80U, 0U, 0U},
  /* 11: 第1步·右脚·落地 */
  {{2248, 2048, 2048, 2248, 2020, 2008, 1988, 2268, 1820, 2180, 2030, 2008, 2400, 2350}, 200U, 50U, 1600U, 16U, 0U, 1U},
  /* 12: Unused straight slot; legacy turn only */
  {{2248, 2048, 2048, 2248, 2020, 2008, 1988, 2268, 1820, 2180, 2030, 2008, 2400, 2350}, 0U, 0U, 1600U, 16U, 0U, 0U},
  /* 13: 第1步·右脚·离地 */
  {{2048, 2048, 2048, 2048, 2030, 2170, 2048, 2360, 1940, 2180, 1930, 1870, 2300, 2270}, 200U, 50U, 1600U, 16U, 0U, 0U},
  /* 14: 第2步·左脚·前摆 */
  {{1648, 2048, 2048, 1648, 2050, 2140, 2248, 1920, 2248, 1920, 1880, 1880, 1660, 1840}, 800U, 50U, 1800U, 80U, 0U, 0U},
  /* 15: 第2步·左脚·落地 */
  {{1848, 2048, 2048, 1848, 2120, 2140, 2300, 2000, 2148, 1840, 2028, 2048, 1750, 1780}, 200U, 50U, 1600U, 16U, 0U, 1U},
  /* 16: Unused straight slot; legacy turn only */
  {{1848, 2048, 2048, 1848, 2120, 2140, 2300, 2000, 2148, 1840, 2028, 2048, 1750, 1780}, 0U, 0U, 1600U, 16U, 0U, 0U},
  /* 17: 第2步·左脚·离地 */
  {{2048, 2048, 2048, 2048, 1950, 2020, 2248, 1920, 2148, 1800, 2180, 2188, 1750, 1720}, 200U, 50U, 1600U, 16U, 0U, 0U},
};
static const uint32_t k_action_delay[ACTION_FRAME_COUNT][ROBOT_SERVO_COUNT] = {
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
};
static const uint32_t k_action_joint_duration[ACTION_FRAME_COUNT][ROBOT_SERVO_COUNT] = {
{200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{800U, 800U, 800U, 800U, 800U, 800U, 800U, 800U, 800U, 800U, 800U, 800U, 800U, 500U},
{200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U},
{800U, 800U, 800U, 800U, 800U, 800U, 800U, 800U, 800U, 800U, 800U, 800U, 700U, 800U},
{200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U},
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
{200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U},
};
#endif
