#include "vl53l0x_test.h"

#define VL53_ADDR              (0x29U << 1)
#define REG_SYSRANGE_START      0x00U
#define REG_INTERRUPT_CLEAR     0x0BU
#define REG_RESULT_STATUS       0x13U
#define REG_RANGE_MM            0x1EU
#define REG_MODEL_ID            0xC0U

static I2C_HandleTypeDef *s_i2c;
static uint8_t s_stop_variable, s_model_id;
static uint8_t s_init_stage;

static uint8_t Write8(uint8_t reg, uint8_t value)
{
  uint8_t retry;
  for (retry=0U;retry<3U;++retry) {
    if (HAL_I2C_Mem_Write(s_i2c, VL53_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                          &value, 1U, 100U)==HAL_OK) return 1U;
    HAL_Delay(2U);
  }
  return 0U;
}
static uint8_t Read8(uint8_t reg, uint8_t *value)
{
  uint8_t retry;
  for (retry=0U;retry<3U;++retry) {
    if (HAL_I2C_Mem_Read(s_i2c, VL53_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                         value, 1U, 100U)==HAL_OK) return 1U;
    HAL_Delay(2U);
  }
  return 0U;
}
static uint8_t Read16(uint8_t reg, uint16_t *value)
{
  uint8_t data[2], retry;
  for (retry=0U;retry<3U;++retry) {
    if (HAL_I2C_Mem_Read(s_i2c, VL53_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                         data, 2U, 100U)==HAL_OK) {
      *value = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
      return 1U;
    }
    HAL_Delay(2U);
  }
  return 0U;
}
static uint8_t ReadMulti(uint8_t reg, uint8_t *data, uint16_t count)
{
  return HAL_I2C_Mem_Read(s_i2c, VL53_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                          data, count, 100U) == HAL_OK;
}
static uint8_t WriteMulti(uint8_t reg, uint8_t *data, uint16_t count)
{
  return HAL_I2C_Mem_Write(s_i2c, VL53_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                           data, count, 100U) == HAL_OK;
}
static uint8_t WriteList(const uint8_t (*list)[2], uint16_t count)
{
  uint16_t i;
  for (i = 0U; i < count; ++i) if (!Write8(list[i][0], list[i][1])) return 0U;
  return 1U;
}

static uint8_t LoadDefaultTuning(void)
{
  static const uint8_t k[][2] = {
    {0xFF,0x01},{0x00,0x00},{0xFF,0x00},{0x09,0x00},{0x10,0x00},{0x11,0x00},
    {0x24,0x01},{0x25,0xFF},{0x75,0x00},{0xFF,0x01},{0x4E,0x2C},{0x48,0x00},
    {0x30,0x20},{0xFF,0x00},{0x30,0x09},{0x54,0x00},{0x31,0x04},{0x32,0x03},
    {0x40,0x83},{0x46,0x25},{0x60,0x00},{0x27,0x00},{0x50,0x06},{0x51,0x00},
    {0x52,0x96},{0x56,0x08},{0x57,0x30},{0x61,0x00},{0x62,0x00},{0x64,0x00},
    {0x65,0x00},{0x66,0xA0},{0xFF,0x01},{0x22,0x32},{0x47,0x14},{0x49,0xFF},
    {0x4A,0x00},{0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},{0x78,0x21},{0xFF,0x01},
    {0x23,0x34},{0x42,0x00},{0x44,0xFF},{0x45,0x26},{0x46,0x05},{0x40,0x40},
    {0x0E,0x06},{0x20,0x1A},{0x43,0x40},{0xFF,0x00},{0x34,0x03},{0x35,0x44},
    {0xFF,0x01},{0x31,0x04},{0x4B,0x09},{0x4C,0x05},{0x4D,0x04},{0xFF,0x00},
    {0x44,0x00},{0x45,0x20},{0x47,0x08},{0x48,0x28},{0x67,0x00},{0x70,0x04},
    {0x71,0x01},{0x72,0xFE},{0x76,0x00},{0x77,0x00},{0xFF,0x01},{0x0D,0x01},
    {0xFF,0x00},{0x80,0x01},{0x01,0xF8},{0xFF,0x01},{0x8E,0x01},{0x00,0x01},
    {0xFF,0x00},{0x80,0x00}
  };
  return WriteList(k, (uint16_t)(sizeof(k) / sizeof(k[0])));
}

/* ST参考流程：读取芯片内保存的有效参考SPAD数量和类型。 */
static uint8_t GetSpadInfo(uint8_t *count, uint8_t *is_aperture)
{
  uint8_t value;
  uint32_t begin;
  if (!Write8(0x80U,0x01U) || !Write8(0xFFU,0x01U) || !Write8(0x00U,0x00U) ||
      !Write8(0xFFU,0x06U) || !Read8(0x83U,&value) ||
      !Write8(0x83U,(uint8_t)(value|0x04U)) || !Write8(0xFFU,0x07U) ||
      !Write8(0x81U,0x01U) || !Write8(0x80U,0x01U) || !Write8(0x94U,0x6AU) ||
      !Write8(0x83U,0x00U)) return 0U;
  begin=HAL_GetTick();
  do {
    if (!Read8(0x83U,&value)) return 0U;
    if (value!=0U) break;
  } while ((uint32_t)(HAL_GetTick()-begin)<200U);
  if (value==0U || !Write8(0x83U,0x01U) || !Read8(0x92U,&value)) return 0U;
  *count=(uint8_t)(value&0x7FU);
  *is_aperture=(uint8_t)((value>>7)&0x01U);
  if (!Write8(0x81U,0x00U) || !Write8(0xFFU,0x06U) || !Read8(0x83U,&value) ||
      !Write8(0x83U,(uint8_t)(value&~0x04U)) || !Write8(0xFFU,0x01U) ||
      !Write8(0x00U,0x01U) || !Write8(0xFFU,0x00U) || !Write8(0x80U,0x00U)) return 0U;
  return 1U;
}

static uint8_t PerformSingleRefCalibration(uint8_t vhv_init_byte)
{
  uint8_t value;
  uint32_t begin;
  if (!Write8(REG_SYSRANGE_START,(uint8_t)(0x01U|vhv_init_byte))) return 0U;
  begin=HAL_GetTick();
  do {
    if (!Read8(REG_RESULT_STATUS,&value)) return 0U;
    if ((value&0x07U)!=0U) break;
  } while ((uint32_t)(HAL_GetTick()-begin)<2000U);
  if ((value&0x07U)==0U) {
    /* 部分VL53L0X兼容模块Phase校准不产生完成中断；先停止本次校准。 */
    Write8(REG_INTERRUPT_CLEAR,0x01U);
    Write8(REG_SYSRANGE_START,0x00U);
    return 0U;
  }
  return Write8(REG_INTERRUPT_CLEAR,0x01U) && Write8(REG_SYSRANGE_START,0x00U);
}

uint8_t VL53L0X_TestInit(I2C_HandleTypeDef *i2c)
{
  uint8_t value, spad_count, spad_is_aperture, spad_map[6], i, enabled;
  uint32_t begin;
  s_i2c = i2c; s_model_id = 0U; s_init_stage = 1U;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_Delay(5U);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_Delay(10U);
  if (HAL_I2C_IsDeviceReady(s_i2c, VL53_ADDR, 3U, 20U) != HAL_OK) return 0U;
  s_init_stage = 2U;
  if (!Read8(REG_MODEL_ID, &s_model_id) || s_model_id != 0xEEU) return 0U;
  if (!Read8(0x89U, &value) || !Write8(0x89U, (uint8_t)(value | 0x01U))) return 0U;
  if (!Write8(0x88U,0x00U) || !Write8(0x80U,0x01U) || !Write8(0xFFU,0x01U) ||
      !Write8(0x00U,0x00U) || !Read8(0x91U,&s_stop_variable) || !Write8(0x00U,0x01U) ||
      !Write8(0xFFU,0x00U) || !Write8(0x80U,0x00U)) return 0U;
  s_init_stage = 3U;
  if (!Read8(0x60U,&value) || !Write8(0x60U,(uint8_t)(value|0x12U)) ||
      !Write8(0x44U,0x00U) || !Write8(0x45U,0x20U) || !Write8(0x01U,0xFFU) ||
      !GetSpadInfo(&spad_count,&spad_is_aperture) ||
      !ReadMulti(0xB0U,spad_map,6U) || !Write8(0xFFU,0x01U) ||
      !Write8(0x4FU,0x00U) || !Write8(0x4EU,0x2CU) || !Write8(0xFFU,0x00U) ||
      !Write8(0xB6U,0xB4U)) return 0U;
  s_init_stage = 4U;
  enabled=0U;
  for (i=0U;i<48U;++i) {
    if (i<(spad_is_aperture?12U:0U) || enabled==spad_count)
      spad_map[i/8U]&=(uint8_t)~(1U<<(i%8U));
    else if ((spad_map[i/8U]>>(i%8U))&0x01U) ++enabled;
  }
  if (!WriteMulti(0xB0U,spad_map,6U) || !LoadDefaultTuning()) return 0U;
  s_init_stage = 5U;
  if (!Write8(0x0AU,0x04U) || !Read8(0x84U,&value) ||
      !Write8(0x84U,(uint8_t)(value & (uint8_t)~0x10U)) ||
      !Write8(REG_INTERRUPT_CLEAR,0x01U) || !Write8(0x01U,0xE8U)) return 0U;
  s_init_stage = 51U;
  if (!Write8(0x01U,0x01U) || !PerformSingleRefCalibration(0x40U)) return 0U;
  s_init_stage = 52U;
  if (!Write8(0x01U,0x02U)) return 0U;
  /* 个别兼容模块第二次Phase校准会超时；第一次VHV成功后允许继续连续测距。 */
  (void)PerformSingleRefCalibration(0x00U);
  s_init_stage = 6U;
  if (!Write8(0x01U,0xE8U)) return 0U;
  if (!Write8(0x80U,0x01U) || !Write8(0xFFU,0x01U) || !Write8(0x00U,0x00U) ||
      !Write8(0x91U,s_stop_variable) || !Write8(0x00U,0x01U) || !Write8(0xFFU,0x00U) ||
      !Write8(0x80U,0x00U) || !Write8(REG_SYSRANGE_START,0x02U)) return 0U;
  begin=HAL_GetTick();
  do {
    if (!Read8(REG_RESULT_STATUS,&value)) return 0U;
    if ((value & 0x07U) != 0U) { s_init_stage = 7U; return 1U; }
  } while ((uint32_t)(HAL_GetTick()-begin)<100U);
  return 0U;
}

uint8_t VL53L0X_TestRead(uint16_t *distance_mm, uint8_t *range_status)
{
  uint8_t ready, raw;
  if (s_i2c==NULL || distance_mm==NULL || range_status==NULL) return 0U;
  if (!Read8(REG_RESULT_STATUS,&ready)) return 0U;
  if ((ready & 0x07U)==0U) return 2U;
  if (!Read8(0x14U,&raw) || !Read16(REG_RANGE_MM,distance_mm)) return 0U;
  *range_status=(uint8_t)((raw>>3)&0x0FU);
  if (!Write8(REG_INTERRUPT_CLEAR,0x01U)) return 0U;
  return 1U;
}

uint8_t VL53L0X_TestModelId(void) { return s_model_id; }
uint8_t VL53L0X_TestInitStage(void) { return s_init_stage; }
