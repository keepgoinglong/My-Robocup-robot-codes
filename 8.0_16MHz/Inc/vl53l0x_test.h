#ifndef VL53L0X_TEST_H
#define VL53L0X_TEST_H

#include "main.h"

/* 8.0控制板接线：XSHUT=PC0，GPIO1=PC1，SDA=PB11，SCL=PB10（板底部I2C口）。 */
uint8_t VL53L0X_TestInit(I2C_HandleTypeDef *i2c);
uint8_t VL53L0X_TestRead(uint16_t *distance_mm, uint8_t *range_status);
uint8_t VL53L0X_TestModelId(void);
uint8_t VL53L0X_TestInitStage(void);

#endif
