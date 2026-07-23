#ifndef AS5600_H
#define AS5600_H

#include "stm32f1xx_hal.h"  // change to your device series header if needed
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AS5600_I2C_ADDR  (0x36 << 1)   // HAL uses 8-bit (7-bit << 1)
#define AS5600_RAW_ANGLE_L  0x0D
#define AS5600_RAW_ANGLE_H  0x0C

// Initialize if you want (not necessary for basic read)
HAL_StatusTypeDef AS5600_Init(I2C_HandleTypeDef *hi2c);

// Read angle in degrees [0..360)
HAL_StatusTypeDef AS5600_ReadAngle_deg(I2C_HandleTypeDef *hi2c, float *angle_deg);

#ifdef __cplusplus
}
#endif

#endif // AS5600_H
