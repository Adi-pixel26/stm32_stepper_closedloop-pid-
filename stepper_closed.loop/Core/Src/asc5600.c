#include "as5600.h"

// Optional init (currently does nothing special, placeholder)
HAL_StatusTypeDef AS5600_Init(I2C_HandleTypeDef *hi2c) {
    // Could check device by reading a register or set configurations if needed

    // no standard who-am-i register for AS5600 — just return HAL_OK
    (void)hi2c;
    return HAL_OK;
}

HAL_StatusTypeDef AS5600_ReadAngle_deg(I2C_HandleTypeDef *hi2c, float *angle_deg) {
    uint8_t buf[2];
    HAL_StatusTypeDef res;

    // Read low and high bytes
    res = HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDR, AS5600_RAW_ANGLE_L, I2C_MEMADD_SIZE_8BIT, &buf[0], 1, HAL_MAX_DELAY);
    if (res != HAL_OK)
    	return HAL_ERROR;

    res = HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDR, AS5600_RAW_ANGLE_H,I2C_MEMADD_SIZE_8BIT, &buf[1], 1, HAL_MAX_DELAY);
    if (res != HAL_OK)
    	return HAL_ERROR;

    uint16_t raw = ((uint16_t)buf[1] << 8) | buf[0];

    // AS5600 angle is 12-bit, mask to keep lower 12 bits
    raw &= 0x0FFFu; // 0..4095

    // Convert to degrees: 360 / 4096 = 0.087890625
    *angle_deg = (float)raw * 0.087890625f;

    return HAL_OK;
}
