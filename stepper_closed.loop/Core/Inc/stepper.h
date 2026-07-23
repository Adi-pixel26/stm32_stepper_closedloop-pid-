#ifndef STEPPER_H
#define STEPPER_H

#include "stm32f1xx_hal.h"
#include "as5600.h"
#include <math.h>

typedef struct {
    TIM_HandleTypeDef *htim_step; // timer used to generate STEP pulses (PWM)
    uint32_t step_channel;        // TIM_CHANNEL_x
    GPIO_TypeDef *dir_port;       // direction GPIO port
    uint16_t dir_pin;             // direction GPIO pin
    I2C_HandleTypeDef *hi2c;      // encoder i2c handle
    float target_angle_deg;       // desired target angle (0..360)
    float steps_per_rev;          // total steps per revolution (motor * microsteps)
    float kp, ki, kd;             // pid constants
    float max_steps_per_sec;      // clamp speed
    float angle_tolerance_deg;    // acceptable angle error to stop
    uint32_t control_interval_ms; // PID interval (ms)
} Stepper_Config_t;

typedef struct {
    float integral;
    float previous_angle_deg;
    float previous_error;
    uint32_t last_tick;
    int8_t current_dir;
} Stepper_State_t;

// Initialize config & state
void Stepper_Init(const Stepper_Config_t *cfg, Stepper_State_t *state);

// Call periodically in main loop (non-blocking). Returns HAL_OK or HAL_ERROR on I2C read fail.
HAL_StatusTypeDef Stepper_Update(const Stepper_Config_t *cfg, Stepper_State_t *state);

// Set target angle (0..360)
static inline void Stepper_SetTarget(Stepper_Config_t *cfg, float angle_deg) {
    if (angle_deg < 0.0f) {
        angle_deg = fmodf(angle_deg, 360.0f) + 360.0f;
    } else if (angle_deg >= 360.0f) {
        angle_deg = fmodf(angle_deg, 360.0f);
    }
    ((Stepper_Config_t *)cfg)->target_angle_deg = angle_deg;
}

#endif // STEPPER_H
