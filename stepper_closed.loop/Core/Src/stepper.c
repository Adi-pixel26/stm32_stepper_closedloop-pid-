#include "stepper.h"
#include <math.h>
extern void delay_us(uint32_t us);
static float normalize_angle_error(float err) {
    // normalize to [-180, +180)
    while (err > 180.0f)
    	err -= 360.0f;
    while (err <= -180.0f)
    	err += 360.0f;
    return err;
}

// NOTE: This helper assumes TIM2 (APB1) for timer clock calculation. If using APB2 timers or different MCU,
// update to compute timer clock properly. For generality we compute by checking which bus TIM is on.
static uint32_t Stepper_GetTimerClockHz(TIM_HandleTypeDef *htim) {
    // Simplified: TIM2 is on PCLK1 for many STM32 families. Adjust if needed.
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    // If APB1 prescaler != 1, timer clocks run at 2x PCLK1.
    // The HAL function does not provide prescaler value, but we can check RCC registers (portable-ish).
    // For simplicity we assume timer clock equals pclk1 * (APB prescaler == 1 ? 1 : 2).
    uint32_t presc = (RCC->CFGR & RCC_CFGR_PPRE1) ? 2u : 1u; // crude; on some MCUs different masks; adjust if wrong.
    return pclk1 * presc;
}

void Stepper_Init(const Stepper_Config_t *cfg, Stepper_State_t *state) {
    (void)cfg;
    if (state) {
        state->integral = 0.0f;
        state->previous_angle_deg = 0.0f;
        state->previous_error = 0.0f;
        state->last_tick = HAL_GetTick();
        state->current_dir = 0;
    }
}

// Main non-blocking update: call from while(1)
HAL_StatusTypeDef Stepper_Update(const Stepper_Config_t *cfg, Stepper_State_t *state) {
    if (!cfg || !state)
    	return HAL_ERROR;

    uint32_t now = HAL_GetTick();
    uint32_t dt_ms = now - state->last_tick;
    if (dt_ms < cfg->control_interval_ms)
    	return HAL_OK; // not time yet

    state->last_tick = now;
    float dt = (float)dt_ms / 1000.0f;

    float measured_deg;
    HAL_StatusTypeDef res = AS5600_ReadAngle_deg(cfg->hi2c, &measured_deg);
    if (res != HAL_OK) return res;

    // compute shortest angle error
    float error = normalize_angle_error(cfg->target_angle_deg - measured_deg);
    state->previous_angle_deg = measured_deg;
    // if within tolerance => stop PWM and reset integral
    if (fabsf(error) <= cfg->angle_tolerance_deg) {
        // stop PWM output (safe stop)
        HAL_TIM_PWM_Stop(cfg->htim_step, cfg->step_channel);
        state->integral = 0.0f;
        state->previous_error = error;
        return HAL_OK;
    }

    // PID
    state->integral += error * dt;
    // anti-windup clamp
    float i_max = cfg->max_steps_per_sec * 360.0f / cfg->steps_per_rev; // arbitrary scaling limit
    if (state->integral > i_max) state->integral = i_max;
    if (state->integral < -i_max) state->integral = -i_max;

    float derivative = (error - state->previous_error) / dt;
    float pid_output = cfg->kp * error + cfg->ki * state->integral + cfg->kd * derivative;
    state->previous_error = error;

    // pid_output is in degrees per second (interpretation). We'll map to steps/sec:
    // steps/sec = abs(pid_output) * steps_per_rev / 360
    float steps_per_sec = fabsf(pid_output) * cfg->steps_per_rev / 360.0f;

    // clamp
    if (steps_per_sec > cfg->max_steps_per_sec)
    	steps_per_sec = cfg->max_steps_per_sec;
    if (steps_per_sec < 1.0f)
    	steps_per_sec = 1.0f;

    // set direction
    if (pid_output >= 0.0f) {
    	if (state->current_dir != 1) {
    	    	        HAL_TIM_PWM_Stop(cfg->htim_step, TIM_CHANNEL_1);     // stop pulses
    	    	        HAL_GPIO_WritePin(cfg->dir_port, cfg->dir_pin, GPIO_PIN_SET); // set CW
    	    	        delay_us(20);  // allow DIR to latch
    	    	        HAL_TIM_PWM_Start(cfg->htim_step,TIM_CHANNEL_1);    // restart pulses
    	    	        state->current_dir = 1;
    	    	    }
    } else {
    	if (state->current_dir != -1) {
    	    	        HAL_TIM_PWM_Stop(cfg->htim_step, cfg->step_channel);
    	    	        HAL_GPIO_WritePin(cfg->dir_port, cfg->dir_pin, GPIO_PIN_RESET); // set CCW
    	    	        delay_us(20);
    	    	        HAL_TIM_PWM_Start(cfg->htim_step, cfg->step_channel);
    	    	        state->current_dir = -1;
    	    }
    }

    // compute timer ARR for desired frequency (steps_per_sec)
    // For PWM frequency = steps_per_sec, with 50% duty
    uint32_t timer_clock = Stepper_GetTimerClockHz(cfg->htim_step); // Hz
    uint32_t prescaler = cfg->htim_step->Instance->PSC; // we'll try keep existing PSC
    if (prescaler == 0)
    	prescaler = 0; // if not set, assume 0

    // compute arr: frequency = timer_clock / ((PSC+1)*(ARR+1))
    // => ARR = (timer_clock / ((PSC+1)*frequency)) - 1
    uint32_t denom = (uint32_t)((prescaler + 1) * steps_per_sec);
    uint32_t arr;
    if (denom == 0)
    	arr = 0xFFFF;
    else {
        uint64_t tmp = (uint64_t)timer_clock * 1ull;
        tmp = tmp / denom;
        if (tmp == 0) tmp = 1;
        arr = (uint32_t)(tmp - 1);
    }

    if (arr > 0xFFFF)
    	arr = 0xFFFF; // keep in 16-bit ARR range (or 32-bit depending on timer)
    if (arr < 10)
    	arr = 10;

    // write ARR and CCRx
    // NOTE: ensure channel mapping; this code assumes TIM_CHANNEL_1 -> CCR1, etc.
    // We set CCR = ARR/2 for ~50% duty
    __HAL_TIM_DISABLE(cfg->htim_step);
    cfg->htim_step->Instance->PSC = prescaler;
    cfg->htim_step->Instance->ARR = arr;

    // set CCR for the configured channel
    // Set CCR1 for 50% duty cycle (pulse width)
    cfg->htim_step->Instance->CCR1 = arr / 2;

    // Re-enable timer
    __HAL_TIM_ENABLE(cfg->htim_step);

    __HAL_TIM_ENABLE(cfg->htim_step);

    // start PWM on channel (non-blocking)
   // HAL_TIM_PWM_Start(cfg->htim_step, cfg->step_channel);
    HAL_TIM_PWM_Start(cfg->htim_step, TIM_CHANNEL_1);
    return HAL_OK;
}
