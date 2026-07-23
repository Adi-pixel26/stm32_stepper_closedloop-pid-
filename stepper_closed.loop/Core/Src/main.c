/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "as5600.h"
#include "usbd_cdc_if.h"
#include "stdio.h"
#include "stepper.h"
#include "math.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#define DIR_PORT          GPIOA
#define DIR_PIN           GPIO_PIN_1
extern uint8_t usb_rx_buffer[];
extern volatile uint32_t usb_rx_length;
extern volatile uint8_t usb_rx_flag;


// ---- Function prototypes ----
void Stepper_Step(uint16_t steps, GPIO_PinState dir);
void Stepper_GeneratePulse(void);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    CMD_UNKNOWN = 0,
    CMD_PING,
    CMD_SET_POS,
    CMD_GET_POS,
    CMD_GET_POS_ANGLE
} usb_cmd_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
Stepper_Config_t stepper_cfg;
Stepper_State_t stepper_state;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);

char buffer[32];
float angle;
static uint8_t fail_count = 0;
int received_angle_deg = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void Stepper_SetupConfig(void) {
    stepper_cfg.hi2c = &hi2c1;               // I2C handle for AS5600
    stepper_cfg.htim_step = &htim2;          // Timer for PWM step generation
    stepper_cfg.step_channel = TIM_CHANNEL_1;// Step pin channel

    stepper_cfg.dir_port = GPIOA;            // Direction pin port
    stepper_cfg.dir_pin = GPIO_PIN_1;        // Direction pin (example: PA1)

    stepper_cfg.kp = 2.0f;                   // PID proportional gain
    stepper_cfg.ki = 0.5f;                   // PID integral gain
    stepper_cfg.kd = 0.1f;                   // PID derivative gain

    stepper_cfg.steps_per_rev = 4000.0f;      // Motor steps per revolution
    stepper_cfg.max_steps_per_sec = 2000.0f; // Max stepping speed
    stepper_cfg.angle_tolerance_deg = 1.0f;  // Stop when within 1 degree
    stepper_cfg.control_interval_ms = 7;    // PID update every 10 ms

    stepper_cfg.target_angle_deg = 180.0f;    // Target position (initially 90°)
}

static inline void usb_ok(void) {
    CDC_Transmit_FS((uint8_t*)"OK\r\n", 4);
}

static inline void usb_err(const char *err) {
    CDC_Transmit_FS((uint8_t*)err, strlen(err));
}

usb_cmd_t parse_usb_command(uint8_t *data, uint32_t len)
{
    char cmd[32];
    uint32_t i = 0;

    if (len == 0 || len >= sizeof(cmd))
        return CMD_UNKNOWN;

    /* Copy only command token (stop at space / CR / LF) EG: for something entered like SET_POS 80, it parses till blank */
    while (i < len && data[i] != ' ' && data[i] != '\r' && data[i] != '\n')
    {
        cmd[i] = data[i];
        i++;
    }
    cmd[i] = '\0';

    if (!strcmp(cmd, "PING")) return CMD_PING;
    if (!strcmp(cmd, "SET_POS")) return CMD_SET_POS;
    if (!strcmp(cmd, "GET_POS")) return CMD_GET_POS;
    if (!strcmp(cmd, "GET_POS_ANGLE")) return CMD_GET_POS_ANGLE;

    return CMD_UNKNOWN;
}

void USB_Command_Handle(uint8_t *data, uint32_t len)
{
    usb_cmd_t cmd = parse_usb_command(data, len);
    char *p = (char *)data;

    switch (cmd)
    {
    case CMD_PING:
    	usb_ok();
        break;

    case CMD_SET_POS:
    {
        while (*p && *p != ' ') { p++; }
        if (!*p) { usb_err("ERR ARG\r\n"); break; }
        p++;

        int angle = atoi(p);
        if (angle < 1 || angle > 359) {
            usb_err("ERR RANGE\r\n");
            break;
        }

        received_angle_deg=angle;
          // Combine text + value into one single message
          char msg[32];
          sprintf(msg, " Value recieved: %d\r\n", angle);

          // Send in one go (prevents empty lines)
          CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
          usb_ok();
        break;
    }
    case CMD_GET_POS:
    case CMD_GET_POS_ANGLE:
    {
        char msg[32];
        snprintf(msg, sizeof(msg), "%.3f\r\n", angle);
        CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
        break;
    }

    default:
        usb_err("ERR CMD\r\n");
        break;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  AS5600_Init(&hi2c1);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

 //char usb_rx_buffer[64];
 //float received_angle_deg = 0;
  Stepper_SetupConfig();
  Stepper_Init(&stepper_cfg, &stepper_state);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  //angle


 	 for(int e=0;e<1000;e++){
 		  AS5600_ReadAngle_deg(&hi2c1, &angle);
 		  stepper_cfg.target_angle_deg = angle;
 		 	 received_angle_deg=stepper_cfg.target_angle_deg;
 	 }
  //uint8_t first_sync_done = 0;
  stepper_cfg.target_angle_deg = 180;     // follow magnet
 	  	       //   Stepper_Update(&stepper_cfg, &stepper_state);
 // HAL_Delay(10000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // Read angle from AS5600
	  	  if (AS5600_ReadAngle_deg(&hi2c1, &angle) == HAL_OK)
	  	   {
	  	          //Stepper_SetTarget(&stepper_cfg, angle);
	  		  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	  		  	  fail_count = 0;
//	  		  	if (first_sync_done == 0)
//	  		  	        {
//	  		  	            stepper_cfg.target_angle_deg = angle;
//	  		  	            first_sync_done = 1;
//	  		  	        }
	  	          stepper_cfg.target_angle_deg = 180+(float)received_angle_deg;     // follow magnet
	  	          Stepper_Update(&stepper_cfg, &stepper_state);


	  	           // Convert to integer (no decimals)
	  	           int int_angle = (int)angle;



	  	      }
	  	else
	  	        {
	  		fail_count++;
	  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	  	            CDC_Transmit_FS((uint8_t*)"Sensor Error\r\n", 14);
	  	          if (fail_count > 3) {   // after 3 consecutive failures
	  	        	I2C_Reset_Bus(&hi2c1);
	  	                  fail_count = 0;
	  	        }
	  	        }
	  	 if( AS5600_ReadAngle_deg(&hi2c1, &angle) == HAL_ERROR)
	  	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

	  	 if (usb_rx_flag)
	  	    {
	  		 usb_rx_flag = 0;       // clear immediately
	  		 USB_Command_Handle(usb_rx_buffer, usb_rx_length);
	  		 usb_rx_length = 0;
	  	    }

	   }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 80000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void I2C_Reset_Bus(I2C_HandleTypeDef *hi2c)
{
    // 1. De-initialize the I2C peripheral
    HAL_I2C_DeInit(hi2c);

    // 2. Reconfigure the GPIO pins manually to release stuck lines
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    // SDA = PB7, SCL = PB6
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 3. Toggle SCL manually to release SDA (9 clock pulses)
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_Delay(1);
    }

    // 4. Generate a STOP condition
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1);

    // 5. Re-initialize I2C
    HAL_I2C_Init(hi2c);
}

void delay_us(uint32_t us) {
    uint32_t cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000; // SysClk MHz
    uint32_t start = DWT->CYCCNT;
    uint32_t delay_cycles = us * cycles_per_us;
    while ((DWT->CYCCNT - start) < delay_cycles);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
