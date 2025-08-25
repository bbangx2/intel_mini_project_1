/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Mini project (Intel Edge Academy 06052025)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


/* 개발자 : 		진재우, 황경태
 * 최종 수정일 : 	2025-06-05
 * 프로그램 설명 :	인텔 엣지 SW아카데미 미니 프로젝트 1
 * 					자동 알약 디스펜서 프로그램 STM32F411RE 보드 main.c 함수
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp.h"
#include "dht.h"
#include "clcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#define ARR_CNT 5
#define CMD_SIZE 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t rx2char;
extern cb_data_t cb_data;
extern volatile unsigned char rx2Flag;
extern volatile char rx2Data[50];
volatile int tim2Flag1Sec=1;
volatile unsigned int tim2Sec;
volatile int keyNo;
volatile int connect_status;
volatile int pillTypeCount;
volatile int pill_doses[5];
volatile char serverMsg[32];
volatile int set_check_1;
volatile int set_check_2;
volatile int motorDone = 1;
volatile int motor_sequence_active = 0;
volatile int current_motor_idx = 0;
volatile int motor_current_step = 0;
volatile uint32_t motor_step_start_time = 0;
int motor_target_repetitions[3] = {0, 0, 0};
int motor_completed_repetitions[3] = {0, 0, 0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
char strBuff[MAX_ESP_COMMAND_LEN];
void MX_GPIO_LED_ON(int flag);
void MX_GPIO_LED_OFF(int flag);
void esp_event(char *);
long map(long x, long in_min, long in_max, long out_min, long out_max);
void set_rgb_led(int red, int green, int blue);
void process_motor_sequence(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	int ret = 0;
	DHT11_TypeDef dht11Data;
	char buff[30];
	char lcdLine2[17];
	int pulse1 = 500;
	int pulse2 = 500;
	int pulse3 = 500;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  printf("Start main() - mini project\r\n");
  ret |= drv_uart_init();
  ret |= drv_esp_init();

  if(ret != 0)
  {
	  printf("Esp response error\r\n");
	  Error_Handler();
  }

  AiotClient_Init();

  DHT11_Init();
  LCD_init(&hi2c1);
  LCD_writeStringXY(0, 0, "hello lcd");

  if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
      Error_Handler();
  }


  if(HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  if(HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
  if(HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }
  if(HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK) { Error_Handler(); }

  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, (pulse1-1)<0?0:pulse1-1);
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, (pulse2-1)<0?0:pulse2-1);
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, (pulse3-1)<0?0:pulse3-1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    process_motor_sequence();

    if(!connect_status)
    {
      while(1)
      {
        if(strstr((char *)cb_data.buf,"+IPD") && cb_data.buf[cb_data.length-1] == '\n')
        {
          strcpy(strBuff,strchr((char *)cb_data.buf,'['));
          memset(cb_data.buf,0x0,sizeof(cb_data.buf));
          cb_data.length = 0;
          esp_event(strBuff);
        }
        if(rx2Flag)
        {
          printf("recv2 : %s\r\n",rx2Data);
          rx2Flag = 0;
        }

        if(tim2Flag1Sec)
        {
          tim2Flag1Sec = 0;
          if(!(tim2Sec%10))
          {
            if(esp_get_status() != 0)
            {
              printf("server connecting ...\r\n");
              esp_client_conn();
            }
          }
        }
        if (connect_status == 1)
        {
          break;
        }
      }
    }

    if (!set_check_1)
    {
      while (1)
      {
        LCD_writeStringXY(0, 0, "Enter pill type:");
        if (keyNo == 1) { pillTypeCount++; keyNo = 0; }
        else if (keyNo == 2) { if (pillTypeCount > 0) pillTypeCount--; keyNo = 0; }
        else if (keyNo == 3) {
          keyNo = 0;
          sprintf(serverMsg, "[HKT_LIN]PILLTYPE@%d\r\n", pillTypeCount);
          printf("%s\r\n",serverMsg);
          esp_send_data(serverMsg);
          set_check_1 = 1;
          break;
        }
        sprintf(lcdLine2, "count : %d    ", pillTypeCount);
        LCD_writeStringXY(1, 0, lcdLine2);
      }
    }

    if (!set_check_2)
    {
      for (int i = 0; i < pillTypeCount; i++)
      {
        int count = 0;
        char line1[17], line2[17];
        sprintf(line1, "Pill %d quantity:", i+1);
        while (1)
        {
          LCD_writeStringXY(0, 0, line1);
          sprintf(line2, "count : %d    ", count);
          LCD_writeStringXY(1, 0, line2);
          if (keyNo == 1) { count++; keyNo = 0; }
          else if (keyNo == 2) { if (count > 0) count--; keyNo = 0; }
          else if (keyNo == 3) {
            pill_doses[i] = count;
            keyNo = 0;
            char msg[32];
            sprintf(msg, "[HKT_LIN]PILL%d_DOSE@%d\r\n", i+1, count);
            esp_send_data(msg);
            LCD_writeStringXY(0, 0, "                ");
            LCD_writeStringXY(1, 0, "                ");
            break;
          }
          HAL_Delay(50);
        }
      }
      set_check_2 = 1;
      printf("setend test\r\n");
      esp_send_data("[HKT_LIN]Setend\r\n");
    }

    if(strstr((char *)cb_data.buf,"+IPD") && cb_data.buf[cb_data.length-1] == '\n')
    {
      strcpy(strBuff,strchr((char *)cb_data.buf,'['));
      memset(cb_data.buf,0x0,sizeof(cb_data.buf));
      cb_data.length = 0;
      esp_event(strBuff);
    }
    if(rx2Flag)
    {
      printf("recv2 : %s\r\n",rx2Data);
      rx2Flag = 0;
    }

    if(tim2Flag1Sec)
    {
      tim2Flag1Sec = 0;
      if(!(tim2Sec%10))
      {
        if(esp_get_status() != 0)
        {
           if (!connect_status) {
              printf("server connecting ... (from periodic check)\r\n");
              esp_client_conn();
           }
        }
      }

      if(!(tim2Sec%5) && motorDone)
      {
        dht11Data = DHT11_readData();
        if(dht11Data.rh_byte1 != 255)
        {
          sprintf(buff,"h: %d%% t: %d.%d'C", dht11Data.rh_byte1, dht11Data.temp_byte1, dht11Data.temp_byte2);
          printf("%s\r\n", buff);
          LCD_writeStringXY(0, 0, buff);
          if (dht11Data.temp_byte1 > 30 || dht11Data.rh_byte1 > 70)
          {
        	  LCD_writeStringXY(0, 0, "Temp, Humi     ");
        	  LCD_writeStringXY(1, 0, "Warning !!!     ");
            sprintf(serverMsg, "[HKT_ARD]DHT@%d@%d.%d\r\n", dht11Data.rh_byte1, dht11Data.temp_byte1, dht11Data.temp_byte2);
            esp_send_data(serverMsg);
          }
        }
        else
          printf("DHT11 response error\r\n");
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  hi2c1.Init.ClockSpeed = 100000;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 38400;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, FANPIN_Pin|DHT11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON0_Pin BUTTON1_Pin BUTTON2_Pin */
  GPIO_InitStruct.Pin = BUTTON0_Pin|BUTTON1_Pin|BUTTON2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FANPIN_Pin DHT11_Pin */
  GPIO_InitStruct.Pin = FANPIN_Pin|DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void trim(char* str) {
    int len = strlen(str);
    while(len > 0 && (str[len-1] == '\n' || str[len-1] == '\r' || str[len-1] == ' ' || str[len-1] == ']')) {
        str[--len] = 0;
    }
}

void esp_event(char * recvBuf)
{
    int i=0;
    char * pToken;
    char * pArray[ARR_CNT]={0};

    if (strlen(recvBuf) > 0) {
        if (recvBuf[strlen(recvBuf)-1] == '\n' || recvBuf[strlen(recvBuf)-1] == '\r') {
            recvBuf[strlen(recvBuf)-1] = '\0';
        }
    }

    printf("\r\nDebug recv (esp_event): %s\r\n",recvBuf);

    pToken = strtok(recvBuf,"[@]");
    while(pToken != NULL)
    {
        pArray[i] = pToken;
        if(++i >= ARR_CNT) break;
        pToken = strtok(NULL,"[@]");
    }

    if (pArray[1] == NULL) {
        printf("ESP Event: Not enough tokens in message.\r\n");
        return;
    }

    if(!strncmp(pArray[1]," New conn",8))
    {
        connect_status = 1;
        printf("ESP Event: New connection established.\r\n");
        return;
    }
    else if(!strncmp(pArray[1]," Already log",8))
    {
        printf("ESP Event: Already logged in, attempting client connection.\r\n");
        esp_client_conn();
        return;
    }
    else if(!strcmp(pArray[1],"start"))
    {
        printf("Non-blocking motor start command received.\r\n");
        if (pArray[2] && pArray[3] && pArray[4]) {
            motor_target_repetitions[0] = atoi(pArray[2]);
            motor_target_repetitions[1] = atoi(pArray[3]);
            motor_target_repetitions[2] = atoi(pArray[4]);

            printf("Motor targets: M1=%d, M2=%d, M3=%d\r\n", motor_target_repetitions[0], motor_target_repetitions[1], motor_target_repetitions[2]);

            motor_completed_repetitions[0] = 0;
            motor_completed_repetitions[1] = 0;
            motor_completed_repetitions[2] = 0;

            motor_current_step = 0;
            motor_sequence_active = 0;

            for (int k=0; k<3; k++) {
                if (motor_target_repetitions[k] > 0) {
                    current_motor_idx = k;
                    motor_sequence_active = 1;
                    motorDone = 0;
                    printf("Starting sequence with motor %d (target reps: %d)\r\n", current_motor_idx + 1, motor_target_repetitions[k]);
                    break;
                }
            }

            if (!motor_sequence_active) {
                motorDone = 1;
                printf("No motor operations requested.\r\n");
            }
        } else {
            printf("Motor start command: missing parameters.\r\n");
        }
    }

    if(pArray[2]) trim(pArray[2]);

    if(pArray[1] && !strcmp(pArray[1],"FAN"))
    {
        printf("FAN command processing.\n");
        if(pArray[2] && !strcmp(pArray[2],"ON"))
        {
            LCD_writeStringXY(0, 0, "FAN ON !!!!     ");
            LCD_writeStringXY(1, 0, "                ");
            printf("FANON command received.\r\n");
            HAL_GPIO_WritePin(GPIOC, FANPIN_Pin, GPIO_PIN_SET);
            printf("FAN is ON.\r\n");
        }
        else if(pArray[2] && !strcmp(pArray[2],"OFF"))
        {
            LCD_writeStringXY(0, 0, "FAN OFF !!!     ");
            LCD_writeStringXY(1, 0, "                ");
            printf("FANOFF command received.\r\n");
            HAL_GPIO_WritePin(GPIOC, FANPIN_Pin, GPIO_PIN_RESET);
            printf("FAN is OFF.\r\n");
        }
    }
}

void process_motor_sequence(void) {
    if (!motor_sequence_active) {
        return;
    }

    uint32_t current_tick = HAL_GetTick();
    TIM_HandleTypeDef* timer_handle = &htim3;
    uint32_t channel;
    int motor_log_idx = current_motor_idx + 1;

    if (current_motor_idx == 0) channel = TIM_CHANNEL_1;
    else if (current_motor_idx == 1) channel = TIM_CHANNEL_2;
    else if (current_motor_idx == 2) channel = TIM_CHANNEL_3;
    else {
        printf("Error: Invalid current_motor_idx %d\n", current_motor_idx);
        motor_sequence_active = 0;
        motorDone = 1;
        return;
    }

    if (motor_completed_repetitions[current_motor_idx] >= motor_target_repetitions[current_motor_idx]) {
        printf("Motor %d finished all %d repetitions.\r\n", motor_log_idx, motor_target_repetitions[current_motor_idx]);
        __HAL_TIM_SetCompare(timer_handle, channel, 500);

        int next_motor_found = 0;
        for (int i = current_motor_idx + 1; i < 3; i++) {
            if (motor_target_repetitions[i] > 0) {
                current_motor_idx = i;
                motor_current_step = 0;
                printf("Switching to motor %d (target reps: %d).\r\n", current_motor_idx + 1, motor_target_repetitions[i]);
                next_motor_found = 1;
                break;
            }
        }

        if (!next_motor_found) {
            printf("All motor sequences complete.\r\n");
            motor_sequence_active = 0;
            motorDone = 1;
            __HAL_TIM_SetCompare(timer_handle, channel, 500);
            return;
        }
    }

    switch (motor_current_step) {
        case 0:
            printf("Motor %d (Rep %d/%d): Step 0 (PWM 500, start 1s wait)\r\n",
                   motor_log_idx, motor_completed_repetitions[current_motor_idx] + 1, motor_target_repetitions[current_motor_idx]);
            __HAL_TIM_SetCompare(timer_handle, channel, 500);
            motor_step_start_time = current_tick;
            motor_current_step = 1;
            break;

        case 1:
            if (current_tick - motor_step_start_time >= 1000) {
                printf("Motor %d: Step 1 complete (1s wait). Moving to Step 2.\r\n", motor_log_idx);
                motor_current_step = 2;
            }
            break;

        case 2:
            printf("Motor %d: Step 2 (PWM 1500, start 2s wait)\r\n", motor_log_idx);
            __HAL_TIM_SetCompare(timer_handle, channel, 1500);
            motor_step_start_time = current_tick;
            motor_current_step = 3;
            break;

        case 3:
            if (current_tick - motor_step_start_time >= 2000) {
                printf("Motor %d: Step 3 complete (2s wait). Moving to Step 4.\r\n", motor_log_idx);
                motor_current_step = 4;
            }
            break;

        case 4:
            printf("Motor %d: Step 4 (PWM 500, start 1s wait for cycle end)\r\n", motor_log_idx);
            __HAL_TIM_SetCompare(timer_handle, channel, 500);
            motor_step_start_time = current_tick;
            motor_current_step = 5;
            break;

        case 5:
            if (current_tick - motor_step_start_time >= 1000) {
                printf("Motor %d: Step 5 complete (1s wait, one repetition finished).\r\n", motor_log_idx);
                motor_completed_repetitions[current_motor_idx]++;

                if (motor_completed_repetitions[current_motor_idx] < motor_target_repetitions[current_motor_idx]) {
                    motor_current_step = 0;
                    printf("Motor %d: Starting next repetition (%d/%d).\r\n",
                           motor_log_idx, motor_completed_repetitions[current_motor_idx] + 1, motor_target_repetitions[current_motor_idx]);
                } else {
                    printf("Motor %d: All %d repetitions done. Next call will check for next motor/end sequence.\r\n",
                           motor_log_idx, motor_target_repetitions[current_motor_idx]);
                }
            }
            break;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int tim2Cnt = 0;
    if (htim->Instance == TIM2)
    {
        tim2Cnt++;
        if(tim2Cnt >= 1000)
        {
            tim2Flag1Sec = 1;
            tim2Sec++;
            tim2Cnt = 0;
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_tick0 = 0, last_tick1 = 0, last_tick2 = 0;
    uint32_t now = HAL_GetTick();

    switch(GPIO_Pin)
    {
        case BUTTON0_Pin:
            if(now - last_tick0 > 200) { keyNo = 3; last_tick0 = now; }
            break;
        case BUTTON1_Pin:
            if(now - last_tick1 > 200) { keyNo = 1; last_tick1 = now; }
            break;
        case BUTTON2_Pin:
            if(now - last_tick2 > 200) { keyNo = 2; last_tick2 = now; }
            break;
    }
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
