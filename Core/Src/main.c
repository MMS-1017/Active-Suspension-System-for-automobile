/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "pid_mpu.h"
#include <stdlib.h>

uint8_t Recieved_Data[1];
int pitch;
uint8_t recieved_check;
int reciv;

//PID_TypeDef motor1_pid;
//PID_TypeDef motor2_pid;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

Motor_t MotorForwardSuspension={

		forwardSus_motor,100,0,0,
		//extl
		// input
		mf_enc2_Pin,
		mf_enc2_GPIO_Port,
		0,
		0,
		//Directions
		mf_dir1_Pin,
		mf_dir1_GPIO_Port,

		mf_dir2_Pin,
		mf_dir2_GPIO_Port,
		// pwm
		TIM_CHANNEL_1,
		&htim5,
		0,0,Backward_move,stop,0
};
Motor_t MotorBackwardSuspension={

		BackwardSus_motor,100,0,0,
		//extl
		mb_enc2_Pin,
		mb_enc2_GPIO_Port,
		0,
		0,
		//input
		mb_dir1_Pin,
		mb_dir1_GPIO_Port,
		mb_dir2_Pin,
		mb_dir2_GPIO_Port,
		//pwm
		TIM_CHANNEL_1,
		&htim2,
		0,0,Backward_move,stop,0
};
/*---------------------------------------------------------*/
/*    Intit Speed motors     */
/*---------------------------------------------------------*/

Motor_t MotorRight={
		Right_motor,100,
		0,0,
		//extl
		mr_enc2_Pin,
		mr_enc2_GPIO_Port,
		0,
		0,
		//input
		mr_dir1_Pin,
		mr_dir1_GPIO_Port,
		mr_dir2_Pin,
		mr_dir2_GPIO_Port,
		//pwm
		TIM_CHANNEL_1,
		&htim3,
		0,0,stop,up_move,0
};


Motor_t MotorLeft={
		Left_motor,100
		,TIM_CHANNEL_4,&htim3,
		//extl
		ml_enc2_Pin,
		ml_enc2_GPIO_Port,
		0,
		0,
		//input
		ml_dir1_Pin,
		ml_dir1_GPIO_Port,
		ml_dir2_Pin,
		ml_dir2_GPIO_Port,
		//pwm
		TIM_CHANNEL_3,
		&htim2,
		0,0,stop,up_move,0

};
void Control_Angle_MPU_Y();

void control_hight(Motor_t* m, int hight)
{

    int Ticks = (hight * 4.5 );
	Position_GetNewTarget(m,Ticks,80 );
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
	uint8_t bitstatus1 ;
	  if (GPIO_Pin == mb_enc1_Pin)
		  {

				bitstatus1 = HAL_GPIO_ReadPin(MotorBackwardSuspension.Encoder2_port, MotorBackwardSuspension.Encoder2_pin);




				if (bitstatus1 == GPIO_PIN_RESET)
				{

					MotorBackwardSuspension.CurrentPosition++;
				}
				else
				{
					MotorBackwardSuspension.CurrentPosition--;

				}
		  }
		  else if (GPIO_Pin == mf_enc1_Pin)
		  {

				bitstatus1 = HAL_GPIO_ReadPin(MotorForwardSuspension.Encoder2_port, MotorForwardSuspension.Encoder2_pin);


				if (bitstatus1 == GPIO_PIN_RESET)
				{
					MotorForwardSuspension.CurrentPosition++;
				}
				else
				{
					MotorForwardSuspension.CurrentPosition--;
				}
		  }

//  if (GPIO_Pin == mb_enc1_Pin)
//  {
//	  MotorBS_ICU(&MotorBackwardSuspension);
//  }
//  else if (GPIO_Pin == mf_enc1_Pin)
//  {
//	  MotorFS_ICU(&MotorForwardSuspension);
//  }
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}

//initial variables of the speed of the motor
int speed_l =0;
int speed_r=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

static uint8_t cnt =0;
	/******* (Get angle Pitch)*****************/
	// 	MPUControlMotor(&MotorBackwardSuspension, mpu1.KalmanAngleX) ;
	// 	MPUControlMotor(&MotorForwmardSuspension, mpu1.KalmanAngleX) ;

	/**______________________________________*/

	if (htim == &htim1)   //trigger every 2.048ms
	{
//		 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
		Position_Calc_Signal(&MotorForwardSuspension);
		Position_Calc_Signal(&MotorBackwardSuspension);

		if(cnt == 25)
		{
		//reset current positions for speed motors
			speed_l = MotorLeft.CurrentPosition ;
		MotorLeft.CurrentPosition = 0;
			speed_r	=MotorRight.CurrentPosition;
			MotorRight.CurrentPosition = 0;
		cnt = 0;
		}
		else cnt ++;
	}

}





/*---------------------------------------------------------*/
/*---------------------------------------------------------*/


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

//functions depending on the encoder mode to measure speed of the motor
void readEncoders(void);
void setMotorSpeed(TIM_HandleTypeDef *htim, uint32_t Channel, double speed);

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
	pitch = 0;



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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	SCB->CPACR |= ((3UL << 20U)|(3UL << 22U));  /* set CP10 and CP11 Full Access */

	//init extl for Position motor encoders
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);  //Motor Backward / forward
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);  //Motor left
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);  //Motor right
	//init interrupt of input capture


	//*********** position **********/

	// functions for motor initialzation
	init_motors(&MotorForwardSuspension);
	init_motors(&MotorBackwardSuspension);

	/*********** speed *************/
	init_motors(&MotorRight);
	init_motors(&MotorLeft);

	DriveMotorDirectionSPeed(&MotorLeft);
	DriveMotorDirectionSPeed(&MotorRight);
//	HAL_TIM_IC_Start_IT(&htim14, TIM_CHANNEL_1);//mr
//	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);//ml
	// 0.43, 1.75,0.1)
	init_PIDVal(&MotorForwardSuspension,0.2, 5,0.1);  //the fshe5aaa one
	init_PIDVal(&MotorBackwardSuspension,1.475, 5,0.0134);  //the fshe5aaa one


	HAL_TIM_Base_Start_IT(&htim1);   // Generate interrupt
	/*   starting  USART1     */

	HAL_Delay(500);
	Position_GetNewTarget(&MotorForwardSuspension, 0,0);
	Position_GetNewTarget(&MotorBackwardSuspension, 0,0);
	HAL_Delay(1000);


	Position_GetNewTarget(&MotorBackwardSuspension, 150,65);
	Position_GetNewTarget(&MotorForwardSuspension, -350,65);
	HAL_Delay(3000);
	HAL_TIM_Base_Stop_IT(&htim1);
	MotorBackwardSuspension.Target_position = 0;
	MotorBackwardSuspension.CurrentPosition = 0;

	MotorForwardSuspension.Target_position = 0;
	MotorForwardSuspension.CurrentPosition = 0;
	HAL_TIM_Base_Start_IT(&htim1);   // Generate interrupt


//#ifdef mpu
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
//	while (MPU6050_Init(&hi2c1) == 1);
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
//#endif

	MotorLeft.Direction_main = Forward_move;
	MotorRight.Direction_main = Forward_move;

	// drive the right and left motors
	DriveMotorDirectionSPeed(&MotorLeft);
	DriveMotorDirectionSPeed(&MotorRight);



	// functions to control speed of the right and left motors
	int speed = 0;

	 __HAL_TIM_SET_COMPARE(MotorLeft.PWM_Timer, MotorLeft.PWM_channel,100);
	 __HAL_TIM_SET_COMPARE(MotorRight.PWM_Timer, MotorRight.PWM_channel,100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		HAL_UART_Receive(&huart1, &recieved_check, 1, HAL_MAX_DELAY);
		if (recieved_check == '1')
		{
			//Receive the following two characters
			HAL_UART_Receive(&huart1, Recieved_Data, 2, HAL_MAX_DELAY);
			//second controller transmits th actual angle +150.
		    reciv =(Recieved_Data[0]-48)*10+(Recieved_Data[1] -48) -50 ;

		    //control active suspension

		    if (reciv < 0)
		    {
		    	//upward the back suspension
		    	MPUControlMotor(&MotorBackwardSuspension, -reciv);

		    }
		    else
		    {    //downward the forward suspension
		    	 MPUControlMotor(&MotorForwardSuspension, -reciv);
		    }


		    HAL_Delay(100);
		}


/*
		do{

 		HAL_UART_Receive(&huart1, &Recieved_Data, 1, HAL_MAX_DELAY);
		if (Recieved_Data >=48 && Recieved_Data <=57)
		pitch = pitch *10 + (Recieved_Data-48)  ;
		count++;

		}	while (Recieved_Data != '\0');
*/


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim1.Init.Period = 32767;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 800-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 800-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 800-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim5.Init.Period = 99;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart6.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, mf_dir1_Pin|mf_dir2_Pin|mb_dir1_Pin|mb_dir2_Pin
                          |mr_dir1_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, mr_dir2_Pin|ml_dir1_Pin|ml_dir2_Pin|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : mf_dir1_Pin mf_dir2_Pin mb_dir1_Pin mb_dir2_Pin
                           mr_dir1_Pin PA8 */
  GPIO_InitStruct.Pin = mf_dir1_Pin|mf_dir2_Pin|mb_dir1_Pin|mb_dir2_Pin
                          |mr_dir1_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : mr_dir2_Pin ml_dir1_Pin ml_dir2_Pin PB12 */
  GPIO_InitStruct.Pin = mr_dir2_Pin|ml_dir1_Pin|ml_dir2_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MPU_int_Pin mr_enc1_Pin mb_enc1_Pin mf_enc1_Pin */
  GPIO_InitStruct.Pin = MPU_int_Pin|mr_enc1_Pin|mb_enc1_Pin|mf_enc1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ml_enc2_Pin mr_enc2_Pin */
  GPIO_InitStruct.Pin = ml_enc2_Pin|mr_enc2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ml_enc1_Pin */
  GPIO_InitStruct.Pin = ml_enc1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ml_enc1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : mb_enc2_Pin mf_enc2_Pin */
  GPIO_InitStruct.Pin = mb_enc2_Pin|mf_enc2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

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
