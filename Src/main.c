/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
	* @author         : VincentChan (software) | JasonNgai (mechanical)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "stdbool.h" 
#include "mpu6050.h"
#include "pca9685_servo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN(x,y) ((x<y)?x:y)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;                   

volatile int timer_counter = 0;              //timer interrupt counter

volatile bool MPU_Sampling = false;          //flag to control sampling of MPU6050
bool average_filter = false;                 //flag to control average filtering
bool plot_curve = true;                      //flag to enable/disable curve plotting

int counter = 0;                             //sample taking counter
const int num_samples = 200;                 //number of samples for taking average

double Kalman_X_buffer = 0;                  //Storing num_samples of Kalman_X to perform averaging
double Kalman_Y_buffer = 0;                  //Storing num_samples of Kalman_Y to perform averaging

double offset_x;                             //noise remoiving from Kalman_X
double offset_y;                             //noise removing from Kalman_Y

double X_output;                             //Resultant X
double Y_output;                             //Resultant Y


enum servo_motor_type{raw_servo=0,pitch_servo=2};       //Servo motor index related to PCA9685

const int raw_init_angle = 90;                           //Initial raw angle for stabilizer
const int pitch_init_angle = 90;                         //Initial pitch angle for stabilizer

const int pitch_max = 30;                               //Mechanical limitation (please confirm with ME designer)
const int pitch_min = -30;                              //Mechanical limitation (please confirm with ME designer)

const int raw_max = 15;                                 //Mechanical limitation (please confirm with ME designer)
const int raw_min = -15;                                //Mechanical limitation (please confirm with ME designer)


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
float trimf(float measurement, float start, float peak, float end);   //Fuzzy Logic: triangular membership function
float Rmf(float measurement, float top, float bottom);                //Fuzzy Logic: R membership function
float Lmf(float measurement, float bottom, float top);                //Fuzzy Logic: L membership function

int raw_FLC(double error);          //raw fuzzy logic controller
int pitch_FLC(double error);        //pitch fuzzy logic controller
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
 #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,0xFFFF);
	return ch;
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	//Print system initialization message
  printf("[SYSTEM] System starts...\r\n");
	printf("[SYSTEM] Connecting to MPU6050...\r\n");
	
	//Connect MPU6050
  while (MPU6050_Init(&hi2c1) == 1);
	
	//Enable Timer 4 interrupt 
	HAL_TIM_Base_Start_IT(&htim4);
	
	//Print MPU6050 connected message
	printf("[SYSTEM] Connection established...\r\n");
	
  //Noted that the libraries control the servo motor from 0 - 180
	PCA9685_Go();                                               //PCA9685 Initialization
	SetPWMFreq(50);                                             //Set Servo PWM Frequency
  setServo(pitch_servo,calculate_PWM(pitch_init_angle));      //Set init raw
	setServo(raw_servo,calculate_PWM(pitch_init_angle));        //Set init pitch
	HAL_Delay(200);                                             //Delay for servo mechanical response
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(MPU_Sampling)
		{
			//Read MPU6050 value and processed by Kalman Filter
      MPU6050_Read_All(&hi2c1,&MPU6050);
			
			//Average Filter
			if(!average_filter)
			{
				//When average filer is not executed
				Kalman_Y_buffer += MPU6050.KalmanAngleY;
				Kalman_X_buffer += MPU6050.KalmanAngleX;
				if(!plot_curve)
				{
				  printf("[LOG] Counter:%d | Kalman Filter: Enabled | Average Filter: Pending\r\n",counter);
				  printf("[LOG] X: %.2f degree | Y: %.2f degree\r\n",MPU6050.KalmanAngleX,MPU6050.KalmanAngleY);
				  printf("[SYSTEM]Please do not move your gyro sensor!\r\n");
				}else{
					printf("$%d %d;\r\n",(int)MPU6050.KalmanAngleX,(int)MPU6050.KalmanAngleY);
				}
			}else{
				//When average filter is ready
				X_output = MPU6050.KalmanAngleX - offset_x;
				Y_output = MPU6050.KalmanAngleY - offset_y;
				if(!plot_curve)
				{
				  printf("[LOG] Smoothed X: %.2f | Smoothed Y: %.2f\r\n",X_output,Y_output);
				}else{
					printf("$%d %d;\r\n",(int)X_output,(int)Y_output);
				}
			}
			
			//Find average offset
			if((counter==num_samples)&&(!average_filter))
			{
				//These statements will only be executed once after taking num_samples, then disable.
				average_filter = true;
				offset_x = Kalman_X_buffer/num_samples;
				offset_y = Kalman_Y_buffer/num_samples;
			}
			printf("\r\n");
			counter++;
		}
		
		//TO-DO: Balancing Control
		
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1024;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 703;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ONBOARD_LED_GPIO_Port, ONBOARD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ONBOARD_BUTTON_Pin */
  GPIO_InitStruct.Pin = ONBOARD_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ONBOARD_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ONBOARD_LED_Pin */
  GPIO_InitStruct.Pin = ONBOARD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ONBOARD_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim4)
	{
	  timer_counter++;               //  Interval = 0.01 second	
		if ((timer_counter%5) == 0)    //  Every 0.05 second
	  {
		  MPU_Sampling = !MPU_Sampling;
			HAL_GPIO_TogglePin(ONBOARD_LED_GPIO_Port,ONBOARD_LED_Pin);
	  }	
	}
}

float trimf(float measurement, float start, float peak, float end)
{
	float fx = 0.0;
	//Triangular Membership Function
	//Remember: end > peak > start (these values are indicating x axis)
	if(measurement <= start) fx=0;
	if((measurement > start)&&(measurement <= peak))
	{
		fx = (measurement-start)/(peak-start);
	}
	if((measurement < end)&&(measurement > peak))
	{
		fx = (end-measurement)/(end-peak);
	}
	if(measurement >= end) fx = 0;
	return fx;	
}

float Rmf(float measurement, float top, float bottom)
{
	float fx = 0.0;
	//Special Trapezodial function: R-function
	//Remember: bottom > top (these values are indicating x axis)
	if(measurement>bottom) fx = 0;
	if(measurement<top) fx = 1;
	if((measurement<=bottom)&&(measurement>=top))
	{
		fx = (bottom-measurement)/(bottom-top);
	}
	return fx;
}

float Lmf(float measurement, float bottom, float top)
{
	float fx = 0.0;
	//Special Trapezodial function: L-function
	//Remember: top > bottom (these values are indicating x axis)
	if(measurement<bottom) fx = 0;
	if(measurement>top) fx = 0;
	if((measurement>=bottom)&&(measurement<=top))
	{
		fx = (measurement-bottom)/(top-bottom);
	}
	return fx;
}

int pitch_FLC(double error)
{
	int servo_adjust = 0;
	//This controller should try to keep Y_output = 0 (balance)
	
	//1.Fuzzification
	
	//define pitch input membership functions and levels
	//1. LN -> Largely negative (the object is sliding down rapidly)
	//2. N  -> Negative         (the object is sliding down slowly)
	//3. Z  -> Zero             (the object is still stationary)
	//4. P  -> Positive         (the object is sliding down slowly)
	//5. LP -> Largely positive (the object is sliding down rapidly)
	
	
  //2.Inferencing
	
	
	//3.Defuzzification: Weighted Average Method
	
	//define servo adjustment membership functions and levels
	//1. LXC -> Largely clockwise
	//2. XC  -> Clockwise        
	//3. UC  -> Unchanged             
	//4. CC  -> Counterclockwise        
	//5. LCC -> Largely counterclockwise	
	
	return servo_adjust;
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
  printf("Fatal Error -> Please contact Vincent or Jason\r\n");
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
