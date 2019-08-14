/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "string.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SLV_ADD 0x68 //must append the RW bit to the end, address is actually 68, but becomes D0
#define INIT_REG 0x75
#define PWR_REG 0x6B
#define SMPL_Rate_REG 0x19
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define GYRO_REG 0x43
#define ACCEL_REG 0x3B
#define TEMP_REG 0x41
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//for bit shifting right because HAL_I2C auto appends the RW bit
uint16_t MPU_ADD =(uint16_t)SLV_ADD;
//data
float AccelX;
float AccelY;
float AccelZ;
float GyroX;
float GyroY;
float GyroZ;
float Temp;
//raw variables
int16_t AccelXR;
int16_t AccelYR;
int16_t AccelZR;
int16_t GyroXR;
int16_t GyroYR;
int16_t GyroZR;
int16_t TempR;
//Uart char array variables(for debugging)
char AXC[100];
char AYC[100];
char AZC[100];
char GXC[100];
char GYC[100];
char GZC[100];
char TC[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void MPU_Init();
//function to get gyroscope data into an array
void getGyro();
//function to get accelerometer data into an array
void getAccel();
//function to get temperature data
void getTemp();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void MPU_Init(void){
	uint8_t check,data;
	//verify device address
		//1.Write to the register you want to read from
	HAL_I2C_Mem_Read(&hi2c1,MPU_ADD<<1,INIT_REG,1,&check,1,100);
	if(check ==104){ //if adress is correct
		data = 0;
		//wake up the sensor
		HAL_I2C_Mem_Write(&hi2c1,MPU_ADD<<1,PWR_REG,1,&data,1,1000);
		data = 0x07; //will yield a sample rate of 1khz
		//write to the register to set data rate, gyro is by default 8khz, send 7+1 and divide yields 1 - check data sheet
		HAL_I2C_Mem_Write(&hi2c1,MPU_ADD<<1,SMPL_Rate_REG,1,&data,1,1000);
		//config gyro and accel (by sending 0 to them), this sets limits as defined in the data sheet on the data we read ex) Accel is within +-2g
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1,MPU_ADD<<1,ACCEL_CONFIG,1,&data,1,1000);
		HAL_I2C_Mem_Write(&hi2c1,MPU_ADD<<1,GYRO_CONFIG,1,&data,1,1000);
	}

}

void getGyro(){
	//declare a buffer
uint8_t gyrobuf[6];
	//read sensor into the buffer
HAL_I2C_Mem_Read(&hi2c1,MPU_ADD<<1,GYRO_REG,1,gyrobuf,6,1000);
// move the raw data out of the buffers - into our raw variables
GyroXR = (int16_t)((gyrobuf[0]<<8) ^ gyrobuf[1]);
GyroYR = (int16_t)((gyrobuf[2]<<8) ^ gyrobuf[3]);
GyroZR = (int16_t)((gyrobuf[4]<<8) ^ gyrobuf[5]);
// correcting according to our config settings(0) refer to the data sheet
GyroX =((float)GyroXR) / 131.0;
GyroY = ((float)GyroYR) / 131.0;
GyroZ = ((float)GyroZR) / 131.0;

}

void getAccel(){
	//declare a buffer
	uint8_t accelbuf[6];
		//read sensor into the buffer
	HAL_I2C_Mem_Read(&hi2c1,MPU_ADD<<1,ACCEL_REG,1,accelbuf,6,1000);
	// move the raw data out of the buffers - into our raw variables
	AccelXR = (int16_t)((accelbuf[0]<<8) ^ accelbuf[1]);
	AccelYR = (int16_t)((accelbuf[2]<<8) ^ accelbuf[3]);
	AccelZR = (int16_t)((accelbuf[4]<<8) ^ accelbuf[5]);
	// correcting according to our config settings(0) refer to the data sheet
	AccelX = ((float)AccelXR) / 16384.0;
	AccelY = ((float)AccelYR) / 16384.0;
	AccelZ = ((float)AccelZR) / 16384.0;

}

void getTemp(){
	uint8_t tempbuf[2];
	HAL_I2C_Mem_Read(&hi2c1,MPU_ADD<<1,TEMP_REG,1,tempbuf,2,1000);
	//moving data out of the buffer into our raw variables
	TempR = (int16_t)((tempbuf[0]<<8) ^ tempbuf[1]);
	Temp = (((float)TempR)/340) +36.53; //temperature in celsius- refer to the data sheet
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
 if( HAL_I2C_IsDeviceReady(&hi2c1,MPU_ADD<<1,2,10)== HAL_OK){
	 char DB[100];
	 sprintf(DB,"Device is Ready\n");
	 HAL_UART_Transmit(&huart2,(uint8_t*)DB,strlen(DB),100);
 }
  MPU_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  //reset readings
AccelX=0;
AccelY=0;
AccelZ=0;
GyroX=0;
GyroY=0;
GyroZ=0;
Temp=0;
	  //read the accelerometer
getAccel();
	  //read the gyroscope
getGyro();
	  //read the temperature
getTemp();
	  //transmit data over Uart for debugging
//1. Because Attolic is stupid and wont print floats, we scale by 100 and typecast to int
int AccelXI=(int)(AccelX*100);
int AccelYI=(int)(AccelY*100);
int AccelZI=(int)(AccelZ*100);
int GyroXI=(int)(GyroX*100);
int GyroYI=(int)(GyroY*100);
int GyroZI=(int)(GyroZ*100);
int TempI=(int)(Temp*100);
//2.Sprintf prints into a character array - we can use this to convert our ints into chars.
sprintf(AXC,"AccelX: %i\r\n",AccelXI);
sprintf(AYC,"AccelY: %i\r\n",AccelYI);
sprintf(AZC,"AccelZ: %i\r\n",AccelZI);
sprintf(GXC,"GyroX: %i\r\n",GyroXI);
sprintf(GYC,"GyroY: %i\r\n",GyroYI);
sprintf(GZC,"GyroZ: %i\r\n",GyroZI);
sprintf(TC,"Temp(C): %i\r\n",TempI);
//3. Transmit the string over uart
HAL_UART_Transmit(&huart2,(uint8_t*)AXC,strlen(AXC),100);

HAL_UART_Transmit(&huart2,(uint8_t*)AYC,strlen(AYC),100);

HAL_UART_Transmit(&huart2,(uint8_t*)AZC,strlen(AZC),100);

HAL_UART_Transmit(&huart2,(uint8_t*)GXC,strlen(GXC),100);

HAL_UART_Transmit(&huart2,(uint8_t*)GYC,strlen(GYC),100);

HAL_UART_Transmit(&huart2,(uint8_t*)GZC,strlen(GZC),100);

HAL_UART_Transmit(&huart2,(uint8_t*)TC,strlen(TC),100);
HAL_Delay(10000);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
