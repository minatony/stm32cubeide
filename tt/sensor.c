/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Accelerometer
#define ACCELEROMETER_READ  0x33
#define ACCELEROMETER_WRITE 0x32
#define LSM303AGR_CTRL_REG1_A               0x20  /* Control register 1 acceleration */
#define LSM303AGR_CTRL_REG2_A               0x21  /* Control register 2 acceleration */
#define LSM303AGR_CTRL_REG3_A               0x22  /* Control register 3 acceleration */
#define LSM303AGR_CTRL_REG4_A               0x23  /* Control register 4 acceleration */
#define LSM303AGR_BLE_MSB                 ((uint8_t)0x40) /*!< Big Endian: data MSB @ lower address */
#define LSM303AGR_ACC_SENSITIVITY_2G     ((float)0.0039)  /*!< accelerometer sensitivity with 2 g full scale [mg/LSB] */
// magnetic
#define MAGNETIC_READ 0x3D
#define MAGNETIC_WRITE 0x3C
#define LSM303AGR_CFG_REG_A_M               0x60  /* Configuration register A magnetic field */
#define LSM303AGR_CFG_REG_B_M               0x61  /* Configuration register B magnetic field */
#define LSM303AGR_CFG_REG_C_M               0x62  /* Configuration register C magnetic field */
// groscopes
#define L3GD20_CTRL_REG1_ADDR         0x20  /* Control register 1 */
#define L3GD20_CTRL_REG2_ADDR         0x21  /* Control register 2 */
#define L3GD20_CTRL_REG3_ADDR         0x22  /* Control register 3 */
#define L3GD20_CTRL_REG4_ADDR         0x23  /* Control register 4 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float ax[3] = {0};
float vx[3] = {0};
float sx = 0;
float Xt_prev_x = 0, Xt_prev_y = 0, Xt_prev_z = 9.8;
float Pt_prev_x = 1, Pt_x, Pt_prev_y = 1, Pt_prev_z = 1;
float accx, accy, accz;
float ref_x = 0, ref_y = 0, ref_z = 0;
uint8_t status = 0;
char Buf[10] = {0}, Buf1[10] = {0}, Buf2[10] = {0}, Buf3[10] = {0}, Buf4[10] = {0}, Buf5[10] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void GyroInit(void){
	uint8_t spiTxBuf[6] = {0x20 | 0b01000000, 0x0f, 0x00, 0x00, 0x20, 0x10};

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
	HAL_Delay(20);

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_SPI_Transmit(&hspi1,spiTxBuf,6,50);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
	HAL_Delay(20);
}
void GyroReadXYZ(void){
	float Data[3] = {0};
	uint8_t spiRxBuf[7] = {0};
	uint8_t spiTxBuf = 0x28 | 0xC0;
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&spiTxBuf,1,50);
	HAL_SPI_Receive(&hspi1,&spiRxBuf[0],6,50);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
	spiTxBuf = 0x23 | 0x80;
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &spiTxBuf, 1, 50);
	HAL_SPI_Receive(&hspi1, &spiRxBuf[7], 1, 50);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
	if(!(spiRxBuf[7] & 0b01000000)){
		for(uint8_t i=0; i<3; i++){
			Data[i]=((int16_t)((uint16_t)spiRxBuf[2*i+1] << 8) + spiRxBuf[2*i]) * 0.07;
		}
	}
	else{
	    for(uint8_t i=0; i<3; i++){
	    	Data[i]=((int16_t)((uint16_t)spiRxBuf[2*i] << 8) + spiRxBuf[2*i+1]) * 0.07;
	    }
	}
}
//void CaliGyro(){
//	for(uint8_t i = 0; i < 1000; i++){
//
//	}
//}
void MagInit(void){
	uint8_t i2cTxBuf = 0x8C;
	HAL_I2C_Mem_Write(&hi2c1, MAGNETIC_WRITE, LSM303AGR_CFG_REG_A_M, 1, &i2cTxBuf, 1, 1);
	i2cTxBuf = 0x03;
	HAL_I2C_Mem_Write(&hi2c1, MAGNETIC_WRITE, LSM303AGR_CFG_REG_B_M, 1, &i2cTxBuf, 1, 1);
	i2cTxBuf = 0x10;
	HAL_I2C_Mem_Write(&hi2c1, MAGNETIC_WRITE, LSM303AGR_CFG_REG_C_M, 1, &i2cTxBuf, 1, 1);
}
void MagReadXYZ(void){
	uint8_t mang2[6] = {0};
	uint8_t i2cBuf[2] = {0};
	float Data[3] = {0};
	i2cBuf[0] = 0xE8;
	HAL_I2C_Master_Transmit(&hi2c1, MAGNETIC_WRITE, &i2cBuf[0], 1, 1);
	HAL_I2C_Master_Receive(&hi2c1, MAGNETIC_READ, mang2, 6, 100);
	HAL_I2C_Mem_Read(&hi2c1, MAGNETIC_READ, LSM303AGR_CFG_REG_C_M, 1, &i2cBuf[1], 1, 1);
	if(!(i2cBuf[1] & 0b00001000)){
		for(uint8_t i=0; i<3; i++){
			Data[i]=((int16_t)((uint16_t)mang2[2*i+1] << 8) + mang2[2*i]) * 0.0015;
		}
	}
	else{
	    for(uint8_t i=0; i<3; i++){
	    	Data[i]=((int16_t)((uint16_t)mang2[2*i] << 8) + mang2[2*i+1]) * 0.0015;
	    }
	}
}
void AccInit(void){
	// thiet lap cam bien, bat cam bien, cho phep truc x, y, z (trang 30 LSM303ARG)
	uint8_t i2cTxBuf = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, ACCELEROMETER_WRITE, LSM303AGR_CTRL_REG2_A, 1, &i2cTxBuf, 1, 1);
	i2cTxBuf = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, ACCELEROMETER_WRITE, LSM303AGR_CTRL_REG3_A, 1, &i2cTxBuf, 1, 1);
	i2cTxBuf = 0x81;
	HAL_I2C_Mem_Write(&hi2c1, ACCELEROMETER_WRITE, LSM303AGR_CTRL_REG4_A, 1, &i2cTxBuf, 1, 1);
	i2cTxBuf = 0x57;
	HAL_I2C_Mem_Write(&hi2c1, ACCELEROMETER_WRITE, LSM303AGR_CTRL_REG1_A, 1, &i2cTxBuf, 1, 1);
}
void AccReadXYZ(float *calx, float *caly, float *calz){
	float nData[3];
	uint8_t mang[6] = {0};
	int16_t Data[3] = {0};
	uint8_t i2cTxBuf[2] = {0};
	i2cTxBuf[0] = 0xA8;
	HAL_I2C_Master_Transmit(&hi2c1, ACCELEROMETER_WRITE, &i2cTxBuf[0], 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, ACCELEROMETER_READ, mang, 6, 100);
	HAL_I2C_Mem_Read(&hi2c1, ACCELEROMETER_READ, LSM303AGR_CTRL_REG4_A, 1, &i2cTxBuf[1], 1, 100);
	if(!(i2cTxBuf[1] & LSM303AGR_BLE_MSB)) {
		for(uint8_t i=0; i<3; i++){
			Data[i]=((int16_t)((uint16_t)mang[2*i+1] << 8) + mang[2*i]);
		}
	}
	else{/* Big Endian Mode */
	    for(uint8_t i=0; i<3; i++){
	    	Data[i]=((int16_t)((uint16_t)mang[2*i] << 8) + mang[2*i+1]);
	    }
	}
	for(uint8_t i = 0; i < 3; i++){
		nData[i] = Data[i] * LSM303AGR_ACC_SENSITIVITY_2G;
	}
	// calibrating
	*calx = nData[0] * 0.0159 + nData[2] * -0.0007 + 0.0655;
	*caly = nData[0] * 0.0003 + nData[1] * 0.0151 - nData[2] * 0.0002 - 0.0217;
	*calz = nData[0] * 0.0007 - nData[1] * 0.0001 + nData[2] * 0.0151 + 0.0127;
}

void KalmanFilter(float *a, float *Xt_prev, float *Pt_prev, uint8_t Q, uint8_t R){
	float Xt_update = 0, Pt_update = 0, Kt = 0, Xt = 0, Pt = 0;
	*a = *a * 9.8;
	Xt_update = *Xt_prev;
	Pt_update = *Pt_prev + Q;
	Kt = Pt_update / (Pt_update + R);
	Xt = Xt_update + (Kt * (*a - Xt_update));
	Pt = (1 - Kt) * Pt_update;
	*Xt_prev = Xt;
	*Pt_prev = Pt;
	*a = Xt;
}
void nomove(float *ax2, float *ay2, float *az2){
	float t_ax = 0, t_ay = 0, t_az = 0;
	for(uint8_t count = 0; count < 100; count++){
		AccReadXYZ(ax2, ay2, az2);
		t_ax += *ax2;
		t_ay += *ay2;
		t_az += *az2;
	}
	*ax2 = t_ax / 100;
	*ay2 = t_ay / 100;
	*az2 = t_az / 100;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	AccReadXYZ(&accx, &accy, &accz);
	accx = accx - ref_x;
	accy = accy - ref_y;
	accz = accz - ref_z;
	KalmanFilter(&accx, &Xt_prev_x, &Pt_prev_x, 1, 100);
	KalmanFilter(&accy, &Xt_prev_y, &Pt_prev_y, 1, 100);
	KalmanFilter(&accz, &Xt_prev_z, &Pt_prev_z, 1, 100);
	if((accx < 0.05) && (accx > -0.07)){
		accx = 0;
	}
	// trang thai

	status++;

	sprintf(Buf, "%.4f,", sx);
	HAL_UART_Transmit(&huart2, (uint8_t *)Buf, strlen(Buf), 100);
	sprintf(Buf2, "%.4f\n", accx);
	HAL_UART_Transmit(&huart2, (uint8_t *)Buf2, strlen(Buf2), 100);
//	sprintf(Buf1, "%.4f\n", ay);
//	HAL_UART_Transmit(&huart2, (uint8_t *)Buf1, strlen(Buf1), 100);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
//	float denta_vx = 0;
	uint8_t count = 0, count1 = 0;
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
  MX_TIM1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  AccInit();
  HAL_Delay(90);
  nomove(&ref_x, &ref_y, &ref_z);
  HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  denta_vx = (5 * ax[0] + 8 * ax[1] - ax[2]) / 1200;
//	  vx[count + 1] = vx[count] + denta_vx;
//	  count++;
//	  ax[0] = ax[1];
//	  ax[1] = ax[2];
//	  while(!status);
//	  ax[2] = accx;
//	  status = 0;
//	  if(count == 2){
//		  sx = sx + (5 * vx[0] + 8 * vx[1] - vx[2]) / 1200;
//		  count = 0;
//	  }
//	  vx[0] = vx[1];
//	  // dung
//	  if(accx == 0){
//		  count1++;
//	  }
//	  else{
//		  count1 = 0;
//	  }
//	  if(count1 == 8){
//		  vx[0] = 0;
//		  vx[1] = 0;
//		  vx[2] = 0;
//		  ax[0] = 0;
//		  ax[1] = 0;
//		  ax[2] = 0;
//	  }
//	  status = 0;
	  /////////////////////////////////////////////////////////////
//	  sprintf(Buf1, "%.4f\n", accx);
//	  HAL_UART_Transmit(&huart2, (uint8_t *)Buf1, strlen(Buf1), 100);
//	  sprintf(Buf2, "%.4f,", ax[0]);
//	  HAL_UART_Transmit(&huart2, (uint8_t *)Buf2, strlen(Buf2), 100);
//	  sprintf(Buf3, "%.4f,", ax[1]);
//	  HAL_UART_Transmit(&huart2, (uint8_t *)Buf3, strlen(Buf3), 100);
//	  sprintf(Buf4, "%.4f\n", ax[2]);
//	  HAL_UART_Transmit(&huart2, (uint8_t *)Buf4, strlen(Buf4), 100);
	  /////////////////////////////////////////////////////////////
    troi nhieu
    ////////////////////////////////////////////////////////////
	  while(!status);
	  vx[1] = vx[0] + 0.01 * accx;
	  sx = sx + vx[0] * 0.01 + 0.00005 * accx;
	  vx[0] = vx[1];
	  status = 0;
	  if(accx == 0){
		  count1++;
	  }
	  else{
		  count1 = 0;
	  }
	  if(count1 > 4){
		  vx[1] = 0;
	  }
	  /////////////////////////////////////////////////////////// lech vai met, moi lan do cho ra ket qua khac nhau
//	  while(!status);
//	  ax[1] = accx;
//	  vx[1] = ax[1] + abs(ax[1] - ax[0]) * 0.005;
//	  sx = sx + vx[1] + abs(vx[1] - vx[0]) * 0.005;
//	  ax[0] = ax[1];
//	  vx[0] = vx[1];
//	  status = 0;
//	  if(accx == 0){
//		  count1++;
//	  }
//	  else{
//		  count1 = 0;
//	  }
//	  if(count1 > 4){
//		  vx[1] = 0;
//	  }
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim1.Init.Prescaler = 1599;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
