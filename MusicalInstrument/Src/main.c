/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define ARM_MATH_CM4
#include "arm_math.h"
#include <stdio.h>
#include <stm32l475e_iot01_accelero.h>


#define pi 3.14159265359


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

int16_t accelero[3];
char buffer[100];
int volume_counter=0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int isPlaying=0;
int isRecording=0;
int isPlayingRecording=0;

//C4: period=3.89ms
int phase_1=0;
int period_1=389;
uint16_t c4_vol3[389];
uint16_t c4_vol2[389];
uint16_t c4_vol1[389];

//E4: period=3.03ms
int phase_2=0;
int period_2=303;
uint16_t e4_vol3[303];
uint16_t e4_vol2[303];
uint16_t e4_vol1[303];

//G4: period=2.55ms
int phase_3=0;
int period_3=255;
uint16_t g4_vol3[255];
uint16_t g4_vol2[255];
uint16_t g4_vol1[255];

//A4: period=2.27ms
int phase_4=0;
int period_4=227;
uint16_t a4_vol3[227];
uint16_t a4_vol2[227];
uint16_t a4_vol1[227];

//C5: period=1.91ms
int phase_5=0;
int period_5=191;
uint16_t c5_vol3[191];
uint16_t c5_vol2[191];
uint16_t c5_vol1[191];

//E5: period=1.52ms
int phase_6=0;
int period_6=152;
uint16_t e5_vol3[152];
uint16_t e5_vol2[152];
uint16_t e5_vol1[152];


//G5: period=1.28ms
int phase_7=0;
int period_7=128;
uint16_t g5_vol3[128];
uint16_t g5_vol2[128];
uint16_t g5_vol1[128];

int recording[1000];
int recordingLength=0;

char C4_block[]= "    ";
char E4_block[]= "            ";
char G4_block[]= "                    ";
char A4_block[]= "                            ";
char C5_block[]= "                                    ";
char E5_block[]= "                                            ";
char G5_block[]= "                                                    ";


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN){
       if(GPIO_PIN == myButton_Pin){

    	   if(isPlaying==0){
    		   isPlaying=1;
    	   }else if(isPlaying==1){
    		   if(isRecording==0){
    			   isRecording=1;
    			   HAL_GPIO_TogglePin(myLED_GPIO_Port, myLED_Pin);
    			   //starts recording
    		   }else{
    			   isRecording=0;
    			   isPlayingRecording=1;
    			   HAL_GPIO_TogglePin(myLED_GPIO_Port, myLED_Pin);
    			   //stop recording
    			   //starts playing
    			   //isPlayingRecording=0;
    			   //recordingLength=0;
    		   }
    	   }

       } else {
              __NOP();
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  BSP_ACCELERO_Init();
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);



  for(int i=0;i<period_1;i++){
	  float radians=2*pi*(i%period_1)/period_1;
	  c4_vol1[i]=3000*(arm_sin_f32(radians)+1)/2;
	  c4_vol2[i]=3500*(arm_sin_f32(radians)+1)/2;
	  c4_vol3[i]=4000*(arm_sin_f32(radians)+1)/2;
  }

  for(int i=0;i<period_2;i++){
	  float radians=2*pi*(i%period_2)/period_2;
	  e4_vol1[i]=3000*(arm_sin_f32(radians)+1)/2;
	  e4_vol2[i]=3500*(arm_sin_f32(radians)+1)/2;
      e4_vol3[i]=4000*(arm_sin_f32(radians)+1)/2;
  }

  for(int i=0;i<period_3;i++){
	  float radians=2*pi*(i%period_3)/period_3;
	  g4_vol1[i]=3000*(arm_sin_f32(radians)+1)/2;
	  g4_vol2[i]=3500*(arm_sin_f32(radians)+1)/2;
      g4_vol3[i]=4000*(arm_sin_f32(radians)+1)/2;
  }

  for(int i=0;i<period_4;i++){
      float radians=2*pi*(i%period_4)/period_4;
      a4_vol1[i]=3000*(arm_sin_f32(radians)+1)/2;
      a4_vol2[i]=3500*(arm_sin_f32(radians)+1)/2;
      a4_vol3[i]=4000*(arm_sin_f32(radians)+1)/2;
  }

  for(int i=0;i<period_5;i++){
      float radians=2*pi*(i%period_5)/period_5;
      c5_vol1[i]=3000*(arm_sin_f32(radians)+1)/2;
      c5_vol2[i]=3500*(arm_sin_f32(radians)+1)/2;
      c5_vol3[i]=4000*(arm_sin_f32(radians)+1)/2;
  }

  for(int i=0;i<period_6;i++){
      float radians=2*pi*(i%period_6)/period_6;
      e5_vol1[i]=3000*(arm_sin_f32(radians)+1)/2;
      e5_vol2[i]=3500*(arm_sin_f32(radians)+1)/2;
      e5_vol3[i]=4000*(arm_sin_f32(radians)+1)/2;
  }

  for(int i=0;i<period_7;i++){
      float radians=2*pi*(i%period_7)/period_7;
      g5_vol1[i]=3000*(arm_sin_f32(radians)+1)/2;
      g5_vol2[i]=3500*(arm_sin_f32(radians)+1)/2;
      g5_vol3[i]=4000*(arm_sin_f32(radians)+1)/2;
  }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  for(int i=0;i<100;i++){
		  buffer[i]='\0';
	  }

	  BSP_ACCELERO_AccGetXYZ(accelero);
	  int a1=accelero[0];
	  int a2=accelero[1];
	  int a3=accelero[2];

	  //sprintf(buffer, " [accelero:%d,%d,%d] \r\n", a1,a2,a3);
	  //HAL_UART_Transmit(&huart1, (uint8_t *)&buffer, sizeof(buffer), HAL_MAX_DELAY);


	  for(int i=0;i<100;i++){
		  buffer[i]='\0';
	  }

	  if(isPlaying==1 && isPlayingRecording==0){
		  if(a1>875){
			  switch (volume_counter){
			  	  case 0:
				  	  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,c4_vol1,period_1,DAC_ALIGN_12B_R);
				  	  break;
			  	  case 1:
			  		  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,c4_vol2,period_1,DAC_ALIGN_12B_R);
			  		  break;
			  	  case 2:
			  		  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,c4_vol3,period_1,DAC_ALIGN_12B_R);
			  		  break;
			  }
			  sprintf(buffer, "[volume:%d]%s|||||||| \r\n",(volume_counter+1),C4_block);

			  //if(isRecording==1){
				  //recording[recordingLength]=1;
				  //recordingLength++;
			  //}

		  }else if (525<a1 && a1<=875){
			  switch (volume_counter){
			  	  case 0:
			  		  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,e4_vol1,period_2,DAC_ALIGN_12B_R);
				  	  break;
			  	  case 1:
			  		  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,e4_vol2,period_2,DAC_ALIGN_12B_R);
			  		  break;
			  	  case 2:
			  		  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,e4_vol3,period_2,DAC_ALIGN_12B_R);
			  		  break;
			  }
			  sprintf(buffer,"[volume:%d]%s|||||||| \r\n",(volume_counter+1),E4_block);

			  //if(isRecording==1){
				  //recording[recordingLength]=2;
				  //recordingLength++;
			  //}

		  }else if(175<a1 && a1<=525){
			  switch (volume_counter){
				  case 0:
					  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,g4_vol1,period_3,DAC_ALIGN_12B_R);
				  break;
				  case 1:
					  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,g4_vol2,period_3,DAC_ALIGN_12B_R);
					  break;
				  case 2:
					  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,g4_vol3,period_3,DAC_ALIGN_12B_R);
					  break;
			  }
			  sprintf(buffer,"[volume:%d]%s|||||||| \r\n",(volume_counter+1),G4_block);

			  //if(isRecording==1){
				  //recording[recordingLength]=3;
				  //recordingLength++;
			  //}

		  }else if (-175<a1 && a1<=175){
			  switch (volume_counter){
			  	  case 0:
			  		  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,a4_vol1,period_4,DAC_ALIGN_12B_R);
				  	  break;
			  	  case 1:
			  		  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,a4_vol2,period_4,DAC_ALIGN_12B_R);
			  		  break;
			  	  case 2:
			  		  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,a4_vol3,period_4,DAC_ALIGN_12B_R);
			  		  break;
			  }
			  sprintf(buffer, "[volume:%d]%s|||||||| \r\n",(volume_counter+1),A4_block);

			  //if(isRecording==1){
				  //recording[recordingLength]=4;
				  //recordingLength++;
			  //}

		  }else if (-525<a1 && a1<=-175){
			  switch (volume_counter){
			  	  case 0:
			  		  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,c5_vol1,period_5,DAC_ALIGN_12B_R);
				  	  break;
			  	  case 1:
			  		  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,c5_vol2,period_5,DAC_ALIGN_12B_R);
			  		  break;
			  	  case 2:
			  		  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,c5_vol3,period_5,DAC_ALIGN_12B_R);
			  		  break;
			  }
			  sprintf(buffer, "[volume:%d]%s|||||||| \r\n",(volume_counter+1),C5_block);

			  //if(isRecording==1){
				  //recording[recordingLength]=5;
				  //recordingLength++;
			  //}

		  }else if(-875<a1 && a1<=-525){
			  switch (volume_counter){
			  	  case 0:
			  		  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,e5_vol1,period_6,DAC_ALIGN_12B_R);
				  	  break;
			  	  case 1:
			  		  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,e5_vol2,period_6,DAC_ALIGN_12B_R);
			  		  break;
			  	  case 2:
			  		  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,e5_vol3,period_6,DAC_ALIGN_12B_R);
			  		  break;
			  }
			  sprintf(buffer, "[volume:%d]%s|||||||| \r\n",(volume_counter+1),E5_block);

			  //if(isRecording==1){
				  //recording[recordingLength]=6;
				  //recordingLength++;
			  //}

		  }else if(a1<=-875){
			  switch (volume_counter){
			  	  case 0:
			  		  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,g5_vol1,period_7,DAC_ALIGN_12B_R);
				  	  break;
			  	  case 1:
			  		  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,g5_vol2,period_7,DAC_ALIGN_12B_R);
			  		  break;
			  	  case 2:
			  		  HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,g5_vol3,period_7,DAC_ALIGN_12B_R);
			  		  break;
			  }
			  sprintf(buffer, "[volume:%d]%s|||||||| \r\n",(volume_counter+1),G5_block);

			  //if(isRecording==1){
				  //recording[recordingLength]=7;
				  //recordingLength++;
			  //}

		  }

		  HAL_UART_Transmit(&huart1, (uint8_t *)&buffer, sizeof(buffer), HAL_MAX_DELAY);
		  HAL_Delay(500);
		  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
		  HAL_Delay(0);
	  }else{
		  if(a2>800){
			  volume_counter=(volume_counter+1)%3;
		  }
		  sprintf(buffer, "[volume:%d] \r\n", (volume_counter+1));
		  HAL_UART_Transmit(&huart1, (uint8_t *)&buffer, sizeof(buffer), HAL_MAX_DELAY);
		  HAL_Delay(2000);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 800;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(myLED_GPIO_Port, myLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : myButton_Pin */
  GPIO_InitStruct.Pin = myButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(myButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : myLED_Pin */
  GPIO_InitStruct.Pin = myLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(myLED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
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
