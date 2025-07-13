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
#include <math.h>
#include <stdio.h>
#include <string.h>

#define slope 0.1
#define offset 2.511

#define average_runs 50 // multiply this value with delay(x) for reading interval. currently (5) * 50 = 250 ms

#define length 9 //for length 9

/*
 *
 * TO DO LIST:
 * Make a deque like thinng with legnth "length" and median  ad the actual current.
 * This current median will be sent whenever encoder message is given. A dataCheck flag will be sent.
 */

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
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void resetEncoder(void); //function to reset encoder values when polling stops

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//store ID of master CAN node
#define MASTER_ID 0x465

//store ID of first slave CAN node
#define SLAVE0_ID 0x550

//store the ID of the current slave node, change this when uploading
#define SLAVE_ID 0x553 //0x551 //0x552 //0x553

//how many pulses are seen on 1 full rotation
#define MAX_COUNT 3800

#define CURRENT_LENGTH 5 //the length of the array housing latest 'n' current readings --> keep odd SHOULD BE LESS THAN 20 OR EDIT THE MEDIAN FUNCTION


CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader;
uint8_t RxData[8];
uint8_t TxData[8];
uint32_t TxMailbox;

uint8_t sysCheck=0; //000 -> 0
uint8_t polling=7; //111->7
uint8_t stopPoll=5; //101->5
uint8_t cutOffMotor=3; //011 ->3 --> send this value mode to forcefully cutoff the motor.

//define variables for storing encoder values
uint32_t currentEnc=0;
uint32_t previousEnc=0; //to calculate velocity
int16_t diff=0;
int32_t pos=0; //absolute position with accuracy of 6'
int16_t angPos=0; //angular position
uint8_t PWM=0;
uint8_t DIR=0; //PWM and DIR pins for motor direction and speed

//current sensor:

int CURRENT_MAX = 15;

uint64_t adc_sum = 0;
float current_sum = 0.0;
int counter = 0;

float latest_current = 0; //the latest current reading goes here

volatile uint16_t ADC_VAL;

float rounding(float x) {
   float frac = (x*100 - (int)(x*100))/100;
   return x - frac; //xx.xx value
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
		ADC_VAL = HAL_ADC_GetValue(&hadc1);
		//HAL_ADC_Start_IT(&hadc1);  // Restart conversion --> ADC aldready in continious conversion mode hence not needed.
}

//median function: max 20 buffer here:
float median(const float *a, int n) {
    float temp[20];  // Safe since n < 20
    for (int i = 0; i < n; i++)
        temp[i] = a[i];

    // Partial selection sort up to median
    for (int i = 0; i <= n / 2; i++) {
        int min = i;
        for (int j = i + 1; j < n; j++)
            if (temp[j] < temp[min])
                min = j;
        float t = temp[i]; temp[i] = temp[min]; temp[min] = t;
    }

    return temp[n / 2];
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
  MX_CAN_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

  //start PWM mode for the motor
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  TIM2->CCR4=0; //set initial PWM value as zero
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin,DIR); //set the initial value of DIR
  //start TIM4 in encoder mode to read the encoder pulses in 4x mode
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  //Current sensor interrupt start:
  HAL_ADC_Start_IT(&hadc1); //start with interrupts


  TxHeader.IDE=CAN_ID_STD;
  TxHeader.RTR=CAN_RTR_DATA;
  TxHeader.StdId=SLAVE_ID; // ID of this CAN node

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  float Voltage_Div = 1.5;

  float current_q[CURRENT_LENGTH] = {0};
  int q_counter = 0;

  while (1)
  {
	  HAL_Delay(5);

	  __disable_irq(); //disable interrupt


	  float voltage = ((ADC_VAL + 69.631)/(1249.362)) * Voltage_Div - offset;
	  float current = ((voltage)/slope);


	if(current > CURRENT_MAX){

		TxHeader.DLC=1;
		TxData[0] = 250; //Return value 250 if motor shutoff

		HAL_GPIO_WritePin(CutoffPower_GPIO_Port, CutoffPower_Pin, 0); //toggle the Nmos to off
		TIM2->CCR4 = 0;
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 0); //shutout directions and PWM


		//DELAY?? (REQUIRED??)
		HAL_Delay(3000); //3 second delay --> to reposition rover?

		if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
		{
			Error_Handler ();
		}
	}


	  current_sum += current;
	  adc_sum += ADC_VAL;
	  counter++;


	  if(counter == average_runs){
		  float avg_current = current_sum/average_runs;

		  avg_current = rounding(avg_current); //to closest x.x value


		  if(avg_current < 0){
			  avg_current = 0; // in no current state sometimes current reading -0.01 Ampere type stuff -> CAN cant handle since uint8_t is unisgned
		  }

		  //float avg_adc = ((float)adc_sum)/average_runs;

		  counter = 0;
		  current_sum = 0;
		  adc_sum = 0;

		  current_q[q_counter] = avg_current; //contains latest 5 readings
		  q_counter++;
		  q_counter = q_counter%CURRENT_LENGTH;

		  //calculate median:

		  latest_current = median(current_q, CURRENT_LENGTH);

	  }

	  __enable_irq(); //enable interupts again


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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 10;  // from 0 to 13, which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO1; //accept messages at FIFO1
  canfilterconfig.FilterIdHigh = MASTER_ID<<5; //accept from only master ID
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0b11111111111<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 0;  // doesn't matter for single CAN MCUs

  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  /* USER CODE END CAN_Init 2 */

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
  htim2.Init.Prescaler = 18-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3800;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CutoffPower_GPIO_Port, CutoffPower_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_Pin */
  GPIO_InitStruct.Pin = DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CutoffPower_Pin */
  GPIO_InitStruct.Pin = CutoffPower_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CutoffPower_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	//receive message stored in the CAN FIFO1 buffer
	if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO1, &RxHeader, RxData) != HAL_OK)
	{
		Error_Handler();
	}

	//check if the message is coming from master node and whether it is meant for the current slave node
	//break down the command into qualifier and address
	uint8_t command=RxData[0]>>5; //look at the first 3 bits -> 8 modes max, see the mode

	uint8_t address=(command<<5)^RxData[0]; //address is contained in last 5 bits of the data byte

	if (RxHeader.DLC==3 && address==SLAVE_ID-SLAVE0_ID) //polling mode will send 3 bytes of data now
	{

		HAL_GPIO_WritePin(CutoffPower_GPIO_Port, CutoffPower_Pin, 1); //toggle the Nmos to on in case it is off


		if (command==polling)
		{
			//set PWM and direction pins
			PWM=RxData[1];
			DIR=RxData[2];
			TIM2->CCR4=PWM; //set the duty cycle as an 8 bit value that is received

			//set the direction pin
			HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin,DIR); //set the initial value of DIR

			//extract information about encoder and send to master node
			TxHeader.DLC=8; //send 8 bytes of data for encoder input

			currentEnc=TIM4->CNT; //get the current count of encoder
			if (currentEnc==previousEnc)
				diff=0; //no change in velocity
			else if (currentEnc>previousEnc)
			{
				if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4)) //underflow occurred
					diff=-previousEnc-__HAL_TIM_GET_AUTORELOAD(&htim4)+currentEnc;
				else
					diff=currentEnc-previousEnc;
			}
			else
			{
				if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4))
					diff=currentEnc-previousEnc;
				else //overflow occurred
					diff=currentEnc+__HAL_TIM_GET_AUTORELOAD(&htim4)-previousEnc;
			}
			pos+=diff;
			previousEnc=currentEnc;

			//performing the calculations for data transfer
			//first 4 bytes -> absolute position value
			//next 2 bytes -> angular position
			//next 2 bytes -> difference in encoder readings

			//extracting bytes for absolute encoder position
			uint8_t b0 = (pos>>24)&0xFF;
			uint8_t b1 = (pos>>16)&0xFF;
			uint8_t b2 = (pos>>8)&0xFF;
			uint8_t b3 = (pos)&0xFF;

			//extracting bytes for angular position
			angPos = round((3600.0*currentEnc)/MAX_COUNT);
			uint8_t b4 = (angPos>>8)&0xFF;
			uint8_t b5 = (angPos)&0xFF;

			//extracting bytes for difference in encoder values
			uint8_t b6 = (diff>>8)&0XFF;
			uint8_t b7 = (diff)&0xFF;

			//preparing data for transmission
			TxData[0]=b0;
			TxData[1]=b1;
			TxData[2]=b2;
			TxData[3]=b3;
			TxData[4]=b4;
			TxData[5]=b5;
			TxData[6]=b6;
			TxData[7]=b7;

			//send the CAN message
			if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
			{
				Error_Handler ();
			}

			//send current data also:

			TxHeader.DLC=2; //send 1 byte of data as current data

			//encode current into both bytes --> byte 1 for integer part, byte 2 for decimal part.
			TxData[0] = (uint8_t)(latest_current);  // integer part
			TxData[1] = (uint8_t)((latest_current - (int)(latest_current)) * 100.0);  // two decimal digits

			if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
			{
				Error_Handler ();
			}

			//toggle GPIO pin to indicate continuous message transmission
			//use this option for debugging
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		}


		else if(command == cutOffMotor){ //to force test the motors stopping

			TxHeader.DLC=1;
			TxData[0] = 255; //confirmation that turned off --> return max value 255 for testing return.

			HAL_GPIO_WritePin(CutoffPower_GPIO_Port, CutoffPower_Pin, 0); //toggle the Nmos to off

			if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
			{
				Error_Handler ();
			}

		}
	}

	if (RxHeader.DLC==1 && address==SLAVE_ID-SLAVE0_ID)
	{
		//perform the systems check of CAN nodes
		if (command==sysCheck)
		{
			TxHeader.DLC=1; //send 1 byte of data as a systems check
			TxData[0]=RxData[0];
			//transmit the received message back to master CAN node
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
			{
				Error_Handler ();
			}

		}


	}


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
