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

#define average_runs 50 // multiply this value with delay(x) for reading interval. currently 5 * 50 = 250 ms
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
#define MASTER_ID 0x365

//store ID of first slave CAN node
#define SLAVE0_ID 0x256

//store the ID of the current slave node, change this when uploading
#define SLAVE_ID 0x256 //0x256 //0x257 //0x258

//how many pulses are seen on 1 full rotation
#define MAX_COUNT 1430


CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader;
uint8_t RxData[8];
uint8_t TxData[8];
uint32_t TxMailbox;


//define prefix commands to set up CAN communication
uint8_t sysCheck=0; //000 -> 0
uint8_t polling=7; //111-> 7
uint8_t stopPoll=5; //101-> 5
uint8_t reset=1; //001 -> 1
uint8_t PID_hold = 3; //011 -> 3

//uint32_t encoderReset = 1000000000; //to prevent overflow, we set the initial position


//define variables for storing encoder values
uint32_t currentEnc=0;
uint32_t previousEnc=0; //to calculate velocity
int16_t diff=0;
int32_t pos=0; //absolute position with accuracy of 6'
//int16_t angPos=0; //angular position
uint8_t PWM=0;
uint8_t DIR=0; //PWM and DIR pins for motor direction and speed



// of the motors to a large value about which it can freely deviate

//mapping to arm -> shoulder,elbow,base,wrist1,wrist2,gripper
//arm degrees -> 90, 0, 50%, _, _, _
uint16_t resetPos[]={8660, 0, 59600, 5000, 5000, 5000}; //max count for each node to calculate angular position
//uint32_t home[]={0,0,0,0,0,0};

//powering up the brake releases it
//GPIO_PIN_RESET -> 1 -> pin HIGH
//GPIO_PIN_SET -> 0 -> pin LOW

//current sensor:

uint64_t adc_sum = 0;
float current_sum = 0.0;
int counter = 0;

volatile uint16_t ADC_VAL;

float rounding(float x) {
   float frac = (x*100 - (int)(x*100))/100;
   return x - frac; //xx.xx value
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{

		ADC_VAL = HAL_ADC_GetValue(&hadc1);


}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	//set the values to oscillate about this large value (no overflow)
//	for (uint8_t i=0;i<6;i++)
//	{
//		home[i] = resetPos[i] + encoderReset;
//	}


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

  //reset the encoders at the start
  resetEncoder();

  //Current sensor interrupt start:
  HAL_ADC_Start_IT(&hadc1); //start with interrupts

  TxHeader.IDE=CAN_ID_STD;
  TxHeader.RTR=CAN_RTR_DATA;
  TxHeader.StdId=SLAVE_ID; // ID of this CAN node

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  float Voltage_Div = 1.5;

  while (1)
  {

	  HAL_Delay(5);

	  __disable_irq(); //disable interrupt

	  float voltage = ((ADC_VAL + 69.631)/(1249.362)) * Voltage_Div - offset;
	  float current = ((voltage)/slope);

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

			TxHeader.DLC=2; //send 1 byte of data as current data

			//encode current into both bytes --> byte 1 for integer part, byte 2 for decimal part.
			TxData[0] = (uint8_t)(avg_current);  // integer part
			TxData[1] = (uint8_t)((avg_current - (int)(avg_current)) * 100.0);  // two decimal digits

			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
			{
				Error_Handler ();
			}

		  //transmit avg_current through CAN

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
  canfilterconfig.FilterMaskIdHigh = MASTER_ID<<5;
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
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
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
  HAL_GPIO_WritePin(Brake_GPIO_Port, Brake_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : Brake_Pin */
  GPIO_InitStruct.Pin = Brake_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Brake_GPIO_Port, &GPIO_InitStruct);

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
	uint8_t command=RxData[0]>>5; //look at the first 3 bits
	uint8_t address=(command<<5)^RxData[0];

	if (RxHeader.DLC==3 && address==SLAVE_ID-SLAVE0_ID) //polling mode will send 3 bytes of data now
	{
		if (command==polling || command == PID_hold)
		{
			//set PWM and direction pins
			PWM=RxData[1];
			DIR=RxData[2];

			TIM2->CCR4=PWM; //set the duty cycle as an 8 bit value that is received

			if (command == polling){
			if (PWM > 0)
				HAL_GPIO_WritePin(Brake_GPIO_Port, Brake_Pin, GPIO_PIN_SET); //set the pin high to open the brake
			else
				HAL_GPIO_WritePin(Brake_GPIO_Port, Brake_Pin, GPIO_PIN_RESET); //activate the brake if no command is given
			}
			else
			{
				HAL_GPIO_WritePin(Brake_GPIO_Port, Brake_Pin, GPIO_PIN_RESET); // activate the brake if no motion command is given
			}
			//set the direction pin
			HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin,DIR); //set the initial value of DIR

			//extract information about encoder and send to master node
			TxHeader.DLC=6; //send 6 bytes of data for encoder input

			currentEnc=TIM4->CNT; //get the current count of encoder and remove offset
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

			/*//extracting bytes for angular position
			angPos=round((3600.0*currentEnc)/(double)MAX_COUNT);
			uint8_t b4 = (angPos>>8)&0xFF;
			uint8_t b5 = (angPos)&0xFF;*/

			//extracting bytes for difference in encoder values
			uint8_t b4 = (diff>>8)&0XFF;
			uint8_t b5 = (diff)&0xFF;

			//preparing data for transmission
			TxData[0]=b0;
			TxData[1]=b1;
			TxData[2]=b2;
			TxData[3]=b3;
			TxData[4]=b4;
			TxData[5]=b5;

			//send the CAN message
			if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
			{
				Error_Handler ();
			}

			//toggle GPIO pin to indicate continuous message transmission
			//use this option for debugging
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		}
	}

	if (RxHeader.DLC==1 && address==SLAVE_ID-SLAVE0_ID)
	{
		HAL_GPIO_WritePin(Brake_GPIO_Port, Brake_Pin, GPIO_PIN_RESET); //activate the brake if no command is given

		//perform the systems check of CAN nodes
		if (command==sysCheck)
		{
			//HAL_GPIO_WritePin(Brake_GPIO_Port, Brake_Pin, GPIO_PIN_SET); //close the brake when not moving

			TxHeader.DLC=1; //send 1 byte of data as a systems check
			TxData[0]=RxData[0];
			//transmit the received message back to master CAN node
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
			{
				Error_Handler ();
			}
		}

		if (command==reset)
		{
			resetEncoder(); //reset the encoders when button is pressed
		}

		/*if (command==stopPoll)
		{
			//reset the encoder to initial state
			resetEncoder();

			//stop the PWM signals
			TIM1->CCR4=0; //set the duty cycle as an 8 bit value that is received

			//set the direction pin
			HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin,GPIO_PIN_RESET); //set the initial value of DIR
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

		}*/
	}
}

void resetEncoder()
{
	//reset all the values for encoder data sharing
	// EXECUTE THE COMMAND ONLY AFTER MOVING TO HOME POSITION
	currentEnc=resetPos[SLAVE_ID - SLAVE0_ID];
	previousEnc=resetPos[SLAVE_ID - SLAVE0_ID];
	diff=0;
	pos=resetPos[SLAVE_ID - SLAVE0_ID];
	//angPos=0;
	TIM4->CNT=0; //reset the counter
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
