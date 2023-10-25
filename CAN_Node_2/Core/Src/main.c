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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PDU_ADD		0x0A
#define DISCHARGE	0x05
#define PRECHARGE	0x03
#define OFF			0x00
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
GPIO_TypeDef* GPIO_Port_List[4] = {SA_GPIO_Port, SB_GPIO_Port, SC_GPIO_Port, SD_GPIO_Port};
uint16_t GPIO_Pin_List[4] = {SA_Pin, SB_Pin, SC_Pin, SD_Pin};

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

int flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

void Precharge(){	//	SA = 1, SB = 1, SC = 0 , SD = 0
	//	SA = 1
	HAL_GPIO_WritePin(GPIO_Port_List[0], GPIO_Pin_List[0], GPIO_PIN_RESET);
	HAL_Delay(500);
	//	SB = 1
	HAL_GPIO_WritePin(GPIO_Port_List[1], GPIO_Pin_List[1], GPIO_PIN_RESET);
	//	Delay do zastapienia mierzeniem napiecia na falowniku
	HAL_Delay(10000);
	//	SC = 0
	HAL_GPIO_WritePin(GPIO_Port_List[2], GPIO_Pin_List[2], GPIO_PIN_SET);
	//	SD = 0
	HAL_GPIO_WritePin(GPIO_Port_List[3], GPIO_Pin_List[3], GPIO_PIN_SET);
}

void Discharge(){	//	SA = 1, SB = 0, SC = 1 , SD = 0
	//	SA = 1
	HAL_GPIO_WritePin(GPIO_Port_List[0], GPIO_Pin_List[0], GPIO_PIN_RESET);
	HAL_Delay(500);
	//	SB = 0
	HAL_GPIO_WritePin(GPIO_Port_List[1], GPIO_Pin_List[1], GPIO_PIN_SET);
	HAL_Delay(500);
	//	SC = 1
	HAL_GPIO_WritePin(GPIO_Port_List[2], GPIO_Pin_List[2], GPIO_PIN_RESET);
	HAL_Delay(500);
	//	SD = 0
	HAL_GPIO_WritePin(GPIO_Port_List[3], GPIO_Pin_List[3], GPIO_PIN_SET);
	HAL_Delay(500);
}

void Off(){	//	SA = 0, SB = 0, SC = 0, SD = 0
	//	SC = 0
	HAL_GPIO_WritePin(GPIO_Port_List[2], GPIO_Pin_List[2], GPIO_PIN_SET);
	HAL_Delay(500);
	//	SA = 0
	HAL_GPIO_WritePin(GPIO_Port_List[0], GPIO_Pin_List[0], GPIO_PIN_SET);
	//	SB = 0
	HAL_GPIO_WritePin(GPIO_Port_List[1], GPIO_Pin_List[1], GPIO_PIN_SET);
	//	SD = 0
	HAL_GPIO_WritePin(GPIO_Port_List[3], GPIO_Pin_List[3], GPIO_PIN_SET);
}

//	INTERRUPTS
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == B1_Pin){

	}
}


void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){		//Funkcja odbierająca dane z CAN
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);
	flag = 1;
}

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
  /* USER CODE BEGIN 2 */

  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);		//Callback na przychodzącą wiadomość po CAN

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (flag == 1){
		  flag = 0;
		  switch(RxHeader.StdId){
		  case PDU_ADD:
			  switch(RxData[0]){
			  case DISCHARGE:
				  Discharge();
				  break;
			  case PRECHARGE:
				  Precharge();
				  break;
			  case OFF:
				  Off();
				  break;
			  default:
				  break;
			  }
			  break;

          default:
			  break;
		  }
	  }



	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
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

  HAL_CAN_Start(&hcan);


  //	CAN FILTER
  CAN_FilterTypeDef filterConf;
  filterConf.FilterActivation = CAN_FILTER_ENABLE;
  filterConf.FilterBank = 0;
  filterConf.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  filterConf.FilterIdHigh = 0x1<<5;
  filterConf.FilterIdLow = 0;
  filterConf.FilterMaskIdHigh = 0x1<<5;
  filterConf.FilterMaskIdLow = 0;
  filterConf.FilterMode = CAN_FILTERMODE_IDMASK;
  filterConf.FilterScale = CAN_FILTERSCALE_32BIT;
  HAL_CAN_ConfigFilter(&hcan, &filterConf);

  /* USER CODE END CAN_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SD_Pin|SC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SB_Pin|SA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_Pin SC_Pin */
  GPIO_InitStruct.Pin = SD_Pin|SC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SB_Pin SA_Pin */
  GPIO_InitStruct.Pin = SB_Pin|SA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
