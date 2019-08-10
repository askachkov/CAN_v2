
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "usb_device.h"
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "CANSPI.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define LOG_RECORDS_SIZE 8
#define bool uint8_t
#define true 1
#define false 0

typedef enum
{
	Status_Error_CANSPI_Was_Not_initilized = 1,
	Status_Error_CANSPI_Transmit = 2,
	Status_Error_CANSPI_Transmit_Passive = 4,
	Status_Error_CANSPI_Recieve_Passive = 8,
	Status_Hanshake_OK = 16,
	Status_With_CAN_Body = 32,
	Status_TestPin = 64,
	Status_Error_HAL = 128,
} Status;

typedef enum
{
	Command_Invalid = -1,
	Command_Turn_CAN120_On = 0,
	Command_Turn_CAN120_Off,
	Command_Tx_CAN,
	Command_Handshake_Comes,
	Command_Reset_CAN,
	Command_MAX
} Command;

Command CURRENT_STATE = Command_Invalid;

#ifdef MCP2515_ENABLED

#pragma pack(push, 1)
typedef struct
{
	uint8_t headerMSG;
	uint32_t status;
	uCAN_MSG msg;
} Message;
#pragma pack(pop)

typedef struct
{
	Message rx;
	uCAN_MSG tx;
	uint32_t lastStatus;
} Context;

#endif//MCP2515_ENABLED
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */
/* USER CODE BEGIN 0 */
void USBD_Reciever_USER(uint8_t * Buf, uint32_t *Len)
{
	//if ( *Len != 1 || Buf[0] >= Command_MAX  ) {
	//	return;
	//}
	CURRENT_STATE = (Command)Buf[0];
}

void updateState(Context * pContext, Status status, bool flag)
{
	if ( !flag ){
		pContext->rx.status &= ~status;
	}
	if ( flag ){
		pContext->rx.status |= status;
	}
}

bool getState(Context * pContext, Status status)
{
	return (pContext->rx.status & status) != 0 ? true : false;
}

void processIncomingCommand(Context * pContext)
{
	uCAN_MSG txMessage;
	uint8_t res = 0;
	if ( CURRENT_STATE != Command_Invalid ){
		switch ( CURRENT_STATE ){
			case Command_Turn_CAN120_On:
				HAL_GPIO_WritePin(PIN_CAN120_GPIO_Port, PIN_CAN120_Pin, GPIO_PIN_SET);
				break;
			case Command_Turn_CAN120_Off:
				HAL_GPIO_WritePin(PIN_CAN120_GPIO_Port, PIN_CAN120_Pin, GPIO_PIN_RESET);
				break;
			case Command_Tx_CAN:
				txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
				txMessage.frame.id = 0xAA;
				txMessage.frame.dlc = 2;
				txMessage.frame.data0 = 0xCC;
				txMessage.frame.data1 = 0xBB;
				res = CANSPI_Transmit(&txMessage);
				updateState(pContext, Status_Error_CANSPI_Transmit, res == 0);
				updateState(pContext, Status_TestPin, !getState(pContext, Status_TestPin));
				break;
			case Command_Handshake_Comes:
				updateState(pContext, Status_Hanshake_OK, true);
				break;
			case Command_Reset_CAN:
				MCP2515_Reset();
				HAL_Delay(1);
				res = CANSPI_Initialize();
				updateState(pContext, Status_Error_CANSPI_Was_Not_initilized, res != 0);
				break;
			default:
				break;
		}
		CURRENT_STATE = Command_Invalid; // Single invocation
	}
}

void processIncomingMessages(Context * pContext)
{
	uint8_t res = 0;
	res = CANSPI_isTxErrorPassive();
	updateState(pContext, Status_Error_CANSPI_Transmit_Passive, res != 0);
	res = CANSPI_isRxErrorPassive();
	updateState(pContext, Status_Error_CANSPI_Recieve_Passive, res != 0);
	if(CANSPI_Receive(&pContext->rx.msg))
	{
		updateState(pContext, Status_With_CAN_Body, true);
	}
}

void sendStateAndMessage(Context * pContext)
{
	if ( pContext->lastStatus != pContext->rx.status ) {
		if ( getState(pContext, Status_With_CAN_Body) == false ){
			//Hasn't a CAN body
			CDC_Transmit_FS((uint8_t*)&pContext->rx, 11);
		}
		if ( getState(pContext, Status_With_CAN_Body) == true ){
			//Has a CAN body
			CDC_Transmit_FS((uint8_t*)&pContext->rx, 11+pContext->rx.msg.frame.dlc);
			//cleanup
			updateState(pContext, Status_With_CAN_Body, false);
			memset(&pContext->rx.msg, 0, sizeof(pContext->rx.msg));
		}
		pContext->lastStatus = pContext->rx.status;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t ret = 0;
	Context context;
	memset(&context, 0, sizeof(context));
	context.rx.headerMSG = 0xFF;
  /* USER CODE END 1 */
  /* MCU Configuration----------------------------------------------------------*/
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
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
#ifdef MCP2515_ENABLED
	ret = CANSPI_Initialize();
	updateState(&context, Status_Error_CANSPI_Was_Not_initilized, ret != 0);
#endif//MCP2515_ENABLED
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
		processIncomingCommand(&context);
		processIncomingMessages(&context);
		sendStateAndMessage(&context);
		//const char * MSG = "Hello\r\n";
		//CDC_Transmit_FS((uint8_t*)MSG, strlen(MSG));
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PIN_CAN120_Pin|CAN_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PIN_CAN120_Pin */
  GPIO_InitStruct.Pin = PIN_CAN120_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PIN_CAN120_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_CS_Pin */
  GPIO_InitStruct.Pin = CAN_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
