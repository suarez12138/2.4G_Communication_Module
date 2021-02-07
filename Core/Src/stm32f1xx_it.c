/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "spi.h"
#include "24l01.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern uint8_t is_receiver;
extern uint8_t rxBuffer[1024];
const uint8_t EMPTY_BUFFER[1024] = {0};
extern uint8_t rxData[1024];
extern uint32_t rxLength;
extern Message to_send;
extern uint8_t TX_ADDRESS[6];
extern uint8_t RX_ADDRESS[6];
extern uint8_t display_buffer[TOTAL_LINE][CHAR_PER_LINE];
extern uint8_t line_type[TOTAL_LINE];
extern uint16_t line_length;

extern uint8_t is_connected;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
void connect();

void disconnect();

/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  HAL_UART_Receive_IT(&huart1, (uint8_t*)rxBuffer, 1);

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	switch (GPIO_Pin)
	{
		case KEY0_Pin:
			if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET)
			{
				connect();
			}
			break;
		case KEY1_Pin:
			if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
			{
				disconnect();

			}
			break;
		default:
			break;
	}
	HAL_Delay(100);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		if (rxBuffer[0] == '\n')
		{
			// TODO: execution of command
			char* token = strtok((char *)rxData, " ");
			if (strcmp(token, "send") == 0)
			{
				char* to_send_data = (char *)(rxData + strlen(token) + 1);
				HAL_UART_Transmit(&huart1, (uint8_t *)to_send_data, strlen(to_send_data), HAL_MAX_DELAY);
				send_packet(to_send_data, MESSAGE_PAYLOAD);

				make_line((char*)to_send_data, strlen((char*)to_send_data), LINE_SENT_MESSAGE);
			}
			else if (strcmp(token, "send_addr") == 0)
			{
				// set address of current host
				char* addr = (char *)(rxData + strlen(token) + 1);
				HAL_UART_Transmit(&huart1, (uint8_t *)addr, strlen(addr), HAL_MAX_DELAY);
				addr[5] = '\0';
				strcpy((char*)TX_ADDRESS, (char*)addr);
			}
			else if (strcmp(token, "recv_addr") == 0)
			{
				// set address of host to communicate
				char* addr = (char *)(rxData + strlen(token) + 1);
				HAL_UART_Transmit(&huart1, (uint8_t *)addr, strlen(addr), HAL_MAX_DELAY);
				addr[5] = '\0';
				strcpy((char*)RX_ADDRESS, (char*)addr);
			}
			else if (strcmp(token, "send_to_peer") == 0)
			{
				char* to_send_data = (char *)(rxData + strlen(token) + 1);
				HAL_UART_Transmit(&huart1, (uint8_t *)to_send_data, strlen(to_send_data), HAL_MAX_DELAY);
				NRF24L01_TX_Mode();
				NRF24L01_TxPacket((uint8_t*)to_send_data);

				make_line((char*)to_send_data, strlen((char*)to_send_data), LINE_SENT_MESSAGE);
			}
			else if (strcmp(token, "connect") == 0)
			{
				connect();
			}
			else if (strcmp(token, "disconnect") == 0)
			{
				disconnect();
			}
			rxLength = 0;
			strlcpy((char*)rxData, (char*)EMPTY_BUFFER, 1024);

		}
		else if (rxBuffer[0] != '\r')
		{
			rxData[rxLength] = rxBuffer[0];
			rxLength++;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (is_connected)
	{
		Message message;
		message.message_type = MESSAGE_KEEP_ALIVE;
		message.payload[0] = '1';
		message.payload[1] = '\n';
		NRF24L01_TX_Mode();
		if (NRF24L01_TxPacket((uint8_t*)&message) != TX_OK)
		{
			char error_message[] = "[system] Connection Lost";
			make_line(error_message, strlen(error_message), LINE_CONNECTION);
			HAL_UART_Transmit(&huart1, (uint8_t*)error_message, strlen(error_message), HAL_MAX_DELAY);

			for (uint8_t i = 0; i < 4; i++)
			{
				HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
				HAL_Delay(300);
			}
		}
	}
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
