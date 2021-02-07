/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "lcd.h"
#include "24l01.h"

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

/* USER CODE BEGIN PV */
// 1 enable connection packet sending, 0 disable
uint8_t is_connected = 0;

uint8_t rxBuffer[1000];
uint8_t rxData[1024];
uint32_t rxLength;
uint8_t is_receiver = 0;
uint8_t display_buffer[TOTAL_LINE][CHAR_PER_LINE];
const uint8_t EMPTY_LINE[CHAR_PER_LINE] = {0};
uint8_t line_type[TOTAL_LINE];
uint16_t line_length = 0;

Message to_send;
Message received;

const int aleft = 21;
const int aright = 219;
const int atop = 21;
const int abottom = 299;
const int alineHeight = 20;
const int awidth = 190;
const int afontSize = 16;

uint8_t amemoryData[14][24] = { 0 };
int amemoryPosition[14];

int alineCount = 0;
int aend=0;
/* TODO:
 * 1. connection setup
 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
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
	MX_USART1_UART_Init();
	MX_SPI1_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	LCD_Init();
	NRF_Init();
	//-------------------------------------------------------------------------------------------------
	LCD_Clear(CYAN);
	POINT_COLOR = BLACK;
	LCD_DrawRectangle(20, 20, 220, 300);
	LCD_Fill(aleft, atop, aright, abottom, WHITE);
	//-------------------------------------------------------------------------------------------------
	NRF24L01_Init();
	HAL_TIM_Base_Start_IT(&htim3);

	uint8_t packet_buffer[100];
	uint8_t line_buffer[500];
	uint8_t line_buffer_ptr = 0;
	char error_message[] = "NRF not initialized";
	while (NRF24L01_Check()) {
		HAL_UART_Transmit_IT(&huart1, (uint8_t *) error_message, strlen(error_message));
		HAL_Delay(1000);
	}
	HAL_UART_Transmit_IT(&huart1, (uint8_t *) error_message, strlen(error_message));


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		NRF24L01_RX_Mode();
		HAL_Delay(100);
		if (is_connected) {

			if (NRF24L01_RxPacket(packet_buffer) == 0) {
				switch (((Message *) &packet_buffer)->message_type) {
					case MESSAGE_PAYLOAD: {
						line_type[line_length] = LINE_RECEIVED_MESSAGE;
						strcpy((char *) display_buffer[line_length], (char *) packet_buffer + 1);
						line_length++;

						HAL_UART_Transmit_IT(&huart1, (uint8_t *) packet_buffer + 1, strlen((char *) packet_buffer + 1));
						break;
					}
					case MESSAGE_FIN: {
						char log[] = "Connection Closed\n";
						HAL_UART_Transmit_IT(&huart1, (uint8_t *) log, strlen(log));

						disconnect();
						break;
					}
					case (MESSAGE_FRAG | MESSAGE_FRAG_START | MESSAGE_FRAG_MORE | MESSAGE_PAYLOAD): {
						strcpy((char *) line_buffer, (char *) packet_buffer + 1);
						line_buffer_ptr = strlen((char *) packet_buffer + 1);
						break;
					}
					case (MESSAGE_FRAG | MESSAGE_FRAG_MORE | MESSAGE_PAYLOAD): {
						strcpy((char *) line_buffer + line_buffer_ptr, (char *) packet_buffer + 1);
						line_buffer_ptr += strlen((char *) packet_buffer + 1);
						break;
					}
					case (MESSAGE_FRAG | MESSAGE_PAYLOAD): {
						strcpy((char *) line_buffer + line_buffer_ptr, (char *) packet_buffer + 1);
						line_buffer_ptr += strlen((char *) packet_buffer + 1);

						make_line((char *) line_buffer, line_buffer_ptr, LINE_RECEIVED_MESSAGE);
						// insert into display buffer
						break;
					}
					case MESSAGE_PEER: {
						line_type[line_length] = LINE_RECEIVED_MESSAGE;
						strcpy((char *) display_buffer[line_length], (char *) packet_buffer);
						line_length++;

						HAL_UART_Transmit_IT(&huart1, (uint8_t *) packet_buffer, strlen((char *) packet_buffer));
						break;
					}
					case MESSAGE_KEEP_ALIVE:
					default: {
						break;
					}

				}
			}
		}
		display();

		/* USER CODE END 3 */
	}
}

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

/* USER CODE BEGIN 4 */
void disconnect() {
	is_connected = 0;
	Message message;
	message.message_type = MESSAGE_FIN;
	NRF24L01_TX_Mode();
	NRF24L01_TxPacket((uint8_t*)&message);

	for (uint8_t i = 0; i < 5; i++)
	{
		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		HAL_Delay(300);
	}
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
}

void connect() {
	is_connected = 1;
	for (uint8_t i = 0; i < 5; i++)
	{
		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		HAL_Delay(300);
	}
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
}
//------display-----------------------------------------------------------------------------------
void showString(char msg[], int position) {
	int startPoint;
	if (position == aright) {
		POINT_COLOR = BLUE;
		int len=0;
		for (int i=0;i<strlen(msg);i++) {
			if (msg[i]!='\r')
				len++;
			else
				break;
		}
		startPoint=aright - len * (afontSize / 2);

	} else if (position == aleft) {
		POINT_COLOR = RED;
		startPoint=aleft;
	}
	LCD_ShowString(startPoint, atop + alineCount * alineHeight, awidth, afontSize, afontSize, (uint8_t*) msg);
	if (alineCount<13)
		alineCount++;
}
int last_line_length=0;
int flag=1;
void display()
{
	alineCount=0;
	int start=0;
	if (line_length>=15) {
		start=line_length-14;
		if (line_length==15 && flag) {
			LCD_Fill(aleft, atop, aright, abottom, WHITE);
			flag=0;
		}
		if (line_length>last_line_length)
			LCD_Fill(aleft, atop, aright, abottom, WHITE);
	}
	last_line_length=line_length;

	for (int i = start; i < line_length; i++)
	{
		switch (line_type[i]) {
			case LINE_RECEIVED_MESSAGE:
			{
				showString((char*)display_buffer[i], aleft);
				break;
			}
			case LINE_SENT_MESSAGE:
			{
				showString((char*)display_buffer[i], aright);
				break;
			}
			case LINE_CONNECTION:
			{
				// TODO:
				// show connection loss message
				showString((char*)display_buffer[i], aleft);
				break;
			}
			default:
			{
				break;
			}
		}
	}

}

void make_line(char *line_buffer, uint16_t len, uint8_t ltype)
{
	line_buffer[len] = '\0';
	char* ptr = line_buffer;
	while (*ptr != '\0')
	{
		line_type[line_length] = ltype;
		// clear the line
		strlcpy((char*)display_buffer[line_length], (char*)EMPTY_LINE, CHAR_PER_LINE);
		strlcpy((char*)display_buffer[line_length], ptr, CHAR_PER_LINE - 1);
		ptr += strlen((char*)display_buffer[line_length]);
		line_length++;
	}
}
void send_packet(char *payload, uint8_t message_type)
{
	NRF24L01_TX_Mode();

	Message message;

	if (strlen(payload) < 31)
	{
		message.message_type = MESSAGE_PAYLOAD;
		strcpy((char*)message.payload, payload);
	}
	else
	{
		message.message_type = MESSAGE_FRAG_START | MESSAGE_FRAG | MESSAGE_FRAG_MORE | MESSAGE_PAYLOAD;
		strlcpy((char*)message.payload, payload, 31);
		NRF24L01_TxPacket((uint8_t*)&message);
		char* ii;
		HAL_Delay(500);
		for (ii = payload + 31; strlen(ii) > 31; ii += 31)
		{
			message.message_type = MESSAGE_FRAG | MESSAGE_FRAG_MORE | MESSAGE_PAYLOAD;
			strlcpy((char*)message.payload, ii, 31);
			NRF24L01_TxPacket((uint8_t*)&message);
			HAL_Delay(500);
		}
		message.message_type = MESSAGE_FRAG | MESSAGE_PAYLOAD;
		strcpy((char*)message.payload, ii);
	}
	NRF24L01_TxPacket((uint8_t*)&message);
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
