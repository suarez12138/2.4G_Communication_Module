#include "main.h"
#include "lcd.h"
#include "string.h"
#include <stdio.h>

ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);

//-------------------------------------------------------------------------------------------------

const int left = 21;
const int right = 219;
const int top = 21;
const int bottom = 299;
const int lineHeight = 20;
const int width = 190;
const int fontSize = 16;
uint8_t rxBuffer[20];
uint16_t raw;


uint8_t memoryData[14][24] = { 0 };
int memoryPosition[14];

int lineCount = 0;
int end=0;
void showString(char msg[], int position) {
	int startPoint;
	if (position == right) {
		POINT_COLOR = BLUE;
		startPoint=right - (strlen(msg)) * (fontSize / 2);
		LCD_ShowString(startPoint,
				top + lineCount * lineHeight, width, fontSize, fontSize,
				(uint8_t*) msg);
	} else if (position == left) {
		POINT_COLOR = RED;
		startPoint=left;
		LCD_ShowString(left, top + lineCount * lineHeight, width, fontSize,
				fontSize, (uint8_t*) msg);
	} else {
		return;
	}
	if (lineCount == 13 && end) {
		rolling(msg,startPoint);
	} else {
		for (int k = 0; k < strlen(msg); k++)
			memoryData[lineCount][k] = msg[k];
		memoryPosition[lineCount]=startPoint;
		if (lineCount < 13)
			lineCount++;
		else end=1;
	}
}

void showInput(uint8_t *msg, int length, int color) {
	POINT_COLOR = color;
	int startPoint=right - (length - 1) * (fontSize / 2);
	LCD_ShowString(startPoint,
			top + lineCount * lineHeight, width, fontSize, fontSize, msg);
	if (lineCount == 13 && end) {
		rolling(msg,startPoint);
	} else {
		for (int k = 0; k < length; k++)
			memoryData[lineCount][k] = msg[k];
		memoryPosition[lineCount]=startPoint;
		if (lineCount < 13)
				lineCount++;
			else end=1;
	}
}

void showTemperature() {
	char temperature[20];
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	raw = HAL_ADC_GetValue(&hadc1);
	double v=raw*(3.3/4096);
	double t=((1.43-v)/4.3)+25;
	sprintf(temperature, "%f\r\n", t);
	HAL_UART_Transmit(&huart1, (uint8_t*)temperature, strlen(temperature), HAL_MAX_DELAY);
	showString("Answer", left);
	showString(temperature, left);
	showString("Question", right);
	return;
}

void rolling(char msg[],int startPoint) {
	for (int i = 0; i < 13; i++) {
		for (int j = 0; j < 24; j++) {
			memoryData[i][j] = memoryData[i + 1][j];
		}
		memoryPosition[i]=memoryPosition[i+1];
	}
	for (int j = 0; j < 24; j++) {
		memoryData[13][j] = msg[j];
	}
	memoryPosition[13]=startPoint;
	LCD_Fill(left, top, right, bottom, WHITE);
	for (int i = 0; i < 14; i++) {
		if(memoryPosition[i]==left)
			POINT_COLOR = RED;
		else
			POINT_COLOR = BLUE;
		LCD_ShowString(memoryPosition[i],
				top + i * lineHeight, width, fontSize, fontSize, memoryData[i]);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		static unsigned char uRx_Data[1024] = { 0 };
		static unsigned char uLength = 0;
		if (rxBuffer[0] == '\n') {
			HAL_UART_Transmit(&huart1, uRx_Data, uLength, 0xffff);
			if (uLength < 25) {
				showInput((uint8_t*) uRx_Data, uLength, BLUE);
				char t[12]="temperature";
				int judge=1;
				for(int i=0;i<11;i++){
					if(t[i]!=uRx_Data[i]){
						judge=0;
						break;
					}
				}
				if (judge)
					showTemperature();
			}
			else {
				for (int i = 0; i < uLength; i += 24) {
					char result[24];
					for (int j = 0; j < 24; j++) {
						result[j] = uRx_Data[i + j];
					}
					if (uLength - i > 24)
						showInput((uint8_t*) result, 25, BLUE);
					else
						showInput((uint8_t*) result, uLength % 24, BLUE);
				}
			}
			uLength = 0;
		} else {
			uRx_Data[uLength] = rxBuffer[0];
			uLength++;
		}
	}
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();

  LCD_Init();
  HAL_UART_Receive_IT(&huart1, (uint8_t*) rxBuffer, 1);


//-------------------------------------------------------------------------------------------------
  	LCD_Clear(CYAN);
  	POINT_COLOR = BLACK;
  	LCD_DrawRectangle(20, 20, 220, 300);
  	LCD_Fill(left, top, right, bottom, WHITE);
  	showString("Question", right);
//-------------------------------------------------------------------------------------------------
  	while(1) {}
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
