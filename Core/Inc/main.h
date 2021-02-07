/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void display();
void make_line(char *line_buffer, uint16_t len, uint8_t ltype);
void send_packet(char *payload, uint8_t message_type);
void connect();
void disconnect();
#define MESSAGE_SYN (uint8_t)0b00000001
#define MESSAGE_KEEP_ALIVE (uint8_t)0b00000010
#define MESSAGE_FIN (uint8_t)0b00000100
#define MESSAGE_PAYLOAD (uint8_t)0b00001000
#define MESSAGE_PEER (uint8_t)'c'

/*
 * Fragmentation
 * FRAG_START & FRAG & FRAG_MORE -> first packet
 * FRAG & FRAG_MORE -> middle packet
 * FRAG -> last packet
 */
#define MESSAGE_FRAG_START (uint8_t) 0b01000000
#define MESSAGE_FRAG (uint8_t)0b00010000
#define MESSAGE_FRAG_MORE (uint8_t)0b00100000

#define CHAR_PER_LINE 26
#define TOTAL_LINE 100

// indicators of line types
#define LINE_FRAG 0
#define LINE_RECEIVED_MESSAGE 1
#define LINE_SENT_MESSAGE 2
#define LINE_CONNECTION 3
#define CHAR_HEIGHT 12
#define CHAR_WIDTH 6

typedef struct message
{
		uint8_t message_type;
		uint8_t payload[31];
}Message;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY0_Pin GPIO_PIN_5
#define KEY0_GPIO_Port GPIOC
#define KEY0_EXTI_IRQn EXTI9_5_IRQn
#define LED0_Pin GPIO_PIN_8
#define LED0_GPIO_Port GPIOA
#define KEY1_Pin GPIO_PIN_15
#define KEY1_GPIO_Port GPIOA
#define KEY1_EXTI_IRQn EXTI15_10_IRQn
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */
#define NRF_IRQ_Pin GPIO_PIN_1
#define NRF_IRQ_GPIO_Port GPIOA
#define NRF_CE_Pin GPIO_PIN_4
#define NRF_CE_GPIO_Port GPIOA
#define NRF_CSN_Pin GPIO_PIN_4
#define NRF_CSN_GPIO GPIOC
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
