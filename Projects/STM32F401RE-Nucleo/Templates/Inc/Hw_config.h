/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Platform config -----------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "Macro.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

#include "Custom_Button.h" 
#include "MEMS_Calculate.h"

/* Define config -------------------------------------------------------------*/

#define USARTx                           	USART2
#define USARTx_CLK_ENABLE()              	__HAL_RCC_USART2_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      	__HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      	__HAL_RCC_GPIOA_CLK_ENABLE() 

#define USARTx_FORCE_RESET()             	__HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           	__HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    	GPIO_PIN_2
#define USARTx_TX_GPIO_PORT              	GPIOA  
#define USARTx_TX_AF                     	GPIO_AF7_USART2
#define USARTx_RX_PIN                    	GPIO_PIN_3
#define USARTx_RX_GPIO_PORT              	GPIOA 
#define USARTx_RX_AF                     	GPIO_AF7_USART2


#define TIMx                         		TIM11
#define TIMx_CLK_ENABLE              		__TIM11_CLK_ENABLE

#define TIMx_RCC_FORCE_RESET              	__HAL_RCC_TIM11_FORCE_RESET
#define TIMx_RCC_RELEASE_RESET             __HAL_RCC_TIM11_RELEASE_RESET

/* Definition for TIMx's NVIC */
#define TIMx_IRQn                    		TIM1_TRG_COM_TIM11_IRQn
#define TIMx_IRQHandler              		TIM1_TRG_COM_TIM11_IRQHandler
/* Macro ---------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
extern UART_HandleTypeDef 		UartHandle;
extern TIM_HandleTypeDef    	TIMxHandle;


void Button_Procedure(void);
void PollingProcedure(void);
void GPIO_Config(void);
void TIM_Config(void);
void USART_Config(void);	
void Error_Handler(void);
/* Exported constants --------------------------------------------------------*/

#endif  /* __HW_CONFIG_H */

