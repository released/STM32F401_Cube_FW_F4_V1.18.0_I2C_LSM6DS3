/* Includes ------------------------------------------------------------------*/
#include "Hw_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF); 

	return ch;
}

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* TIM handle declaration */
TIM_HandleTypeDef    			TIMxHandle;

/* UART handler declaration */
UART_HandleTypeDef 			UartHandle;

uint32_t tickstart = 0 ;
uint8_t Flag_Button = 0;

uint8_t Flag_swap = 1;
/* Private functions ---------------------------------------------------------*/
void Button_Procedure(void)
{
	Custom_ButtonScan();

	if (Custom_Button4PressedOnce())
	{
		Flag_Button = (Flag_Button==1)?(0):(1);
		printf("4444\r\n");	
	}	

	if (Custom_Button4PressedLong()&& !Custom_Button4PressedOnce())
	{
		printf("4444 long\r\n");
	}
}

void PollingProcedure(void)
{	
	#if 1	//measure timing
	
	tickstart = HAL_GetTick();
	
	appLSM6DS3_SelfTest();

	Accelerator_filter();
	Gyroscope_filter();

	if (Flag_swap)
	{
		Tilt_Angle_Calculate();
		printf("Get counter :             %8.3lf ms\r\n" , (float)(HAL_GetTick() - tickstart)/1000);		
	}
	else
	{
//		printf("ACC:%6d,%6d,%6d, GYRO:%6d,%6d,%6d\r\n",
//					appLSM6DS3_GetAccData(AXIS_X),appLSM6DS3_GetAccData(AXIS_Y),appLSM6DS3_GetAccData(AXIS_Z),
//					appLSM6DS3_GetGyroData(AXIS_X),appLSM6DS3_GetGyroData(AXIS_Y),appLSM6DS3_GetGyroData(AXIS_Z));		
		printf("Get counter : %8.3lf ms\r\n" , (float)(HAL_GetTick() - tickstart)/1000);
	}

	#else
	
	appLSM6DS3_SelfTest();

	Accelerator_filter();
	Gyroscope_filter();
	
	Tilt_Angle_Calculate();

	#endif
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t TimerCounter_1000ms = 0;	
	static uint16_t TimerCounter_5000ms = 0;		
	static uint32_t tmp = 0;

	//insert application for TIMER (1ms)
	if (TimerCounter_1000ms++ >= 1000)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);	
		printf("%s : %d\n",__FUNCTION__,tmp++);
		TimerCounter_1000ms = 0;
	}

	if (TimerCounter_5000ms++ >= 5000)
	{
		Flag_swap = !Flag_swap;
		TimerCounter_5000ms = 0;
	}

	
}

void GPIO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

  	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
}

void TIM_Config(void)	//1ms
{
//	uint32_t	uTimPrescalerValue = 0;

	/* Compute the prescaler value to have TIMx counter clock equal to 1K Hz */
//	uTimPrescalerValue = (uint32_t) (((SystemCoreClock/4) / 1000) - 1);

	/*##-1- Configure the TIM peripheral #######################################*/ 
	/* Set TIMx instance */
	TIMxHandle.Instance = TIMx;

	/* Initialize TIMx peripheral as follow:
	   + Period = 
	   + Prescaler = SystemCoreClock/10000 Note that APB clock = TIMx clock if
	                 APB prescaler = 1.
	   + ClockDivision = 0
	   + Counter direction = Up
	*/
	TIMxHandle.Init.Period = 7000 - 1;
	TIMxHandle.Init.Prescaler = 12 -1;	//uTimPrescalerValue;
	TIMxHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TIMxHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	if(HAL_TIM_Base_Init(&TIMxHandle) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

	/*##-2- Start the TIM Base generation in interrupt mode ####################*/
	/* Start Channel1 */
	if(HAL_TIM_Base_Start_IT(&TIMxHandle) != HAL_OK)
	{
		/* Starting Error */
		Error_Handler();
	}
}

void USART_Config(void)	
{
	UartHandle.Instance          = USARTx;

	UartHandle.Init.BaudRate     = 115200;
	UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits     = UART_STOPBITS_1;
	UartHandle.Init.Parity       = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode         = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

	if(HAL_UART_Init(&UartHandle) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler(); 
	}

	/* Output a message on Hyperterminal using printf function */
	printf("\r\nUART Printf Example: retarget the C library printf function to the UART\r\n");

	printf("HAL_RCC_GetSysClockFreq = %d\r\n",HAL_RCC_GetSysClockFreq());
 	printf("HAL_RCC_GetHCLKFreq = %d\r\n",HAL_RCC_GetHCLKFreq());
	printf("HAL_RCC_GetPCLK1Freq = %d\r\n",HAL_RCC_GetPCLK1Freq());
	printf("HAL_RCC_GetPCLK2Freq = %d\r\n",HAL_RCC_GetPCLK2Freq());

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
	/* User may add here some code to deal with this error */
	while(1)
	{
	}
}


