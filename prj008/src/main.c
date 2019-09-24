/*
  ******************************************************************************
  070253 - Circuitos Microprocessados
  Prof. Lucio Rene Prade
  Exemplo  - Timer
  ******************************************************************************
 */


#include "main.h"

/* Struct definida em  stm32f0xx_hal_tim.h para configrar o timer*/
TIM_HandleTypeDef    TimHandle;

/* Struct definida em  stm32f0xx_hal_gpio.h para configrar o port*/
static GPIO_InitTypeDef  GPIO_InitStruct;

/* Declaração da variável Prescaler */
uint32_t uwPrescalerValue = 0;

static void SystemClock_Config(void);
static void Error_Handler(void);

int main(void) {

  /* Inicializa as bibliotecas HAL */
  HAL_Init();

  /* Configura o clock do sistema */
  SystemClock_Config();

  /* Habilita o Clock no port do led,  função definida em stm32f0xx_hal_rcc.h*/
   __GPIOA_CLK_ENABLE();

   /* Configura  o pino do led como output push-pull */
   GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
   GPIO_InitStruct.Pin 	= GPIO_PIN_5;

   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


   /* Configura a variável prescaler com valor de contagem  para 10000 Hz */
   uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;

   /* Configura TIM1 */
   TimHandle.Instance = TIM3;
   TimHandle.Init.Period            = 10000 - 1;
   TimHandle.Init.Prescaler         = uwPrescalerValue;
   TimHandle.Init.ClockDivision     = 0;
   TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
   TimHandle.Init.RepetitionCounter = 0;
   HAL_TIM_Base_Init(&TimHandle);

   /* Configura a geração de interrupção para o timer 1 */
   HAL_TIM_Base_Start_IT(&TimHandle);



   while (1) {

   }


}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}



/* Função de configuração do clock
 *            System Clock source            = PLL (HSI/2)
 *            SYSCLK(Hz)                     = 48000000
 *            HCLK(Hz)                       = 48000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            HSI Frequency(Hz)              = 8000000
 *            PREDIV                         = 1
 *            PLLMUL                         = 12
 *            Flash Latency(WS)              = 1
 * @param  None
 * @retval None
 */

static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* No HSE Oscillator on Nucleo, Activate PLL with HSI/2 as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)!= HAL_OK)
  {
    Error_Handler();
  }
}


/* Função chamada no caso de erro na configuração */
static void Error_Handler(void)
{
   while(1);
}

