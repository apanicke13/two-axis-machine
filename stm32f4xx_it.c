/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @date    13/05/2015 09:14:38
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "xnucleoihm02a1_interface.h"
#include "example_usart.h"
#include "xnucleoihm02a1.h"
#include "L6470.h"

#define DEBOUNCE_DELAY 50
#define DEFAULT_Y_SPEED  28000
#define DEFAULT_X_SPEED  28000

/**
  * @addtogroup MicrosteppingMotor_Example
  * @{
  */

/**
  * @addtogroup STM32F4XX_IT
  * @{
  */

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
  * @addtogroup STM32F4XX_IT_Exported_Functions
  * @{
  */

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI Line1 interrupt.
*/

void EXTI1_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

/**
* @brief This function handles EXTI Line0 interrupt.
*/

void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
* @brief This function handles USART2 global interrupt.
*/

void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
  USART_ITCharManager(&huart2);
}

/**
* @brief This function handles EXTI Line[15:10] interrupts.
*/

void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

uint32_t startTick_9_5 = 0U;
uint8_t startDebouncing_9_5 = 1U;

//PC5 PC6 PC8 PC9
// need an adapter to connect to the through holes
void EXTI9_5_IRQHandler(void)
{
  if (startDebouncing_9_5 == 1U)
  {
    startTick_9_5 = HAL_GetTick();
    startDebouncing_9_5 = 0U;
    // Don't clear interrupt until debounce delay is exceeded
  }
  else if (HAL_GetTick() > startTick_9_5 + DEBOUNCE_DELAY)
  {
    //Switch 5 for X1 using pin PC5
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == 1)
    {
      L6470_HardStop(0);
      L6470_Run(0,1,DEFAULT_X_SPEED);
    }
    // Switch 6 for X2 using pin PC6
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == 1)
    {
      L6470_HardStop(0);
      L6470_Run(0,0,DEFAULT_X_SPEED);
    }
    // Switch 8 for Y1 using pin PC8
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 1)
    {
      L6470_HardStop(1);
      L6470_Run(1,1,DEFAULT_Y_SPEED);
    }
    // Switch 9 for Y2 using pin PC9
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == 1)
    {
      L6470_HardStop(1);
      L6470_Run(1,0,DEFAULT_Y_SPEED);
    }

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);
    startDebouncing_9_5 = 1U;

  }
}
/**
  * @}
  */ /* End of STM32F4XX_IT_Exported_Functions */

/**
  * @}
  */ /* End of STM32F4XX_IT */

/**
  * @}
  */ /* End of MicrosteppingMotor_Example */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/