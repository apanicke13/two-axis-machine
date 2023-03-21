/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 09/10/2014 11:13:03
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
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

//test git
#include "example.h"
#include "example_usart.h"

// these headers contain motor call def'ns
#include "params.h"
#include "xnucleoihm02a1_interface.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_adc.h"
#include "stdio.h"

//Limit Switches Definitions
#define LIM_SWITCH_X1 GPIO_PIN_5//GPIO_PIN_0 // PA0
#define LIM_SWITCH_X2 GPIO_PIN_6//GPIO_PIN_9 // PB9
#define LIM_SWITCH_Y1 GPIO_PIN_8//GPIO_PIN_7 // PC7 //this one made both motors stop
#define LIM_SWITCH_Y2 GPIO_PIN_9//GPIO_PIN_1 // PA1
#define DEFAULT_Y_SPEED  28000 
#define DEFAULT_X_SPEED  28000

//Motor Definitions
#define M0 0
#define M1 1

#define FWD 1
#define BWD 0

#define NUCLEO_USE_USART

// #define ADC_X_PIN GPIO_PIN_0 // PC0
#define POT_X_ACTUAL_MIN 94 
#define POT_X_ACTUAL_MAX 3366 
#define MOTOR_X_MAX 40000
#define MOTOR_X_DEADBAND 28000

 #define MOTOR_Y_MAX 40000
 #define MOTOR_Y_DEADBAND 28000
// #endif

eL6470_DirId_t x_direction = L6470_DIR_FWD_ID;
eL6470_DirId_t y_direction = L6470_DIR_FWD_ID;

uint16_t Read_ADC(ADC_HandleTypeDef *hadc) {
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 100);

    return HAL_ADC_GetValue(hadc);
}

uint16_t Calibrate_ADC_Val(uint16_t uncalibrated)
{
    return 4095*(uncalibrated - POT_X_ACTUAL_MIN)/(POT_X_ACTUAL_MAX-POT_X_ACTUAL_MIN);
}

/**
  * @defgroup   MotionControl
  * @{
  */

/**
  * @addtogroup BSP
  * @{
  */

/**
  * @}
  */ /* End of BSP */

/**
  * @addtogroup MicrosteppingMotor_Example
  * @{
  */

/**
  * @defgroup   ExampleTypes
  * @{
  */

//#define MICROSTEPPING_MOTOR_EXAMPLE        //!< Uncomment to performe the standalone example
#define MICROSTEPPING_MOTOR_USART_EXAMPLE  //!< Uncomment to performe the USART example
#if ((defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
  #error "Please select an option only!"
#elif ((!defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (!defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
  #error "Please select an option!"
#endif
#if (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE) && (!defined (NUCLEO_USE_USART)))
  #error "Please define "NUCLEO_USE_USART" in "stm32fxxx_x-nucleo-ihm02a1.h"!"
#endif

/**
  * @}
  */ /* End of ExampleTypes */

/**
  * @brief The FW main module
  */
int main(void)
{
  /* NUCLEO board initialization */
  NUCLEO_Board_Init();
  
  /* X-NUCLEO-IHM02A1 initialization */
  BSP_Init();

  //Limit switch initialization
  // lim_switch_init();
  
#ifdef NUCLEO_USE_USART
  /* Transmit the initial message to the PC via UART */
  USART_TxWelcomeMessage();
  	USART_Transmit(&huart2, " X-CUBE-SPN2 v1.0.0\n\r");
#endif

 GPIO_InitTypeDef GPIO_InitStruct;

    __GPIOA_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();
    __GPIOC_CLK_ENABLE();

    //Limit switch Initialization
    // X1
    GPIO_InitStruct.Pin = LIM_SWITCH_X1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // X2
    GPIO_InitStruct.Pin = LIM_SWITCH_X2;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Y1
    GPIO_InitStruct.Pin = LIM_SWITCH_Y1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Y2
    GPIO_InitStruct.Pin = LIM_SWITCH_Y2;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_NVIC_SetPriority((IRQn_Type)(EXTI9_5_IRQn), 0, 0);
    HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI9_5_IRQn)); 

   // ADC Initialization
  // GPIO_InitStruct.Pin = GPIO_PIN_0;
  // GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // MX_ADC1_Init();
  
#if defined (MICROSTEPPING_MOTOR_EXAMPLE)
  /* Perform a batch commands for X-NUCLEO-IHM02A1 */
  MicrosteppingMotor_Example_01();
  
  /* Infinite loop */
  while (1);
#elif defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)
  /* Fill the L6470_DaisyChainMnemonic structure */
  Fill_L6470_DaisyChainMnemonic();
	
	/*Initialize the motor parameters */
	Motor_Param_Reg_Init();

  L6470_HardStop(0);
  L6470_HardStop(1);

  // L6470_Run(0, 1, DEFAULT_X_SPEED);//X Motor
  // L6470_Run(1, 1, DEFAULT_Y_SPEED);//Y Motor
  
  L6470_Move(0, 1, 500000);//X Motor
  L6470_Move(1, 1, 100000);//Y Motor

  // uint8_t buf[16];

  /* Infinite loop */
  while (1)
  {
    /* Check if any Application Command for L6470 has been entered by USART */
    USART_CheckAppCmd(); //This checks motor commands
    // uint16_t adc_convert_val = Read_ADC(&hadc1);
    // num2str(adc_convert_val, buf);
    // USART_Transmit(&huart2, buf);
    // USART_Transmit(&huart2, "\r\n"); //Calibration
  }
#endif
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ /* End of MicrosteppingMotor_Example */

/**
  * @}
  */ /* End of MotionControl */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
