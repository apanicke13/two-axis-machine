#Two-Axis_machine

This code is an example of controlling microstepping motors with the X-NUCLEO-IHM02A1 motor driver using an STM32F4 microcontroller. The code uses a variety of headers and libraries to interface with the motor driver, read from an analog-to-digital converter (ADC), and interface with a computer via USART.

<b><h2>Dependencies</b></h2>

To use this code, you will need the following:

- X-NUCLEO-IHM02A1 motor driver
- STM32F4 microcontroller
- Keil μVision IDE
- X-CUBE-SPN2 software pack
- STMicroelectronics STM32CubeMX
- C programming knowledge

<b><h2>Hardware setup</b></h2>

The hardware setup for this code involves connecting the X-NUCLEO-IHM02A1 motor driver to the STM32F4 microcontroller.

<b><h2>Usage</b></h2>

Open the project in Keil μVision IDE.
Compile and download the code to the STM32F4 microcontroller.
The microcontroller will interface with the X-NUCLEO-IHM02A1 motor driver and read from the ADC to control the microstepping motors.
Connect the microcontroller to a computer via USART to interface with it.

<b><h2>Files</b></h2>

- example.h - contains declarations for functions used in the example code
- example_usart.h - contains declarations for USART functions used in the example code
- params.h - contains definitions for motor call definitions
- xnucleoihm02a1_interface.h - contains definitions for the X-NUCLEO-IHM02A1 motor driver interface
- stm32f4xx_it.h - contains declarations for interrupt handlers
- stm32f4xx_hal_adc.h - contains definitions for ADC functions used in the example code
- stdio.h - contains definitions for standard input/output functions used in the example code

<b><h2>Functionality</b></h2>

This code uses several definitions and initializations to set up the microcontroller to interface with the X-NUCLEO-IHM02A1 motor driver and ADC. The code then sets up limit switches and initializes the motors. The main function of the code is then run, which interfaces with the X-NUCLEO-IHM02A1 motor driver to control the microstepping motors. The code also includes USART functionality to interface with a computer.

<b><h2>Examples</b></h2>

This code includes two examples:

- MICROSTEPPING_MOTOR_EXAMPLE - standalone example
- MICROSTEPPING_MOTOR_USART_EXAMPLE - USART example

To run a particular example, uncomment the corresponding line in the code.
