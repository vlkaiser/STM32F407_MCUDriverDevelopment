/***************************************************************************************************************************
* This file is subject to the terms and conditions defined in file 'LICENSE.txt',
* which is part of this source code package.
*
* Project						: Toggle LED
* Program Name					: 001_LED_Toggle.c
* Author						: vkaiser
* Date Created					: Oct-29-2019
*
* Purpose						:
*
*
* MCU							: STM32F407VGT6
* Language						: C
* Hardware Modifications		: N/A,
* 								:add external pullup resistor between PD12 and VCC
* Debugger						: ST-Link Debugger (on-board)
*
* Repo / Revision History		: https://github.com/vlkaiser/STM32F407_Drivers
*
* - Special Setup -
* Keil Pack Installer 			: Device Specific DFP
*								:
*
* Revision History				:
* 	Date				Author			Notes
* 	10-30-2019			vkaiser			- Initial commit
*
***************************************************************************************************************************/


#include "stm32f407xx.h"

/* Software Delay Function */
void delay(void)
{
	for(uint32_t i = 0; i < 500000/4; i++);
}

int main(void)
{
	GPIO_Handle_t gpioLED;										//Initialize Handler

	gpioLED.pGPIOx = GPIOD;										// Point to Port D

	gpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;		//Config pin 12
	gpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;			// Output Mode
	gpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;			// Fast (Normal)
	//gpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;		//// Output Pin Pullup
	//gpioLED.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;		//// Already using pullup, no need for this control
	gpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;		// Output Pin Open Drain
	//gpioLED.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PU;		// Needs pullup - 40kohm is too weak
	gpioLED.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;			// We're going to use an external pullup! 320ohm to VCC on the discovery.

	GPIO_PeriClockCtrl(GPIOD, ENABLE);							//Enable the Peripheral Clock
	GPIO_Init(&gpioLED);										//Initialize Pin 12

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);				//Toggle pin 12
		delay();												//Wait, so we can see toggle.
	}

	return 0;
}// main

