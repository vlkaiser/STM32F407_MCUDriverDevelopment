/***************************************************************************************************************************
* This file is subject to the terms and conditions defined in file 'LICENSE.txt',
* which is part of this source code package.
*
* Project						: Toggle LED via button press
* Program Name					: 002_LED_Btn.c
* Author						: vkaiser
* Date Created					: Oct-30-2019
*
* Purpose						: Validate the DIY STM32F407 MCU and peripheral GPIO drivers
*
*
* MCU							: STM32F407VGT6
* Language						: C
* Hardware Modifications		: N/A
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

#define BTN_PRESSED			HIGH		//HIGH defined in stm32f407xx.h as ENABLE or 1

/* Software Delay Function */
void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t gpioLEDGreen, gpioBtn;								//Initialize Handlers

	/* LED Configuration */
	gpioLEDGreen.pGPIOx = GPIOD;										// Point to Port D

	gpioLEDGreen.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;		//Config pin 12
	gpioLEDGreen.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;			// Output Mode
	gpioLEDGreen.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;		// Fast (Normal)
	gpioLEDGreen.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;		// Output Pin Pullup
	gpioLEDGreen.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;		// Already using pullup, no need for this control

	GPIO_PeriClockCtrl(GPIOD, ENABLE);								//Enable the Peripheral Clock
	GPIO_Init(&gpioLEDGreen);										//Initialize Pin 12

	/* Button Configuration */
	gpioBtn.pGPIOx = GPIOA;										// Point to Port A

	gpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;			//Config pin 0
	gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;				// Output Mode
	gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;			// Fast (Normal)
	gpioBtn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;			// Schematic PullDown Resistor.

	GPIO_PeriClockCtrl(GPIOA, ENABLE);							//Enable the Peripheral Clock
	GPIO_Init(&gpioBtn);										//Initialize Pin 0

	while(1)
	{
		/* Read state from button pin */
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) == BTN_PRESSED)
		{
			delay();												//debounce for button so only executed once.
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);				//Toggle pin 12
		}
	}

}//main

