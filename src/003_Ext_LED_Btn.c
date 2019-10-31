/***************************************************************************************************************************
* This file is subject to the terms and conditions defined in file 'LICENSE.txt',
* which is part of this source code package.
*
* Project						: Toggle external LED via external button press
* Program Name					: 003_Ext_LED_Btn.c
* Author						: vkaiser
* Date Created					: Oct-30-2019
*
* Purpose						: Validate the DIY STM32F407 MCU and peripheral GPIO drivers
*
*
* MCU							: STM32F407VGT6
* Language						: C
* Hardware Modifications		: PB12 to pushbutton(pb) pin 1,2; VCC to pb pin 1,2; GND to pb 3,4; Pullup Resistor to VCC and LED Anode;  LED Cathode to PA10
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

#define BTN_PRESSED			LOW		//LOW defined in stm32f407xx.h as ENABLE or 1


/* Software Delay Function */
void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t gpioExtLED, gpioExtBtn;								//Initialize Handlers

	/* LED Configuration */
	gpioExtLED.pGPIOx = GPIOA;										// Point to Port A

	gpioExtLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_10;		//Config pin 10
	gpioExtLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;			// Output Mode
	gpioExtLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;		// Fast (Normal)
	gpioExtLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;		// Output Pin Push Pull
	gpioExtLED.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;		// External pullup, no need for this control

	GPIO_PeriClockCtrl(GPIOA, ENABLE);								//Enable the Peripheral Clock
	GPIO_Init(&gpioExtLED);											//Initialize Pin 14

	/* Button Configuration	*/
	gpioExtBtn.pGPIOx = GPIOB;										// Point to Port B

	gpioExtBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;			//Config pin 12
	gpioExtBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;				// Output Mode
	gpioExtBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;			// Fast (Normal)
	gpioExtBtn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PU;			// Schematic PullDown Resistor.

	GPIO_PeriClockCtrl(GPIOB, ENABLE);							//Enable the Peripheral Clock
	GPIO_Init(&gpioExtBtn);										//Initialize Pin 12

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_12) == BTN_PRESSED)
		{
			delay();												//debounce for button so only executed once.
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_10);				//Toggle LED pin 10
		}

	}

}//main

