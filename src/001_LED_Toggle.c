/*
 * 001_LED_Toggle.c
 *
 *  Created on: Oct 29, 2019
 *      Author: VKaiser
 */

#include "stm32f407xx.h"

/* Software Delay Function */
void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t gpioLED;										//Initialize Handler

	gpioLED.pGPIOx = GPIOD;										// Point to Port D

	gpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;		//Config pin 12
	gpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;			// Output Mode
	gpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;			// Fast (Normal)
	gpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;		// Output Pin Pullup
	gpioLED.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;			// Already using pullup, no need for this control

	GPIO_PeriClockCtrl(GPIOD, ENABLE);							//Enable the Peripheral Clock
	GPIO_Init(&gpioLED);										//Initialize Pin 12

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);				//Toggle pin 12
		delay();												//Wait, so we can see toggle.
	}

	return 0;
}// main

