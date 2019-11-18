/***************************************************************************************************************************
* This file is subject to the terms and conditions defined in file 'LICENSE.txt',
* which is part of this source code package.
*
* Project						: Use a button press to trigger an interrupt
* Program Name					: 004_Interrupt_Btn
* Author						: vkaiser
* Date Created					: Oct-31-2019
*
* Purpose						: Validate the DIY STM32F407 MCU and peripheral GPIO drivers
*
*
* MCU							: STM32F407VGT6
* Language						: C
* Hardware Modifications		:
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
* 	10-31-2019			vkaiser			- Initial commit
*
***************************************************************************************************************************/
#include "stm32f407xx.h"
#include "string.h"

#define BTN_PRESSED			LOW		//LOW defined in stm32f407xx.h as ENABLE or 1


/* Software Delay Function */
void delay(void)
{
	// introduces approx 200ms delay when sysclk is 16MHz
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLED, GpioBtn;	//Create new struct instances
	// Set struct members to 0:
	memset(&GpioLED, 0, sizeof(GpioLED));
	memset(&GpioBtn, 0, sizeof(GpioBtn));

	//Config LED GPIO
	GpioLED.pGPIOx = GPIOD;
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLED.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;

	//Config Btn GPIO
	GpioBtn.pGPIOx = GPIOD;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PU;
	GpioBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_PeriClockCtrl(GPIOD, ENABLE);

	GPIO_Init(&GpioLED);
	GPIO_Init(&GpioBtn);

	//IRQ Configurations
	GPIO_IRQPriorityConfig(IRQ_EXTI9_5, NVIC_IRQ_PRI_15);		//Not strictly needed because only 1 interrupt
	GPIO_IRQInterruptConfig(IRQ_EXTI9_5, ENABLE);

	while(1);			//Wait for interrupt


	return 0;
}

void EXTI9_5_IRQHandler(void)
{
	delay();	//200ms debounce delay

	//Handle the interrupt
	GPIO_IRQHandling(GPIO_PIN_5);		//Takes in Pin Number for interrupt

	//Do Something
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
}
