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

#define BTN_PRESSED			LOW		//LOW defined in stm32f407xx.h as ENABLE or 1


/* Software Delay Function */
void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{

}
