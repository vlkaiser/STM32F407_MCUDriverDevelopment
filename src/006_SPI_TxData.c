/***************************************************************************************************************************
* This file is subject to the terms and conditions defined in file 'LICENSE.txt',
* which is part of this source code package.
*
* Project						: Test SPI Send Data (Blocking/Polling API) Test
* Program Name					: 006_SPI_TxData.c
* Author						: vkaiser
* Date Created					: Jan 07, 2020
*
* Purpose						: Validate the DIY STM32F407 MCU and peripheral SPI Drivers
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
* 	01-07-2020			vkaiser			- Initial commit
*
***************************************************************************************************************************/
#include "stm32f407xx.h"
#include "string.h"

/* Software Delay Function */
void delay(void)
{
	// introduces approx 200ms delay when sysclk is 16MHz
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{


	return 0;
}
