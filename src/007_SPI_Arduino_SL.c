/***************************************************************************************************************************
* This file is subject to the terms and conditions defined in file 'LICENSE.txt',
* which is part of this source code package.
*
* Project						: Use a button press to trigger an interrupt
* Program Name					: 004_Interrupt_Btn
* Author						: vkaiser
* Date Created					: Oct-31-2019
*
* Purpose						: Uses the STM32407 as the SPI Master, and an Arduino as the SPI Slave.
*									SPI in full duplex, DFF=0, SSM=0, SCLK = 2MHz, fclk = 16MHz. 
*									Arduino does not return data -> MISO not required.
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

