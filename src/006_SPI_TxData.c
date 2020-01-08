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

void SPI2_GPIOInits(void)
{
	//Select and config GPIO Pins for SPI:  MOSI, (No MISO req'd - no slave), SCLK, (No NSS Req'd - no slave)
	/*
	 * PB15 - SPI2_MOSI
	 * PB14 - SPI2_MISO
	 * PB13 - SPI2_SCLK
	 * PB12 - SPI2_NSS
	 * AF Mode: 5
	 */
	GPIO_Handle_t GPIO_SPIPins;

	GPIO_SPIPins.pGPIOx = GPIOB;
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinAltFncMode = 5;
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// MOSI
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&GPIO_SPIPins);

	// MISO
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&GPIO_SPIPins);

	// SCLK
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&GPIO_SPIPins);

	// NSS
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&GPIO_SPIPins);

	GPIO_PeriClockCtrl(GPIOB, ENABLE);		//Initialize Peripheral Clock

}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;				//Default
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;				//Default
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;					//Software Slave Managed for NSS
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;	//SCLK 8MHz

	SPI_Init(&SPI2Handle);

	SPI_PeriClockCtrl(SPI2, ENABLE);		//Initialize Peripheral Clock

}

int main(void)
{
	//Initialize the GPIO Pins to act as SPI2 pins
	SPI2_GPIOInits();

	//Initialize the SPI peripheral
	SPI2_Inits();

	//SPI_Handle - set parameters *pSPIHandle
	//SPI_Init - send SPI_Handle: SPI_Init(SPI_Handle_t *pSPIHandle)
	//SPI_SendData: SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)



	return 0;
}
