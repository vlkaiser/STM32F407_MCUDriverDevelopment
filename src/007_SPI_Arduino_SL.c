/***************************************************************************************************************************
* This file is subject to the terms and conditions defined in file 'LICENSE.txt',
* which is part of this source code package.
*
* Project						: Use a button press to trigger an interrupt
* Program Name					: 004_Interrupt_Btn
* Author						: vkaiser
* Date Created					: Feb-20-2020
*
* Purpose						: Uses the STM32407 as the SPI Master, and an Arduino as the SPI Slave.
*									SPI in full duplex, DFF=0, SSM=0, SCLK = 2MHz, fclk = 16MHz. 
*									Arduino does not return data -> MISO not required.
*
* MCU							: STM32F407VGT6 and Arduino Uno
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
* 	2-20-2020			vkaiser			- Initial commit
*
***************************************************************************************************************************/

#include "stm32f407xx.h"
#include "string.h"

#define BTN_PRESSED			HIGH		//HIGH defined in stm32f407xx.h as ENABLE or 1

void GPIO_ButtonInit(void);

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
	 * PB15 - SPI2_MOSI -> Arduino 11
	 * PB14 - SPI2_MISO -> Not Used
	 * PB10 - SPI2_SCLK -> Arduino 13
	 * PB12 - SPI2_NSS  -> 10
	 * AF Mode: 5
	 */
	GPIO_Handle_t GPIO_SPIPins;

	GPIO_SPIPins.pGPIOx = GPIOB;
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinAltFncMode = 5;
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_10;
	GPIO_Init(&GPIO_SPIPins);

	// MOSI
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&GPIO_SPIPins);

	// MISO - Unused in this application
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&GPIO_SPIPins);


	// NSS - Used in this application
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&GPIO_SPIPins);


	//GPIO_PeriClockCtrl(GPIOB, ENABLE);		//Initialize Peripheral Clock - moved into driver

}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;				//Default
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;				//Default - idle state, clock is low
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;					//Hardware Slave Managed via Arduino so NSS disabled
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;	//SCLK = 2MHZ, divide HSI SCLK by 8

	SPI_Init(&SPI2Handle);

	//SPI_PeriClockCtrl(SPI2, ENABLE);		//Initialize Peripheral Clock - moved into driver

}

void GPIO_ButtonInit(void)
{
	/* Button Configuration */
	GPIO_Handle_t gpioBtn;										//Initialize Handler

	gpioBtn.pGPIOx = GPIOA;										// Point to Port A

	gpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;			// Config pin 0
	gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;			// Output Mode
	gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;		// Fast (Normal)
	gpioBtn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;		// Schematic PullDown Resistor.

	GPIO_Init(&gpioBtn);										//Initialize Pin 0
}


int main(void)
{
	char user_data[] = "Hello World!";		//Data to Send

	//Init Button
	GPIO_ButtonInit();

	//Initialize the GPIO Pins to act as SPI2 pins
	SPI2_GPIOInits();

	//Initialize the SPI peripheral
	SPI2_Inits();

	/***
	* Set the Slave Select Output Enable to 1 (Master Mode MCU)
	* SSOE EN = NSS Output Enable
	* NSS pin is automatically managed by the hardware: (SS Active High, MOSI Active High)
	*	eg SPE = 1, NSS = LOW
	*	   SPE = 0, NSS = HIGH
	***/
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		//Wait for Button Press:
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) );

		delay();		//debounce delay

		//Enable SPI2 Peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//First send the LENGTH or # of Bytes information so the Slave knows how many bytes to receive.
		uint8_t dataLen = strlen(user_data);
		SPI_SendData( SPI2, &dataLen, 1 );

		//Then send the DATA
		//SPI_SendData: SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
		//SPI_SendData( SPI2, user_data, strlen(user_data) );			//NOTE: *pTXBuffer is uint8_t, and user_data is char.  Must TYPECAST
		SPI_SendData( SPI2, (uint8_t*)user_data, strlen(user_data) );


		/* Before we Close SPI we must make sure SPI is not BUSY [BSY = 0] by checking the SPI Status BSY flag */
		// Wait for busy flag != 1
		while( SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG) );

		//Disable the peripheral - return to Idle state
		SPI_PeripheralControl(SPI2, DISABLE);

		//Hardware Slave Select
		//The slave will not respond unless SS on the slave is pulled to low.
		//SSM = 0 (disabled), SPE = 1, NSS (Slave select) will be low automatically
		//SSM = 1, SPE = 0, NSS will be high automatically
		//In order to ENABLE the NSS we need the SSOE control bit to be enabled.

	}

	return 0;
}

