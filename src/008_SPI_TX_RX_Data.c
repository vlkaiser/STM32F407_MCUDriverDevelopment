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
#include <stdio.h>

//Include capability for semihosting - debugging print statements
//See notes to setup Linker and Debug Configurations in conjunction with this.
extern void initialise_monitor_handles();

#define BTN_PRESSED			HIGH		//HIGH defined in stm32f407xx.h as ENABLE or 1

#define LED_ON				1
#define LED_OFF				0

//Command Codes for the slave to recognize and process
#define COMMAND_LED_CTRL			0x50
#define COMMAND_SENSOR_READ			0x51
#define COMMAND_LED_READ			0x52
#define COMMAND_PRINT				0x53
#define COMMAND_ID_READ				0x54

//Arduino Analog Pins
#define ANALOG_PIN0		0
#define ANALOG_PIN1		1
#define ANALOG_PIN2		2
#define ANALOG_PIN3		3
#define ANALOG_PIN4		4

//Arduino LED Pin
#define LED_PIN			9

// Prototypes
//void delay(void);
void delay(uint32_t interval);
void SPI2_GPIOInits(void);
void SPI2_Inits(void);
void GPIO_ButtonInit(void);
uint8_t SPI_VerifyResponse(uint8_t ackbyte);
uint8_t sendSPI(uint8_t commandcode);

/***************************************************************************************************************/

/*******************************************************************************
 * @fn					- delay
 *
 * @brief				- Software Delay Function
 *
 * @param[in]			- void
 *
 * @return				- void
 *
 * @note				- introduces approx 200ms delay when sysclk is 16MHz
 ******************************************************************************/
void delay(uint32_t interval)
{
	// introduces approx 200ms delay when sysclk is 16MHz
	//for(uint32_t i = 0; i < 500000/2; i++);
	for(uint32_t i = 0; i < interval; i++);
}

/*******************************************************************************
 * @fn					- SPI2_GPIOInits
 *
 * @brief				- Initialize SPI2 GPIO pins
 *
 * @param[in]			- void
 *
 * @return				- void
 *
 * @note				- Select and config GPIO Pins for SPI:  MOSI, MISO, SCLK, NSS
 ******************************************************************************/
void SPI2_GPIOInits(void)
{
	/*
	 * PB15 - SPI2_MOSI -> Arduino 11
	 * PB14 - SPI2_MISO -> Arduino 12
	 * PB10 - SPI2_SCLK -> Arduino 13
	 * PB12 - SPI2_NSS  -> 10
	 * AF Mode: 5
	 */
	GPIO_Handle_t GPIO_SPIPins;

	GPIO_SPIPins.pGPIOx = GPIOB;
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinAltFncMode = 5;
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	//GPIO_SPIPins.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PU;		//because we have a slave connected, PullUp
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_10;
	GPIO_Init(&GPIO_SPIPins);

	// MOSI
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&GPIO_SPIPins);

	// MISO
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&GPIO_SPIPins);


	// NSS - Hardware slave control
	GPIO_SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&GPIO_SPIPins);


	//GPIO_PeriClockCtrl(GPIOB, ENABLE);		//Initialize Peripheral Clock - moved into driver

}

/*******************************************************************************
 * @fn					- SPI2_Inits
 *
 * @brief				- Initialize SPI2 parameters
 *
 * @param[in]			- void
 *
 * @return				- void
 *
 * @note				- For SCLK = 2MHZ, divide HSI SCLK by 8
 * 						- SPI_PeriClockCtrl(SPI2, ENABLE) - Initialize Peripheral Clock moved into driver
 ******************************************************************************/
void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;	//SCLK = 2MHZ, divide HSI SCLK by 8
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;				//Default - idle state, clock is low
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;				//Default
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;					//Hardware Slave Managed via Arduino so NSS Enabled


	SPI_Init(&SPI2Handle);

}

/*******************************************************************************
 * @fn					- GPIO_ButtonInit
 *
 * @brief				- Initialize GPIO push button
 *
 * @param[in]			- void
 *
 * @return				- void
 *
 * @note				-
 ******************************************************************************/
void GPIO_ButtonInit(void)
{
	/* Button Configuration */
	GPIO_Handle_t gpioBtn;										//Initialize Handler

	gpioBtn.pGPIOx = GPIOA;										// Point to Port A

	gpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;			// Config pin 0
	gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;			// Output Mode
	gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;		// Fast (Normal)
	gpioBtn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;		// Schematic PullDown Resistor.

	GPIO_Init(&gpioBtn);										//Initialize Pin 0, initializes peripheral clock control
}

/*******************************************************************************
 * @fn					- SPI_VerifyResponse
 *
 * @brief				- Verifies if Slave return byte is ACK or NACK
 *
 * @param[in]			- uint8_t ackbyte
 *
 * @return				- uint8_t 1 if ACK, 0 if NACK
 *
 * @note				- Takes in ACK/NACK byte from slave, and determines if ACK or NACK
 ******************************************************************************/
uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{

	if(ackbyte == 0xF5)
	{
		// ACK
		return 1;
	}
	else
	{
		// NACK
		return 0;
	}
}

/*******************************************************************************
 * @fn					- sendSPI
 *
 * @brief				- Sends command code over SPI with dummy read, write and receives ACK.
 *
 * @param[in]			- uint8_t commandcode
 *
 * @return				- SPI_VerifyResponse (ACK: 1 or NACK: 0)
 *
 * @note				-
 ******************************************************************************/
uint8_t sendSPI(uint8_t commandcode)
{
	uint8_t dummy_write = 0xAA;
	uint8_t dummy_read;
	uint8_t ackbyte;

	//Enable SPI2 Peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	// Send the Command Codes over SPI to the Slave.
	// If the Slave recognizes the command code, it will send ACK
	// Every time MS/SL sends 1 byte, it receives 1 byte in return

	// 1. Send Command: CMD_LED_CRL <pin no(1)>	<value(1)>
	SPI_SendData (SPI2, &commandcode, 1 );			//Send the command code:  SPI_SendData: SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)

	// 2. Read return (garbage) byte from Slave and clear the RXNE flag
	SPI_ReceiveData(SPI2, &dummy_read, 1);

	// 3. Receive ACK if Valid, NACK if invalid
	// Ask for Data from Slave (8 bit communication, use 1 byte.  16 bit communication, send 2 bytes).
	SPI_SendData (SPI2, &dummy_write, 1);

	// 4. Receive ACK/NACK from Slave
	SPI_ReceiveData(SPI2, &ackbyte, 1);

	// 5. Verify if ACK or NACK
	return (SPI_VerifyResponse(ackbyte));

}

/*******************************************************************************
 * 									 MAIN
 ******************************************************************************/
int main(void)
{

	uint8_t dummy_write = 0xAA;
	uint8_t dummy_read;
	uint8_t args[2];
	uint8_t analog_read;
	uint8_t setLED_State = 1;
	uint8_t id[10];

	uint32_t debounceInterval = 500000/2;
	uint32_t SPI_Interval = 300;		//Seems to be min delay

	//Initialize semihosting print statements in the IDE Console
	initialise_monitor_handles();
	printf("PrintF Statements Initialized\n");

	//Init Push Button
	GPIO_ButtonInit();
	printf("GPIO Initialized\n");

	//Initialize the GPIO Pins to act as SPI2 pins
	SPI2_GPIOInits();

	//Initialize the SPI peripheral
	SPI2_Inits();
	printf("SPI Initialized\n");

	/***
	* Set the Slave Select Output Enable to 1 (Master Mode MCU)
	* SSOE EN = NSS Output Enable
	* NSS pin is automatically managed by the hardware: (SS Active High, MOSI Active High)
	*	eg SPE = 1, NSS = LOW (Slave Enabled)
	*	   SPE = 0, NSS = HIGH (Slave Disabled)
	***/
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		//Wait for Button Press:
		printf("Waiting...\n");
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) );
		delay(debounceInterval);		//debounce delay

		// COMMAND_LED_CTRL
		printf("COMMAND_LED_CTRL:\n");
		uint8_t commandcode = COMMAND_LED_CTRL;			//create a variable for the command codes
		if (sendSPI(commandcode))
		{
			printf("ACK: Send LED CMD.\n");
			//Send Arguments
			args[0] = LED_PIN;				//Arduino Pin
			//args[1] = LED_ON;				//State of pin
			args[1] = setLED_State;				//State of pin
			SPI_SendData(SPI2, args, 2);	//Send data
			printf("LED: %d\n", setLED_State);
		}

		//Wait for Button Press
		printf("Waiting...\n");
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) );
		delay(debounceInterval);		//debounce delay

		// COMMAND_SENSOR_READ
		printf("COMMAND_SENSOR_READ:\n");
		commandcode = COMMAND_SENSOR_READ;

		if ( sendSPI(commandcode) )					//SPI_Send/Read, Send/Read
		{
			printf("ACK: Send SENSOR READ.\n");
			//Send Arguments
			args[0] = ANALOG_PIN1;				//Arduino Pin
			SPI_SendData(SPI2, args, 1);		//SPI_Send (Arduino Pin to read data from)

			delay(SPI_Interval);	//Wait

			// Read Sensor Data
			SPI_ReceiveData(SPI2, &dummy_read, 1);		//SPI_Receive

			//Delay for slave to ready with the data
			delay(SPI_Interval);	//Wait

			SPI_SendData(SPI2, &dummy_write, 1);		//Dummy write to initiate slave transfer

			SPI_ReceiveData(SPI2, &analog_read, 1);		//Read sensor data

			delay(SPI_Interval);	//Wait

			printf("Sensor Data %d \n", analog_read);

		}

		//Wait for Button Press
		printf("Waiting...\n");
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) );
		delay(debounceInterval);		//debounce delay

		// COMMAND_LED_READ
		printf("COMMAND_LED_READ:\n");
		commandcode = COMMAND_LED_READ;			//create a variable for the command codes
		if (sendSPI(commandcode))
		{
			printf("ACK: Read LED\n");
			//Send Arguments
			args[0] = LED_PIN;				//Arduino Pin
			SPI_SendData(SPI2, args, 1);	//Send Pin to read

			// Read Data
			SPI_ReceiveData(SPI2, &dummy_read, 1);		//"ACK" for send - Received Data on the MISO line

			//Delay for slave to ready with the data
			delay(SPI_Interval);

			SPI_SendData (SPI2, &dummy_write, 1);		//Dummy write to initiate slave transfer
			SPI_ReceiveData(SPI2, &analog_read, 1);		//Read led state

			//update next LED_State
			if(analog_read == 1){setLED_State = 0;}
			if(analog_read == 0){setLED_State = 1;}

			//printf("LED State: \n");
			printf("LED State: %d\n", analog_read);
		}

		//Wait for Button Press
		printf("Waiting...\n");
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) );
		delay(debounceInterval);		//debounce delay

		// COMMAND_PRINT
		printf("COMMAND_PRINT:\n");
		commandcode = COMMAND_PRINT;			//create a variable for the command codes
		if (sendSPI(commandcode))
		{
			printf("ACK: Send Print CMD.\n");
			uint8_t msg[] = "This shit is bananas!";
			//Send Arguments
			args[0] = strlen((char*)msg);				//data length is first slave receive, and is used to setup iterating thru msg
			SPI_SendData(SPI2, args, 1);				//Send args

			SPI_SendData(SPI2, msg, args[0]);			//Second slave receive iterates thru 'msg' i times, and you have to pass in data (msg) and datalength (strlen -> args[0])

			printf("Printed\n");
		}


		//Wait for Button Press
		printf("Waiting...\n");
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) );
		delay(debounceInterval);		//debounce delay

		// COMMAND_ID_READ
		printf("COMMAND_ID_READ:\n");
		commandcode = COMMAND_ID_READ;			//create a variable for the command codes
		if (sendSPI(commandcode))
		{
			printf("ACK: Read CMD ID.\n");

			for( int i = 0; i< 10; i++ )
			{
				SPI_SendData (SPI2, &dummy_write, 1);		//Dummy write to initiate slave transfer
				SPI_ReceiveData(SPI2, &id[i], 1);		//Read led state
			}
			id[11] = '\0';		//Terminal character

			//printf("CMD ID\n");
			printf("CMD ID: %s\n", id);
		}

		/* Before we Close SPI we must make sure SPI is not BUSY [BSY = 0] by checking the SPI Status BSY flag */
		// Wait for busy flag != 1
		while( SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG) );

		//Disable the peripheral - return to Idle state
		SPI_PeripheralControl(SPI2, DISABLE);

	}// While()

	return 0;
}

