//#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"

/**********************************************************************************************************************
 *										APIs supported by this driver
 *						For more information about the APIs - see function definitions
 **********************************************************************************************************************/

/*************************************** Private Function Prototypes **************************************************/

//Interrupt Event Handler Functions - take in the SPIHandle struct
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);



/**********************************************************************
 * @fn					- SPI_PeriClockCtrl
 *
 * @brief				- Enable or Disable Peripheral Clock
 *
 * @param[in]			- Base address of SPI peripheral Ex: pSPIx = SPI1
 * @param[in]			- ENABLE or DISABLE macro (MCU header file)
 *
 * @return				- void
 *
 * @note				- none
 **********************************************************************/
void SPI_PeriClockCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}//SPI_PeriClockCtrl

/**********************************************************************
* @fn					- SPI_Init
*
* @brief				- Initialize the registers of the given SPI
*
* @param[in]			- Base address of Handle Structure for a SPI Pin (pSPIx and SPIConfig)
* @param[in]			-
*
* @return				- void
*
* @note				- none
**********************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// SPI_CR1 Register
	uint32_t tempreg = 0;

	/* Enable Peripheral clock for SPI */
	SPI_PeriClockCtrl(pSPIHandle->pSPIx, ENABLE);		//Initialize Peripheral Clock

	// Device Mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// Bus Config
	if( pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//Bit 15 = 0, 	Bit 14 = DNC, Bit 10 = 0
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);		//15 = BIDIMODE, 14 = BIDIOE, 10 = RXONLY

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//Bit 15 = 1, Bit 14 = 1 (TX only), Bit 10 = DNC
		tempreg |= (1 << SPI_CR1_BIDIMODE);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX_ONLY)
	{
		//Bit 15 = 0, Bit 14 = DNC, Bit 10 = 1
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// SPI CLK Speed
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// SPI DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// SPI CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// SPI CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	// SPI SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	// Write to CR1 Register:
	pSPIHandle->pSPIx->SPI_CR1 = tempreg;


}//SPI_Init

/**********************************************************************
 * @fn					- SPI_DeInit
 *
 * @brief				- De-Initialize the registers of the given SPI (Reset to default)
 *
 * @param[in]			- Base address of SPI peripheral Ex: pSPIx = SPIA
 * @param[in]			-
 *
 * @return				- void
 *
 * @note				-
 **********************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}


}//SPI_DeInit

/**********************************************************************
 * @fn					- SPI_PeripheralControl
 *
 * @brief				- Enable or Disable SPI peripheral
 *
 * @param[in]			- Base address of SPI peripheral Ex: pSPIx = SPI1
 * @param[in]			- Enable or Disable
 *
 * @return				- void
 *
 * @note				-
 **********************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if( EnorDi == ENABLE )
	{
		pSPIx->SPI_CR1 |= ( 1 << SPI_CR1_SPE );		//Set SPE bit to 1, Enable
	}else
	{
		pSPIx->SPI_CR1 &= ~( 1 << SPI_CR1_SPE );	//Clear SPE bit, Disable
	}

}//SPI_PeripheralControl

/**********************************************************************
 * @fn					- SPI_SSIConfig
 *
 * @brief				- Enable or Disable SPI peripheral Internal Slave Select
 *
 * @param[in]			- Base address of SPI peripheral Ex: pSPIx = SPI1
 * @param[in]			- Enable or Disable
 *
 * @return				- void
 *
 * @note				- If SSI is LOW (SW Mode), OR NSS is LOW (HW Mode), MODF will not allow SPI MSTR or SPE set
 **********************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if( EnorDi == ENABLE )
	{
		pSPIx->SPI_CR1 |= ( 1 << SPI_CR1_SSI );		//Set SSI bit to 1, Enable
	}else
	{
		pSPIx->SPI_CR1 &= ~( 1 << SPI_CR1_SSI );	//Clear SSI bit, Disable
	}

}//SPI_SSIConfig

/**********************************************************************
 * @fn					- SPI_SSOEConfig
 *
 * @brief				- Enable or Disable SPI peripheral Internal Slave Select Output Enable
 *
 * @param[in]			- Base address of SPI peripheral Ex: pSPIx = SPI1
 * @param[in]			- Enable or Disable
 *
 * @return				- void
 *
 * @note				- If MCU is Master, SSOE Enables NSS with automatic hardware slave select control (SPE)
 **********************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if( EnorDi == ENABLE )
	{
		pSPIx->SPI_CR2 |= ( 1 << SPI_CR2_SSOE );		//Set SSI bit to 1, Enable
	}else
	{
		pSPIx->SPI_CR2 &= ~( 1 << SPI_CR2_SSOE );	//Clear SSI bit, Disable
	}

}//SPI_SSOEConfig

/***************************************************************************************************
 * @fn					- SPI_SendData
 *
 * @brief				- Data structure Base Address, Data and length to TX Data
 *
 * @param[in]			- Base address of SPI peripheral Ex: pSPIx = SPIA
 * @param[in]			- Buffer of Data to Send
 * @param[in]			- Data Length
 *
 * @return				- void
 *
 * @note				- Blocking API: The function call will wait until all bytes are transmitted
 * 						  Polling API (Polls for Flag Status - could hang permanently. Will need watchdog)
 ***************************************************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{

	while (len > 0)
	{
		//Wait until TX Buffer Empty:
		while( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );

		//if DFF = 1 (16 bit)
		if( pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF) )
		{
			//put 2 bytes into DR:
				//Typecast (uint8 pointer-to-buffer [1 byte]) to (uint16 pointer-to-buffer [2 bytes]), then Dereference that pointer
			pSPIx->SPI_DR = *((uint16_t*)pTxBuffer);

			len -= 2; 					//decrement length remaining by 2 bytes

			(uint16_t*)pTxBuffer++;		//Increment TxBuffer by 2 bytes (cast to 2 bytes)

		}else
		{
			//put 1 byte into DR:
				//Dereferenced pointer-to-buffer
			pSPIx->SPI_DR = *pTxBuffer;

			len--;				//decrement length remaining by 1 byte

			pTxBuffer++;		//Increment TxBuffer by 1 byte
		}

	}


}//SPI_SendData

/**********************************************************************
 * @fn					- SPI_ReceiveData
 *
 * @brief				- Data structure Base Address, Data and length of Data to RX
 *
 * @param[in]			- Base address of SPI peripheral Ex: pSPIx = SPIA
 * @param[in]			- Buffer of Data to RX
 * @param[in]			- Data Length
 *
 * @return				- void
 *
 * @note				-
 **********************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	while( len > 0)
	{
		//Wait until RX Buffer Full:
		while( SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET );

		//if DFF = 1 (16 bit)
		if( pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF) )
		{
			//READ 2 bytes FROM DR:
				//Typecast (uint8 pointer-to-buffer [1 byte]) to (uint16 pointer-to-buffer [2 bytes]), then Dereference that pointer

			*((uint16_t*)pRxBuffer) = pSPIx->SPI_DR;

			len -= 2; 					//decrement length remaining by 2 bytes

			(uint16_t*)pRxBuffer++;		//Increment TxBuffer by 2 bytes (cast to 2 bytes)

		}else
		{
			//Read 1 byte from DR:
				//Dereferenced pointer-to-buffer
			*pRxBuffer = pSPIx->SPI_DR;

			len--;				//decrement length remaining by 1 byte

			pRxBuffer++;		//Increment TxBuffer by 1 byte
		}

	}

}//SPI_ReceiveData

/**********************************************************************
 * @fn					- SPI_IRQInterruptConfig
 *
 * @brief				- Configure Interrupt
 *
 * @param[in]			- Interrupt Number
 * @param[in]			- Enable or Disable
 * @param[in]			-
 *
 * @return				- void
 *
 * @note				-
 **********************************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		//If Enable
		if(IRQNumber <= 31)
		{
			//program ISER0 registers
			*NVIC_ISER0 |= ( 1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 registers
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 registers - IRQ only goes up to 80
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}

	}
	else
	{
		if(IRQNumber <= 31)
		{
			//Program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 registers
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ICER2 registers - IRQ only goes up to 80
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}

	}

}//SPI_IRQConfig

/**********************************************************************
 * @fn					- SPI_IRQPriorityConfig
 *
 * @brief				- Configure Interrupt Priority for a pin number
 *
 * @param[in]			- IRQ Priority
 * @param[in]			- IRQ Number
 * @param[in]			-
 *
 * @return				- void
 *
 * @note				-
 **********************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQ_Number, uint32_t IRQ_Priority)
{

}//SPI_IRQPriorityConfig

/**********************************************************************
 * @fn					- SPI_IRQHandling
 *
 * @brief				- Determine which interrupt triggered the event
 *
 * @param[in]			- SPIHandle struct
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- void
 *
 * @note				- Checks TXE, RXNE, ERRgm
 **********************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t tempSR, tempCtrl;

	//Check for TXE:
	// bitwise AND Status Register with a 1 shifted to the TXEIE position
	// If the bit is set in the interrupt, tempSR will return 1.  Else, 0.
	tempSR = pSPIHandle->pSPIx->SPI_SR & ( 1 << SPI_SR_TXE );

	//Check if the TXEIE (Transmit Interrupt Enable) control bit is set
	tempCtrl = pSPIHandle->pSPIx->SPI_CR2 & ( 1 << SPI_CR2_TXEIE );

	//If both are true:
	if (tempSR && tempCtrl)
	{
		//Handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//Check for RXE and RXEIE
	tempSR = pSPIHandle->pSPIx->SPI_SR & ( 1 << SPI_SR_RXNE );
	tempCtrl = pSPIHandle->pSPIx->SPI_CR2 & ( 1 << SPI_CR2_RXNIE );

	//If both are true:
	if (tempSR && tempCtrl)
	{
		//Handle RXE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	//Check for Errors
	// Not using Mixed Master, CRC or TI-specific frame format.  Not implementing those error checking.
	tempSR = pSPIHandle->pSPIx->SPI_SR & ( 1 << SPI_SR_OVR );
	tempCtrl = pSPIHandle->pSPIx->SPI_CR2 & ( 1 << SPI_CR2_ERRIE );

	//If both are true:
	if (tempSR && tempCtrl)
	{
		//Handle RXE
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}


}//SPI_IRQHandling

/**********************************************************************
 * @fn					- spi_txe_interrupt_handle
 *
 * @brief				- Handles TXE Interrupt Event
 *
 * @param[in]			- SPIHandle struct
 *
 * @return				- void
 *
 * @note				-
 **********************************************************************/
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	//if DFF = 1 (16 bit)
	if( pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF) )
	{
		//put 2 bytes into DR:
			//Typecast (uint8 pointer-to-buffer [1 byte]) to (uint16 pointer-to-buffer [2 bytes]), then Dereference that pointer
		pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pTxBuffer);

		pSPIHandle->TxLen -= 2; 				//decrement length remaining by 2 bytes

		(uint16_t*)pSPIHandle->pTxBuffer++;		//Increment TxBuffer by 2 bytes (cast to 2 bytes)

	}else
	{
		//put 1 byte into DR:
			//Dereferenced pointer-to-buffer
		pSPIHandle->pSPIx->SPI_DR = *pSPIHandle->pTxBuffer;

		pSPIHandle->TxLen--;			//decrement length remaining by 1 byte

		pSPIHandle->pTxBuffer++;		//Increment TxBuffer by 1 byte
	}

	//When data length is 0, close the communication, inform app that Tx is over.
	if( ! pSPIHandle->TxLen )
	{
		//1. disable TXEIE, prevent new interrupts
		pSPIHandle->pSPIx->SPI_CR2 &= ~( 1 << SPI_CR2_TXEIE );

		//2. Reset the buffers
		SPI_CloseTransmission(pSPIHandle);

		//3. Inform the Application
		// The application must implement this callback
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_COMPLETE);

	}
}//spi_txe_interrupt_handle

/**********************************************************************
 * @fn					- spi_rxne_interrupt_handle
 *
 * @brief				- Handles RXNE Interrupt Event
 *
 * @param[in]			- SPIHandle struct
 *
 * @return				- void
 *
 * @note				-
 **********************************************************************/
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//if DFF = 1 (16 bit)
	if( pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF) )
	{
		//READ 2 bytes FROM DR:
			//Typecast (uint8 pointer-to-buffer [1 byte]) to (uint16 pointer-to-buffer [2 bytes]), then Dereference that pointer

		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR;

		pSPIHandle->RxLen -= 2; 				//decrement length remaining by 2 bytes

		(uint16_t*)pSPIHandle->pRxBuffer++;		//Increment RxBuffer by 2 bytes (cast to 2 bytes)

	}else
	{
		//Read 1 byte from DR:
			//Dereferenced pointer-to-buffer
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->SPI_DR;

		pSPIHandle->RxLen--;			//decrement length remaining by 1 byte

		pSPIHandle->pRxBuffer++;		//Increment RxBuffer by 1 byte
	}

	//When data length is 0, close the communication, inform app that Rx is over.
	if( ! pSPIHandle->RxLen )
	{
		//1. disable RXNIE, prevent new interrupts
		pSPIHandle->pSPIx->SPI_CR2 &= ~( 1 << SPI_CR2_RXNIE );

		//2. Reset the buffers
		SPI_CloseReception(pSPIHandle);

		//3. Inform the Application
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_COMPLETE);
	}

}//spi_rxne_interrupt_handle

/**********************************************************************
 * @fn					- spi_ovr_err_interrupt_handle
 *
 * @brief				- Handles Overrun Error Interrupt Event
 *
 * @param[in]			- SPIHandle struct
 *
 * @return				- void
 *
 * @note				-
 **********************************************************************/
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp;
	//1. Clear the OVR Flag:  Read the DR reg, Read the SR reg.
	// Make sure the application is not engaged
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->SPI_DR;
		temp = pSPIHandle->pSPIx->SPI_SR;
	}
	(void)temp;	//for temp variables that are unused (not called)

	//2. Inform the application.
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);

	//If the application is busy, it must call ClearOVRFlag explicitly.
	//The application must implement CloseTransmission.

}//spi_ovr_err_interrupt_handle


/**********************************************************************
 * @fn					- SPI_CloseTransmission
 *
 * @brief				- The application may call this function to close TX
 *
 * @param[in]			- SPIHandle struct
 *
 * @return				- void
 *
 * @note				-
 **********************************************************************/
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	//1. disable TXEIE, prevent new interrupts
	pSPIHandle->pSPIx->SPI_CR2 &= ~( 1 << SPI_CR2_TXEIE );

	//2. Reset the buffers
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}//SPI_CloseTransmission

/**********************************************************************
 * @fn					- SPI_CloseReception
 *
 * @brief				- The application may call this function to close RX
 *
 * @param[in]			- SPIHandle struct
 *
 * @return				- void
 *
 * @note				-
 **********************************************************************/
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	//1. disable RXNIE, prevent new interrupts
	pSPIHandle->pSPIx->SPI_CR2 &= ~( 1 << SPI_CR2_RXNIE );

	//2. Reset the buffers
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}//SPI_CloseReception

/**********************************************************************
 * @fn					- SPI_ClearOVRFlag
 *
 * @brief				- The application may call this function to clear OVR Error
 *
 * @param[in]			- SPI_RegDef pSPIx
 *
 * @return				- void
 *
 * @note				-
 **********************************************************************/
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->SPI_DR;
	temp = pSPIx->SPI_SR;

	(void)temp;	//for temp variables that are unused (not called)

}//SPI_ClearOVRFlag

/**********************************************************************
 * @fn					- SPI_SendDataIT
 *
 * @brief				- Transmit data on interrupt - when data is available to send, trigger an interrupt to send it
 *
 * @param[in]			- struct of parameters *pSPIHandle
 * @param[in]			- data to transmit *pTxBuffer
 * @param[in]			- length of data
 *
 * @return				- State (of SPI register TX_BUSY)
 *
 * @note				- API to send data with interrupt mode.
 *	 					  Use the Handler to actually write data to the data register
 **********************************************************************/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->TxState;

	if( state != SPI_BUSY_IN_TX)
	{
		//1. Save the TX buffer address and LEN information in global variables (Handle Structure)
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;

		//2. Mark the SPI state as busy in TX so no other code can take over the same SPI peripheral until TX is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->SPI_CR2 |= ( 1 << SPI_CR2_TXEIE);

	}

	return state;

}//SPI_SendDataIT

/**********************************************************************
 * @fn					- SPI_ReceiveDataIT
 *
 * @brief				- Receive data on interrupt - when data is available to read, trigger an interrupt to read it
 *
 * @param[in]			- struct of parameters *pSPIHandle
 * @param[in]			- data to Read *pRxBuffer
 * @param[in]			- length of data
 *
 * @return				- State (of SPI register RX_BUSY)
 *
 * @note				- API to send data with interrupt mode.
 *	 					  Use the Handler to actually write data to the data register
 **********************************************************************/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->RxState;

		if( state != SPI_BUSY_IN_RX)
		{
			//1. Save the TX buffer address and LEN information in global variables (Handle Structure)
			pSPIHandle->pRxBuffer = pRxBuffer;
			pSPIHandle->RxLen = len;

			//2. Mark the SPI state as busy in TX so no other code can take over the same SPI peripheral until TX is over
			pSPIHandle->RxState = SPI_BUSY_IN_RX;

			//3. Enable the TXIE control bit to get interrupt whenever TXE flag is set in SR
			pSPIHandle->pSPIx->SPI_CR2 |= ( 1 << SPI_CR2_RXNIE);

		}

		return state;


}//SPI_ReceiveDataIT

/**********************************************************************
 * @fn					- SPI_GetStatusFlag
 *
 * @brief				- Takes in a status flag and returns whether the flag is set or reset
 *
 * @param[in]			- Base address of SPI peripheral Ex: pSPIx = SPIA
 * @param[in]			- Status Register FlagName (defined in SPI.h)
 *
 * @return				- void
 *
 * @note				- Flag set and cleared by the hardware
 **********************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName )
{
	if(pSPIx->SPI_SR & flagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
} //SPI_GetStatusFlag


/**********************************************************************
 * Weak Implementations of Functions
 **********************************************************************/
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	//This is a weak implementation in the event the application does not implement this function.
}

