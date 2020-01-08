//#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"

/**********************************************************************************************************************
 *										APIs supported by this driver
 *						For more information about the APIs - see function definitions
 **********************************************************************************************************************/

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
 * @param[in]			- Base address of GPIO peripheral Ex: pSPIx = SPIA
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
		SPI1_REG_RESET();		//ToDo: Define SPIx_Reg_Reset
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
 * @param[in]			- Base address of GPIO peripheral Ex: pSPIx = SPI1
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

}

/***************************************************************************************************
 * @fn					- SPI_SendData
 *
 * @brief				- Data structure Base Address, Data and length to TX Data
 *
 * @param[in]			- Base address of GPIO peripheral Ex: pSPIx = SPIA
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
 * @brief				- Configure Interrupt Handling for a pin number
 *
 * @param[in]			- pinNumber of IRQ
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- void
 *
 * @note				-
 **********************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{

}//SPI_IRQHandling

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
 * @note				-
 **********************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName )
{
	if(pSPIx->SPI_SR & flagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
} //SPI_GetStatusFlag
