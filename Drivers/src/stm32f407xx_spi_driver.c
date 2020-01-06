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

}//SPI_DeInit

/**********************************************************************
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
 * @note				-
 **********************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{

}//SPI_SendData

/**********************************************************************
 * @fn					- SPI_ReceiveData
 *
 * @brief				- Data structure Base Address, Data and length of Data to RX
 *
 * @param[in]			- Base address of GPIO peripheral Ex: pSPIx = SPIA
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

