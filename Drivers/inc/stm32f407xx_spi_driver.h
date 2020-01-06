/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Dec 10, 2019
 *      Author: VKaiser
 */

#ifndef SRC_STM32F407XX_SPI_DRIVER_H_
#define SRC_STM32F407XX_SPI_DRIVER_H_

#include <stm32f407xx.h>

/*
 * Configuration Structure for a SPIx Peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;					/*!< @SPI_DeviceMode	- SPI Mode: Master or Slave											>*/
	uint8_t SPI_BusConfig;					/*!< @SPI_BusConfig 	- SPI Mode Config: Full Duplex, Half Duplex, Simplex				>*/
	uint8_t SPI_SclkSpeed;					/*!< @SPI_SclkSpeed 	- SPI Serial Clock Speed											>*/
	uint8_t SPI_DFF;						/*!< @SPI_DFF 			- SPI Data Format: 8 bit or 16 bit data xfer						>*/
	uint8_t SPI_CPOL;						/*!< @SPI_CPOL 			- Idle state of the clock when no data xfered (default = 0 (LOW))	>*/
	uint8_t SPI_CPHA;						/*!< @SPI_CPHA 			- Data sampled on clock edge 1st/2nd (default = 0 / 1) [w/ CPOL]	>*/
	uint8_t SPI_SSM;						/*!< @SPI_SSM 			- Software Slave Management: Hardware (EN) or Software control		>*/

}SPI_Config_t;

/*
 * Handle Structure for a SPIx Peripheral
 */
typedef struct
{
	SPI_RegDef_t 	*pSPIx;					/*!< @SPI_RegDef_t	- Holds the base address of SPIx(x:0, 1, 2) peripheral					>*/
	SPI_Config_t	SPIConfig;				/*!< @SPI_Config_t 	- SPI Mode Config struct												>*/

}SPI_Handle_t;


/**********************************************************************************************************************
 *										APIs supported by this driver
 *						For more information about the APIs - see function definitions
 **********************************************************************************************************************/
/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);	/*!< Enable or Disable Peripheral Clock. Ex use: pSPIx = SPI1, EnorDi = ENABLE >*/

/*
 * Peripheral Initialization/De-Initialization
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);						/*!< Initialize the registers of the given SPI >*/
void SPI_DeInit(SPI_RegDef_t *pSPIx);							/*!< De-Initialize the registers of the given SPI (Reset to default) >*/

/*
 * Peripheral Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);			/*!< Base Address, Pointer to Data, size of data (std uint32_t) >*/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);		/*!< Base Address, Pointer to Data storage, size of data >*/

/*
 * Peripheral IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);						/*!< Configure Interrupt Enable/Disable >*/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);				/*!< Configure Interrupt Priority >*/
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);										/*!< Configure Interrupt Handling for a pin number >*/

/*
 * Other Peripheral Control APIs
 */




#endif /* SRC_STM32F407XX_SPI_DRIVER_H_ */
