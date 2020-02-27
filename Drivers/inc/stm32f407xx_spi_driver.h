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

/*
 * @SPI_DeviceMode	//SPI_CR1 MSTR
 */
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/*
 * @SPI_BusConfig	//SPI_CR1 BIDIMODE, BIDIOE, RXONLY
 */
#define SPI_BUS_CONFIG_FD						1		/* FULL DUPLEX 				*/
#define SPI_BUS_CONFIG_HD						2		/* HALF DUPLEX 				*/
//#define SPI_BUS_CONFIG_SIMPLEX_TX_ONLY		4		/* SIMPLEX TRANSMIT ONLY 	*/
#define SPI_BUS_CONFIG_SIMPLEX_RX_ONLY			3		/* SIMPLEX RECEIVE ONLY 	*/

/*
 * @SPI_SclkSpeed	//SPI_CR1 BaudRate [BR]
 * HSI Clock (System Clock) = 16MHz
 */
#define SPI_SCLK_SPEED_DIV2						0		/* fpclk/2		*/
#define SPI_SCLK_SPEED_DIV4						1		/* fpclk/4		*/
#define SPI_SCLK_SPEED_DIV8						2		/* fpclk/8		*/
#define SPI_SCLK_SPEED_DIV16					3		/* fpclk/16		*/
#define SPI_SCLK_SPEED_DIV32					4		/* fpclk/32		*/
#define SPI_SCLK_SPEED_DIV64					5		/* fpclk/64		*/
#define SPI_SCLK_SPEED_DIV128					6		/* fpclk/128	*/
#define SPI_SCLK_SPEED_DIV256					7		/* fpclk/256	*/

/*
 * @SPI_DFF	//SPI_CR1 Data Frame Format
 */
#define SPI_DFF_8BITS							0		/* DEFAULT: 8-bit data frame format for TX/RX 	*/
#define SPI_DFF_16BITS							1		/* 16-bit data frame format for TX/RX 			*/

/*
 * @SPI_CPOL	//SPI_CR1 CLOCK POLARITY
 */
#define	SPI_CPOL_HIGH							1		/* Clock Polarity High when idle	*/
#define	SPI_CPOL_LOW							0		/* Clock Polarity Low when idle	*/

/*
 * @SPI_CPHA	//SPI_CR1 Clock Phase
 */
#define	SPI_CPHA_HIGH							1		/* The second clock transition is the first data capture edge	*/
#define	SPI_CPHA_LOW							0		/* The first clock transition is the first data capture edge	*/

/*
 * @SPI_SSM		//SPI_CR1 SLAVE MANAGEMENT (SSM/SSI)
 */
#define SPI_SSM_DI								0		/* Software Slave Management Disabled (HW slave management)	*/
#define SPI_SSM_EN								1		/* Software Slave Management Enabled	*/

/*
 *SPI Related Status Flag Definitions
 */
#define SPI_BSY_FLAG							(1 << SPI_SR_BSY)		/* Masking info of SPI Busy Flag in SPI SR register 				*/
#define SPI_OVR_FLAG							(1 << SPI_SR_OVR)		/* Masking info of SPI Overrun Flag in SPI SR register 				*/
#define SPI_MODF_FLAG							(1 << SPI_SR_MODF)		/* Masking info of SPI Mode Fault Flag in SPI SR register 			*/
#define SPI_CRCERR_FLAG							(1 << SPI_SR_CRCERR)	/* Masking info of SPI CRC Error Flag in SPI SR register 			*/
#define SPI_UDR_FLAG							(1 << SPI_SR_UDR)		/* Masking info of SPI Underrun Flag in SPI SR register 			*/
#define SPI_CHSIDE_FLAG							(1 << SPI_SR_CHSIDE)	/* Masking info of SPI Channel Side Flag in SPI SR register 		*/
#define SPI_TXE_FLAG							(1 << SPI_SR_TXE)		/* Masking info of Transmit buffer empty Flag in SPI SR register 	*/
#define SPI_RXNE_FLAG							(1 << SPI_SR_RXNE)		/* Masking info of RX Buffer Not empty Flag in SPI SR register 		*/

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
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName );					/*!< Get status register flag state >*/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);					/*!< Enable or Disable SPI Peripheral >*/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);							/*!< Enable or Disable SPI Internal Slave Select  >*/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);							/*!< Enable or Disable SSOE Control bit for NSS (Slave Select Output Enable)  >*/
#endif /* SRC_STM32F407XX_SPI_DRIVER_H_ */
