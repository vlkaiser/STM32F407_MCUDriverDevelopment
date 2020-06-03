/*
 * stm32f407xx.h
 *
 *  Created on: Oct 17, 2019
 *      Author: VKaiser
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

/*
 * Common Definitions / Macros
 */
#define __vo		volatile

#define ENABLE 			1
#define DISABLE 		0
#define HIGH			ENABLE
#define LOW				DISABLE
#define	SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET

/************************ START: Processor Specific Details **********************************/
 /*
 * ARM Cortex M4 Processor NVIC ISERx Register Addresses
 */
#define NVIC_ISER0					( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1					( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2					( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3					( (__vo uint32_t*)0xE000E10C )
#define NVIC_ISER4					( (__vo uint32_t*)0xE000E110 )
#define NVIC_ISER5					( (__vo uint32_t*)0xE000E114 )
#define NVIC_ISER6					( (__vo uint32_t*)0xE000E118 )
#define NVIC_ISER7					( (__vo uint32_t*)0xE000E11C )

 /*
 * ARM Cortex M4 Processor NVIC ICERx Register Addresses
 */
#define NVIC_ICER0					( (__vo uint32_t*)0xE000E180 )
#define NVIC_ICER1					( (__vo uint32_t*)0xE000E184 )
#define NVIC_ICER2					( (__vo uint32_t*)0xE000E188 )
#define NVIC_ICER3					( (__vo uint32_t*)0xE000E18C )
#define NVIC_ICER4					( (__vo uint32_t*)0xE000E190 )
#define NVIC_ICER5					( (__vo uint32_t*)0xE000E194 )
#define NVIC_ICER6					( (__vo uint32_t*)0xE000E198 )
#define NVIC_ICER7					( (__vo uint32_t*)0xE000E19C )

 /*
 * ARM Cortex M4 Processor NVIC IPR Register Addresses
 */
#define NVIC_IPR_BASE_ADDR			( (__vo uint32_t*)0xE000E400 )

#define NO_PR_BITS_IMPLEMENTED		4

 /*
 * ARM Cortex M4 Processor NVIC Priority Macros
 */
#define NVIC_IRQ_PRI_0				0
#define NVIC_IRQ_PRI_1				1
#define NVIC_IRQ_PRI_2				2
#define NVIC_IRQ_PRI_3				3
#define NVIC_IRQ_PRI_4				4
#define NVIC_IRQ_PRI_5				5
#define NVIC_IRQ_PRI_6				6
#define NVIC_IRQ_PRI_7				7
#define NVIC_IRQ_PRI_8				8
#define NVIC_IRQ_PRI_9				9
#define NVIC_IRQ_PRI_10				10
#define NVIC_IRQ_PRI_11				11
#define NVIC_IRQ_PRI_12				12
#define NVIC_IRQ_PRI_13				13
#define NVIC_IRQ_PRI_14				14
#define NVIC_IRQ_PRI_15				15


/************************ TODO: PERIPHERAL INTERRUPTS **********************************/

/*
 * IRQ (Interrupt Request) Numbers for STM32F407x MCU
 */
//GPIO
#define IRQ_EXTI0		6
#define IRQ_EXTI1		7
#define IRQ_EXTI2		8
#define IRQ_EXTI3		9
#define IRQ_EXTI4		10
#define IRQ_EXTI9_5		23
#define IRQ_EXTI5_10	40
//SPI
#define IRQ_SPI1		35
#define IRQ_SPI2		36
#define IRQ_SPI3		51


/************************ END: Processor Specific Details **********************************/


/************************ TODO: PERIPHERAL BASE ADDRESSES **********************************/
/*
 * Base addresses of Flash and SRAM memories
 * U means unsigned integer. Addresses are never signed.
 */

#define FLASH_BASEADDR				0x08000000U		/* FLASH Base Address: KB */
#define SRAM1_BASEADDR				0x20000000U		/* Main Memory SRAM1 Base Address: 112KB */
#define SRAM2_BASEADDR				0x2001C000U		/* Auxiliary Internal Memory SRAM2 Base Address: 16KB */
#define ROM_BASEADDR				0x1FFF0000U		/* ROM: System Memory Base Address: 30KB*/
#define OTP_BASEADDR				0x1FFF7800U		/* One Time Programmable Memory Base Address: 528Bytes */

#define SRAM 						SRAM1_BASEADDR


/*
 * AHBx and APBx Peripheral Base Base Addresses
 * Bus Domain
 */

#define PERIPH_BASE					0x40000000U		/* Peripheral Base Address (AHB1) */
#define APB1PERIPH_BASE				PERIPH_BASE		/* APB1 Base Address */
#define APB2PERIPH_BASE				0x40010000U		/* APB2 Base Address */
#define AHB1PERIPH_BASE				0x40020000U		/* AHB1 Base Address */
#define AHB2PERIPH_BASE				0x50000000U		/* AHB2 Base Address */

/*
 * RCC Peripheral Base Address
 */


/*
 * AHB1 Peripheral Base Addresses
 * All desired peripherals hanging on AHB1 Bus - GPIO
 */
#define GPIOA_BASEADDR				(AHB1PERIPH_BASE + 0x0000U)		/* GPIOA Base Address */
#define GPIOB_BASEADDR				(AHB1PERIPH_BASE + 0x0400U)		/* GPIOB Base Address */
#define GPIOC_BASEADDR				(AHB1PERIPH_BASE + 0x0800U)		/* GPIOC Base Address */
#define GPIOD_BASEADDR				(AHB1PERIPH_BASE + 0x0C00U)		/* GPIOD Base Address */
#define GPIOE_BASEADDR				(AHB1PERIPH_BASE + 0x1000U)		/* GPIOE Base Address */
#define GPIOF_BASEADDR				(AHB1PERIPH_BASE + 0x1400U)		/* GPIOF Base Address */
#define GPIOG_BASEADDR				(AHB1PERIPH_BASE + 0x1800U)		/* GPIOG Base Address */
#define GPIOH_BASEADDR				(AHB1PERIPH_BASE + 0x1C00U)		/* GPIOH Base Address */
#define GPIOI_BASEADDR				(AHB1PERIPH_BASE + 0x2000U)		/* GPIOI Base Address */
#define RCC_BASEADDR				(AHB1PERIPH_BASE + 0x3800U)		/* RCC Base Address */

/*
 * APB1 Peripheral Base Addresses
 * All peripherals hanging on APB1 Bus
 * UART does not produce synchronous communication.  USART can work in sync/async mode.
 */
#define SPI2_BASEADDR				(APB1PERIPH_BASE + 0x3800U)		/* SPI2 Base Address */
#define SPI3_BASEADDR				(APB1PERIPH_BASE + 0x3C00U)		/* SPI3 Base Address */

#define USART2_BASEADDR				(APB1PERIPH_BASE + 0x4400U)		/* USART2 Base Address */
#define USART3_BASEADDR				(APB1PERIPH_BASE + 0x4800U)		/* USART3 Base Address */
#define UART4_BASEADDR				(APB1PERIPH_BASE + 0x4C00U)		/* UART4 Base Address */
#define UART5_BASEADDR				(APB1PERIPH_BASE + 0x5000U)		/* UART5 Base Address */

#define I2C1_BASEADDR				(APB1PERIPH_BASE + 0x5400U)		/* I2C1 Base Address */
#define I2C2_BASEADDR				(APB1PERIPH_BASE + 0x5800U)		/* I2C2 Base Address */
#define I2C3_BASEADDR				(APB1PERIPH_BASE + 0x5C00U)		/* I2C3 Base Address */

/*
 * APB2 Peripheral Base Addresses
 * All peripherals hanging on APB2 Bus
 */
#define USART1_BASEADDR				(APB2PERIPH_BASE + 0x1000U)		/* USART1 Base Address */
#define USART6_BASEADDR				(APB2PERIPH_BASE + 0x1400U)		/* USART6 Base Address */

#define SPI1_BASEADDR				(APB2PERIPH_BASE + 0x3000U)		/* SPI1 Base Address */
#define SPI4_BASEADDR				(APB2PERIPH_BASE + 0x3400U)		/* SPI4 Base Address */

#define SYSCFG_BASEADDR				(APB2PERIPH_BASE + 0x3800U)		/* SYSCFG Base Address */
#define EXTI_BASEADDR				(APB2PERIPH_BASE + 0x3C00U)		/* EXTI Base Address */


/************************ TODO: PERIPHERAL REGISTER DEFINITIONS **********************************/

/*
 * Peripheral Register Definition Structure for RCC
 * Example: RCC_RegDef_t *pRCC = RCC;
 */
typedef struct
{
	__vo uint32_t CR;			/*	Clock control register										Address Offset: 0x00	*/
	__vo uint32_t PLLCFGR;		/*	PLL configuration register									Address Offset: 0x04	*/
	__vo uint32_t CFGR;			/*	Clock configuration register								Address Offset: 0x08	*/
	__vo uint32_t CIR;			/*	Clock interrupt register									Address Offset: 0x0C	*/
	__vo uint32_t AHB1RSTR;		/*	AHB1 peripheral reset register								Address Offset: 0x10	*/
	__vo uint32_t AHB2RSTR;		/*	AHB2 peripheral reset register								Address Offset: 0x14	*/
	__vo uint32_t AHB3RSTR;		/*	AHB3 peripheral reset register								Address Offset: 0x18	*/
	__vo uint32_t RESERVED1;	/*  RESERVED 0x1C																		*/
	__vo uint32_t APB1RSTR;		/*	APB1 peripheral reset register								Address Offset: 0x20	*/
	__vo uint32_t APB2RSTR;		/*	APB2 peripheral reset register								Address Offset: 0x24	*/
	uint32_t RESERVED2[2];			/*  RESERVED 0x28-0X2C																*/
	__vo uint32_t AHB1ENR;		/*	AHB1 peripheral clock enable register						Address Offset: 0x30	*/
	__vo uint32_t AHB2ENR;		/*	AHB2 peripheral clock enable register						Address Offset: 0x34	*/
	__vo uint32_t AHB3ENR;		/*	AHB3 peripheral clock enable register						Address Offset: 0x38	*/
	uint32_t RESERVED3;				/*  RESERVED 0x3C																	*/
	__vo uint32_t APB1ENR;		/*	APB1 peripheral clock enable register						Address Offset: 0x40	*/
	__vo uint32_t APB2ENR;		/*	APB2 peripheral clock enable register						Address Offset: 0x44	*/
	uint32_t RESERVED4[2];			/*  RESERVED 0x48-0X4C																*/
	__vo uint32_t AHB1LPENR;	/*	AHB1 peripheral clock enable in low power mode register		Address Offset: 0x50	*/
	__vo uint32_t AHB2LPENR;	/*	AHB2 peripheral clock enable in low power mode register		Address Offset: 0x54	*/
	__vo uint32_t AHB3LPENR;	/*	AHB3 peripheral clock enable in low power mode register		Address Offset: 0x58	*/
	uint32_t RESERVED5;				/*  RESERVED 0x5C	 																*/
	__vo uint32_t APB1LPENR;	/*	APB1 peripheral clock enable in low power mode register		Address Offset: 0x60	*/
	__vo uint32_t APB2LPENR;	/*	APB2 peripheral clock enable in low power mode register		Address Offset: 0x64	*/
	__vo uint32_t AHP3LPENR;	/*	APB3 peripheral clock enable in low power mode register		Address Offset: 0x68	*/
	uint32_t RESERVED6;				/*  RESERVED 0x6C																	*/
	__vo uint32_t BDCR;			/*	RCC Backup domain control register							Address Offset: 0x70	*/
	__vo uint32_t CSR;			/*	RCC clock control & status register							Address Offset: 0x74	*/
	uint32_t RESERVED7[2];			/*	RESERVED 0x78-0X7C																*/
	__vo uint32_t SSCGR;		/*	RCC spread spectrum clock generation register				Address Offset: 0x80	*/
	__vo uint32_t PLLI2SCFGR;	/*	RCC PLLI2S configuration register							Address Offset: 0x84	*/
} RCC_RegDef_t;

/*
 * Peripheral Register Definition Structure for SYSCFG
 * Example: SYSCFG_RegDef_t *pSYSCFG = (SYSCFG_RegDef_t*)SYSCFG_BASEADDR;
 */
typedef struct
{
	__vo uint32_t MEMRMP;		/*	SYSCFG Memory Remap Register							Address Offset: 0x00		*/
	__vo uint32_t PMC;			/*	SYSCFG Peripheral Mode Config Register					Address Offset: 0x04		*/
	__vo uint32_t EXTICR[4];	/*	SYSCFG External Interrupt Config 1..4 Registers			Address Offset: 0x08..0x14	*/
	uint32_t RESERVED2[2];		/*  RESERVED 0x18-0x1C																	*/
	__vo uint32_t CMPCR;		/*	Compensation Cell Control Register						Address Offset: 0x20		*/

} SYSCFG_RegDef_t;


/*
 * Peripheral Register Definition Structure for EXTI (Interrupts)
 * Example: EXTI_RegDef_t *pEXTI1 = (EXTI_RegDef_t*)EXTI1_BASEADDR;
 */
typedef struct
{
	__vo uint32_t IMR;		/*	EXTI Interrupt Mask Register					Address Offset: 0x00	*/
	__vo uint32_t EMR;		/*	EXTI Event Mask Register						Address Offset: 0x04	*/
	__vo uint32_t RTSR;		/*	EXTI Rising Trigger Selection Register			Address Offset: 0x08	*/
	__vo uint32_t FTSR;		/*	EXTI Falling Trigger Selection Register			Address Offset: 0x0C	*/
	__vo uint32_t SWIER;	/*	EXTI Software Interrupt Event  Register			Address Offset: 0x10	*/
	__vo uint32_t PR;		/*	EXTI Pending Register							Address Offset: 0x14	*/

} EXTI_RegDef_t;

/*
 * Peripheral Register Definition Structure for GPIO
 * Example: GPIO_RegDef_t *pGPIOA = GPIOA;
 */
typedef struct
{
	__vo uint32_t MODER;	/*	Port Mode Register									Address Offset: 0x00	*/
	__vo uint32_t OTYPER;	/*	Output Type Register								Address Offset: 0x04	*/
	__vo uint32_t OSPEEDR;	/*	Output Speed  Register								Address Offset: 0x08	*/
	__vo uint32_t PUPDR;	/*	Pull-up/Pull-down  Register							Address Offset: 0x0C	*/
	__vo uint32_t IDR;		/*	Input Data  Register								Address Offset: 0x10	*/
	__vo uint32_t ODR;		/*	Output Data  Register								Address Offset: 0x14	*/
	__vo uint32_t BSRR;		/*	Bit Set/Reset  Register Low							Address Offset: 0x18	*/
	__vo uint32_t LCKR;		/*	Port Config Lock  Register							Address Offset: 0x1C	*/
	__vo uint32_t AFR[2];	/*	Alternate Function Register [0] LOW, [1] HIGH]		Address Offset: 0x20	*/
} GPIO_RegDef_t;

/*
 * Peripheral Register Definition Structure for SPI
 * Example: SPI_RegDef_t *pSPI = SPI;
 */
typedef struct
{
	__vo uint32_t SPI_CR1;		/*	SPI Control Register 1						Address Offset: 0x00	*/
	__vo uint32_t SPI_CR2;		/*	SPI Control Register 2						Address Offset: 0x04	*/
	__vo uint32_t SPI_SR;		/*	SPI Status Register							Address Offset: 0x08	*/
	__vo uint32_t SPI_DR;		/*	SPI Data Register							Address Offset: 0x0C	*/
	__vo uint32_t SPI_CRCPR;	/*	SPI CRC Polynomial Register					Address Offset: 0x10	*/
	__vo uint32_t SPI_RXCRCR;	/*	SPI RX CRC Register							Address Offset: 0x14	*/
	__vo uint32_t SPI_TXCRCR;	/*	SPI TX CRC Register							Address Offset: 0x18	*/
	__vo uint32_t SPI_I2SCFGR;	/*	SPI I2S Configuration Register				Address Offset: 0x1C	*/
	__vo uint32_t SPI_I2SPR;	/*	SPI I2S Prescaler Register					Address Offset: 0x20	*/
} SPI_RegDef_t;

/*
 * Peripheral Register Definition Structure for I2C
 * Example: I2C_RegDef_t *pI2C1 = (I2C_RegDef_t*)I2C1_BASEADDR;
 */
typedef struct
{
	__vo uint32_t CR1;		/*	I2C Control register 1					Address Offset: 0x00	*/
	__vo uint32_t CR2;		/*	I2C Control register 2					Address Offset: 0x04	*/
	__vo uint32_t OAR1;		/*	I2C Own address register 1				Address Offset: 0x08	*/
	__vo uint32_t OAR2;		/*	I2C Own address register 2				Address Offset: 0x0C	*/
	__vo uint32_t DR;		/*	I2C Data register						Address Offset: 0x10	*/
	__vo uint32_t SR1;		/*	I2C Status register 1					Address Offset: 0x14	*/
	__vo uint32_t SR2;		/*	I2C Status register 2					Address Offset: 0x18	*/
	__vo uint32_t TRISE;	/*	I2C T-Rise register						Address Offset: 0x1C	*/
	__vo uint32_t FLTR;		/*	I2C Filter register						Address Offset: 0x20	*/
} I2C_RegDef_t;

/*
 * Peripheral Register Definition Structure for USART
 * Example: USART_RegDef_t *pUSART1 = (USART_RegDef_t*)USART1_BASEADDR;
 */
typedef struct
{
	__vo uint32_t SR;		/*	USART Status register						Address Offset: 0x00	*/
	__vo uint32_t DR;		/*	USART Data register							Address Offset: 0x04	*/
	__vo uint32_t BRR;	/*	USART Baud rate register					Address Offset: 0x08	*/
	__vo uint32_t CR1;	/*	USART Control register 1					Address Offset: 0x0C	*/
	__vo uint32_t CR2;	/*	USART Control register 2					Address Offset: 0x10	*/
	__vo uint32_t CR3;	/*	USART Control register 3					Address Offset: 0x14	*/
	__vo uint32_t GTPR;	/*	USART Guard time and pre-scaler register		Address Offset: 0x18	*/
} USART_RegDef_t;

/************************ TODO: PERIPHERAL DEFINITIONS **********************************/

/*
 * Peripheral Definitions (Peripheral Base Addresses Typecast to xxx_RegDef_t)
 */
#define GPIOA	((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD	((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE	((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF	((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG	((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH	((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI	((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC		((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI	((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG	((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1	((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2	((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3	((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4	((SPI_RegDef_t*)SPI4_BASEADDR)

/************************ TODO: PERIPHERAL CLOCK ENABLE/DISABLE DEFINITIONS **********************************/
/*
 * Clock Enable Macros for GPIOx Peripherals
 */
#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 0))			/* GPIOA PERIPHERAL CLOCK ENABLE		*/
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 1))			/* GPIOB PERIPHERAL CLOCK ENABLE		*/
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 2))			/* GPIOC PERIPHERAL CLOCK ENABLE		*/
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 3))			/* GPIOD PERIPHERAL CLOCK ENABLE		*/
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 4))			/* GPIOE PERIPHERAL CLOCK ENABLE		*/
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 5))			/* GPIOF PERIPHERAL CLOCK ENABLE		*/
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 6))			/* GPIOG PERIPHERAL CLOCK ENABLE		*/
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 7))			/* GPIOH PERIPHERAL CLOCK ENABLE		*/
#define GPIOI_PCLK_EN()	(RCC->AHB1ENR |= ( 1 << 8))			/* GPIOI PERIPHERAL CLOCK ENABLE		*/

/*
 * Clock Enable Macros for I2Cx Peripherals
 */
#define I2C1_PCLK_EN()	(RCC->APB1ENR |= ( 1 << 21))		/* I2C1 PERIPHERAL CLOCK ENABLE			*/
#define I2C2_PCLK_EN()	(RCC->APB1ENR |= ( 1 << 22))		/* I2C2 PERIPHERAL CLOCK ENABLE			*/
#define I2C3_PCLK_EN()	(RCC->APB1ENR |= ( 1 << 23))		/* I2C3 PERIPHERAL CLOCK ENABLE			*/

/*
 * Clock Enable Macros for SPIx Peripherals
 */
#define SPI1_PCLK_EN()	(RCC->APB2ENR |= ( 1 << 12))		/* SPI1 PERIPHERAL CLOCK ENABLE			*/
#define SPI2_PCLK_EN()	(RCC->APB1ENR |= ( 1 << 14))		/* SPI2 PERIPHERAL CLOCK ENABLE			*/
#define SPI3_PCLK_EN()	(RCC->APB1ENR |= ( 1 << 15))		/* SPI3 PERIPHERAL CLOCK ENABLE			*/
#define SPI4_PCLK_EN()	(RCC->APB2ENR |= ( 1 << 13))		/* SPI4 PERIPHERAL CLOCK ENABLE			*/

/*
 * Clock Enable Macros for USARTx Peripherals
 */
#define USART1_PCLK_EN()	(RCC->APB2ENR |= ( 1 << 4))				/* USART1 PERIPHERAL CLOCK ENABLE		*/
#define UART2_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 17))			/* UART2 PERIPHERAL CLOCK ENABLE		*/
#define UART3_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 18))			/* UART3 PERIPHERAL CLOCK ENABLE		*/
#define UART4_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 19))			/* UART4 PERIPHERAL CLOCK ENABLE		*/
#define UART5_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 20))			/* UART5 PERIPHERAL CLOCK ENABLE		*/
#define USART6_PCLK_EN()	(RCC->APB2ENR |= ( 1 << 5))				/* USART6 PERIPHERAL CLOCK ENABLE		*/

/*
 * Clock Enable Macros for SYSCFG Peripherals
 */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= ( 1 << 14))			/* SYSCFG PERIPHERAL CLOCK ENABLE		*/


/*
 * Clock Disable Macros for GPIO Peripherals
 */
#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 0))		/* GPIOA PERIPHERAL CLOCK DISABLE			*/
#define GPIOB_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 1))		/* GPIOB PERIPHERAL CLOCK DISABLE			*/
#define GPIOC_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 2))		/* GPIOC PERIPHERAL CLOCK DISABLE			*/
#define GPIOD_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 3))		/* GPIOD PERIPHERAL CLOCK DISABLE			*/
#define GPIOE_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 4))		/* GPIOE PERIPHERAL CLOCK DISABLE			*/
#define GPIOF_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 5))		/* GPIOF PERIPHERAL CLOCK DISABLE			*/
#define GPIOG_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 6))		/* GPIOG PERIPHERAL CLOCK DISABLE			*/
#define GPIOH_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 7))		/* GPIOH PERIPHERAL CLOCK DISABLE			*/
#define GPIOI_PCLK_DI()	(RCC->AHB1ENR &= ~( 1 << 8))		/* GPIOI PERIPHERAL CLOCK DISABLE			*/

/*
 * Clock Disable Macros for I2Cx Peripherals
 */
#define I2C1_PCLK_DI()	(RCC->APB1ENR &= ~( 1 << 21))		/* I2C1 PERIPHERAL CLOCK DISABLE			*/
#define I2C2_PCLK_DI()	(RCC->APB1ENR &= ~( 1 << 22))		/* I2C2 PERIPHERAL CLOCK DISABLE			*/
#define I2C3_PCLK_DI()	(RCC->APB1ENR &= ~( 1 << 23))		/* I2C3 PERIPHERAL CLOCK DISABLE			*/

/*
 * Clock Disable Macros for SPIx Peripherals
 */
#define SPI1_PCLK_DI()	(RCC->APB2ENR &= ~( 1 << 12))		/* SPI1 PERIPHERAL CLOCK DISABLE			*/
#define SPI2_PCLK_DI()	(RCC->APB1ENR &= ~( 1 << 14))		/* SPI2 PERIPHERAL CLOCK DISABLE			*/
#define SPI3_PCLK_DI()	(RCC->APB1ENR &= ~( 1 << 15))		/* SPI3 PERIPHERAL CLOCK DISABLE			*/
#define SPI4_PCLK_DI()	(RCC->APB2ENR &= ~( 1 << 13))		/* SPI4 PERIPHERAL CLOCK DISABLE			*/

/*
 * Clock Disable Macros for USARTx Peripherals
 */
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~( 1 << 4))			/* USART1 PERIPHERAL CLOCK DISABLE		*/
#define UART2_PCLK_DI()		(RCC->APB1ENR &= ~( 1 << 17))			/* UART2 PERIPHERAL CLOCK DISABLE		*/
#define UART3_PCLK_DI()		(RCC->APB1ENR &= ~( 1 << 18))			/* UART3 PERIPHERAL CLOCK DISABLE		*/
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~( 1 << 19))			/* UART4 PERIPHERAL CLOCK DISABLE		*/
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~( 1 << 20))			/* UART5 PERIPHERAL CLOCK DISABLE		*/
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~( 1 << 5))			/* USART6 PERIPHERAL CLOCK DISABLE		*/

/*
 * Clock Disable Macros for SYSCFG Peripherals
 */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~( 1 << 14))			/* SYSCFG PERIPHERAL CLOCK DISABLE		*/

/*
 * Reset GPIOx Peripheral Macros
 */
#define GPIOA_REG_RESET()		do{(RCC->AHB1RSTR |= ( 1 << 0)); (RCC->AHB1RSTR &= ~( 1 << 0));} while(0)
#define GPIOB_REG_RESET()		do{(RCC->AHB1RSTR |= ( 1 << 1)); (RCC->AHB1RSTR &= ~( 1 << 1));} while(0)
#define GPIOC_REG_RESET()		do{(RCC->AHB1RSTR |= ( 1 << 2)); (RCC->AHB1RSTR &= ~( 1 << 2));} while(0)
#define GPIOD_REG_RESET()		do{(RCC->AHB1RSTR |= ( 1 << 3)); (RCC->AHB1RSTR &= ~( 1 << 3));} while(0)
#define GPIOE_REG_RESET()		do{(RCC->AHB1RSTR |= ( 1 << 4)); (RCC->AHB1RSTR &= ~( 1 << 4));} while(0)
#define GPIOF_REG_RESET()		do{(RCC->AHB1RSTR |= ( 1 << 5)); (RCC->AHB1RSTR &= ~( 1 << 5));} while(0)
#define GPIOG_REG_RESET()		do{(RCC->AHB1RSTR |= ( 1 << 6)); (RCC->AHB1RSTR &= ~( 1 << 6));} while(0)
#define GPIOH_REG_RESET()		do{(RCC->AHB1RSTR |= ( 1 << 7)); (RCC->AHB1RSTR &= ~( 1 << 7));} while(0)
#define GPIOI_REG_RESET()		do{(RCC->AHB1RSTR |= ( 1 << 8)); (RCC->AHB1RSTR &= ~( 1 << 8));} while(0)

/*
 *  GPIOx Port-to-Port Code Macro
 */
#define GPIO_BASEADDR_TO_CODE(x)  ( (x == GPIOA) ? 0 : \
									(x == GPIOB) ? 1 : \
									(x == GPIOC) ? 2 : \
									(x == GPIOD) ? 3 : \
									(x == GPIOE) ? 4 : \
									(x == GPIOF) ? 5 : \
									(x == GPIOG) ? 6 : \
									(x == GPIOH) ? 7 : 0 )

/*
 * Reset SPIx Peripheral Macros
 */
#define SPI1_REG_RESET()		do{(RCC->APB2RSTR |= ( 1 << 12)); (RCC->AHB2RSTR &= ~( 1 << 12));} while(0)
#define SPI2_REG_RESET()		do{(RCC->APB1RSTR |= ( 1 << 14)); (RCC->AHB1RSTR &= ~( 1 << 14));} while(0)
#define SPI3_REG_RESET()		do{(RCC->APB1RSTR |= ( 1 << 15)); (RCC->AHB1RSTR &= ~( 1 << 15));} while(0)
#define SPI4_REG_RESET()		do{(RCC->APB2RSTR |= ( 1 << 13)); (RCC->AHB2RSTR &= ~( 1 << 13));} while(0)

/************************ TODO: PERIPHERAL BIT POSITION DEFINITIONS **********************************/
/**********************************************
 * Bit Position Definitions for SPI Peripheral
 **********************************************/
#define SPI_CR1_BIDIMODE	15
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_CRCEN		13
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_DFF			11
#define SPI_CR1_RXONLY		10
#define SPI_CR1_SSM			9
#define SPI_CR1_SSI			8
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SPE			6
#define SPI_CR1_BR			3
#define SPI_CR1_MSTR		2
#define SPI_CR1_CPOL		1
#define SPI_CR1_CPHA		0

#define SPI_CR2_TXEIE		7
#define SPI_CR2_RXNIE		6
#define SPI_CR2_ERRIE		5
#define SPI_CR2_FRF			4
#define SPI_CR2_SSOE		2
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_RXDMAEN		0

#define SPI_SR_FRE			8
#define SPI_SR_BSY			7
#define SPI_SR_OVR			6
#define SPI_SR_MODF			5
#define SPI_SR_CRCERR		4
#define SPI_SR_UDR			3
#define SPI_SR_CHSIDE		2
#define SPI_SR_TXE			1
#define SPI_SR_RXNE			0


/************************ TODO: INCLUDE PERIPHERAL HEADER FILES **********************************/
/*
 * Peripheral Header Files
 */
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

#endif /* INC_STM32F407XX_H_ */

































