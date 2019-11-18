/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Oct 22, 2019
 *      Author: VKaiser
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include <stm32f407xx.h>

/*
 * Configuration Structure for a GPIO Pin, 0x04 Offset
 */
typedef struct
{
	uint8_t GPIO_PinNumber;					/*!< @GPIO_PIN_NUM 		- GPIO Pin Number [0..15]								>*/
	uint8_t GPIO_PinMode;					/*!< @GPIO_PIN_MODES 	- GPIO Pin Mode [Input/Output/Alt Function/Analog]		>*/
	uint8_t GPIO_PinSpeed;					/*!< @GPIO_PIN_SPEED 	- GPIO Output Speed Type	[Low/Med/High/Very High]	>*/
	uint8_t GPIO_PinPuPdCtrl;				/*!< @GPIO_PIN_PUPD 	- GPIO Pull Up/Pull Down Control						>*/
	uint8_t GPIO_PinOPType;					/*!< @GPIO_PIN_OP 		- GPIO Output Data Type									>*/
	uint8_t GPIO_PinAltFncMode;				/*!< @GPIO_PIN_ALTFN 	- GPIO Alternate Function Mode							>*/

}GPIO_PinConfig_t;

/*
 * Handle Structure for a GPIO Pin
 */
typedef struct
{
 	//Pointer to hold the base address of the GPIO peripheral - Use Example: pGPIOx = GPIOC;

	GPIO_RegDef_t *pGPIOx;					/*!< Holds the base address of the GPIO port to which the pin belongs 	>*/
	GPIO_PinConfig_t GPIO_PinConfig;		/*!< Holds the GPIO pin configuration settings 							>*/

}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUM
 * Possible GPIO pin Numbers
 */
#define GPIO_PIN_0					0		/*!< GPIO PIN 0				>*/
#define GPIO_PIN_1					1		/*!< GPIO PIN 1				>*/
#define GPIO_PIN_2					2		/*!< GPIO PIN 2				>*/
#define GPIO_PIN_3					3		/*!< GPIO PIN 3				>*/
#define GPIO_PIN_4					4		/*!< GPIO PIN 4				>*/
#define GPIO_PIN_5					5		/*!< GPIO PIN 5				>*/
#define GPIO_PIN_6					6		/*!< GPIO PIN 6				>*/
#define GPIO_PIN_7					7		/*!< GPIO PIN 7				>*/
#define GPIO_PIN_8					8		/*!< GPIO PIN 8				>*/
#define GPIO_PIN_9					9		/*!< GPIO PIN 9				>*/
#define GPIO_PIN_10					10		/*!< GPIO PIN 10			>*/
#define GPIO_PIN_11					11		/*!< GPIO PIN 11			>*/
#define GPIO_PIN_12					12		/*!< GPIO PIN 12			>*/
#define GPIO_PIN_13					13		/*!< GPIO PIN 13			>*/
#define GPIO_PIN_14					14		/*!< GPIO PIN 14			>*/
#define GPIO_PIN_15					15		/*!< GPIO PIN 15			>*/

/*
 * @GPIO_PIN_MODES
 * Possible GPIO pin modes
 */
#define GPIO_MODE_IN			0		/*!< INPUT (RESET STATE) 							>*/
#define GPIO_MODE_OUT			1		/*!< GENERAL PURPOSE OUTPUT MODE 					>*/
#define GPIO_MODE_ALTFN			2		/*!< ALTERNATE FUNCTION MODE 						>*/
#define	GPIO_MODE_ANALOG		3		/*!< ANALOG MODE 									>*/
#define	GPIO_MODE_IT_FT			4		/*!< INTERRUPT FALLING EDGE TRIGGER 				>*/
#define	GPIO_MODE_IT_RT			5		/*!< INTERRUPT RISING EDGETRIGGER					>*/
#define	GPIO_MODE_IT_RFT		6		/*!< INTERRUPT RISING EDGE/FALLING EDGE TRIGGER		>*/

/*
 * @GPIO_PIN_OP
 * Possible GPIO OUTPUT TYPES
 */
#define GPIO_OP_TYPE_PP			0		/*!< GPIO OUTPUT TYPE PUSH-PULL			>*/
#define GPIO_OP_TYPE_OD			1		/*!< GPIO OUTPUT TYPE OPEN DRAIN		>*/

/*
 * @GPIO_PIN_PUPD
 * Possible GPIO PULLUP AND PULLDOWN CONFIGURATION MACROS
 */
#define GPIO_NO_PUPD			0		/*!< GPIO FLOAT							>*/
#define GPIO_PIN_PU				1		/*!< GPIO PULL UP						>*/
#define GPIO_PIN_PD				2		/*!< GPIO PULL DOWN						>*/

/*
 * @GPIO_PIN_SPEED
 * Possible GPIO OUTPUT SPEEDS
 */
#define GPIO_SPEED_LOW			0		/*!< GPIO OUTPUT SPEED LOW				>*/
#define GPIO_SPEED_MEDIUM		1		/*!< GPIO OUTPUT SPEED MEDIUM			>*/
#define GPIO_SPEED_FAST			2		/*!< GPIO OUTPUT SPEED HIGH				>*/
#define GPIO_SPEED_HIGH			3		/*!< GPIO OUTPUT SPEED VERY HIGH		>*/

/*
 * @GPIO_PIN_ALTFN
 * Possible GPIO ALTERNATE FUNCTIONS
 */
#define AF0						0		/*!< GPIO ALTERNATE FUNCTION 0			>*/
#define AF1						1		/*!< GPIO ALTERNATE FUNCTION 1			>*/
#define AF2						2		/*!< GPIO ALTERNATE FUNCTION 2			>*/
#define AF3						3		/*!< GPIO ALTERNATE FUNCTION 3			>*/
#define AF4						4		/*!< GPIO ALTERNATE FUNCTION 4			>*/
#define AF5						5		/*!< GPIO ALTERNATE FUNCTION 5			>*/
#define AF6						6		/*!< GPIO ALTERNATE FUNCTION 6			>*/
#define AF7						7		/*!< GPIO ALTERNATE FUNCTION 7			>*/
#define AF8						8		/*!< GPIO ALTERNATE FUNCTION 8			>*/
#define AF9						9		/*!< GPIO ALTERNATE FUNCTION 9			>*/
#define AF10					10		/*!< GPIO ALTERNATE FUNCTION 10			>*/
#define AF11					11		/*!< GPIO ALTERNATE FUNCTION 11			>*/
#define AF12					12		/*!< GPIO ALTERNATE FUNCTION 12			>*/
#define AF13					13		/*!< GPIO ALTERNATE FUNCTION 13			>*/
#define AF14					14		/*!< GPIO ALTERNATE FUNCTION 14			>*/
#define AF15					15		/*!< GPIO ALTERNATE FUNCTION 15			>*/
/**********************************************************************************************************************
 *										APIs supported by this driver
 *						For more information about the APIs - see function definitions
 **********************************************************************************************************************/
/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);			/*!< Enable or Disable Peripheral Clock. Ex use: pGPIOx = GPIOA, EnorDi = ENABLE >*/

/*
 * Peripheral Initialization
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);					/*!< Initialize the registers of the given GPIO >*/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);					/*!< De-Initialize the registers of the given GPIO (Reset to default) >*/

/*
 * Data Read/Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);				/*!< Read digital output from GPIO Pin >*/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);									/*!< Read port of GPIO Port >*/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);	/*!< Read digital output from GPIO Pin >*/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);						/*!< Read digital output from GPIO Pin >*/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);					/*!< Read digital output from GPIO Pin >*/

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);						/*!< Configure Interrupt Enable/Disable >*/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);					/*!< Configure Interrupt Priority >*/
void GPIO_IRQHandling(uint8_t pinNumber);												/*!< Configure Interrupt Handling for a pin number >*/


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */

