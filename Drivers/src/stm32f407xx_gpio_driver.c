/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Oct 22, 2019
 *      Author: VKaiser
 */

#include "stm32f407xx_gpio_driver.h"

/**********************************************************************************************************************
 *										APIs supported by this driver
 *						For more information about the APIs - see function definitions
 **********************************************************************************************************************/

/**********************************************************************
 * @fn					- GPIO_PeriClockCtrl
 *
 * @brief				- Enable or Disable Peripheral Clock
 *
 * @param[in]			- Base address of GPIO peripheral Ex: pGPIOx = GPIOA
 * @param[in]			- ENABLE or DISABLE macro (MCU header file)
 *
 * @return				- void
 *
 * @note				- none
 **********************************************************************/
void GPIO_PeriClockCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	//can't switch case on a pointer.
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOB)
		{
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOC)
		{
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	//ELSE DISABLE
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOB)
		{
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOC)
		{
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		} else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
} //GPIO_PeriClockCtrl

/**********************************************************************
 * @fn					- GPIO_Init
 *
 * @brief				- Initialize the registers of the given GPIO
 *
 * @param[in]			- Base address of Handle Structure for a GPIO Pin (pGPIOx and GPIO_PinConfig)
 * @param[in]			-
 *
 * @return				- void
 *
 * @note				- none
 **********************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;		//temp. register

	/* Configure the Mode Settings of the GPIO Pin */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non-interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	// Shift the MODE by 2 x pinNumber
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);						// Clear MODER
		pGPIOHandle->pGPIOx->MODER |= temp;			// Or the temp value for those bits into the GPIO MODER register,
	}
	else
	{
		//ToDo: Configure the interrupt mode of the GPIO Pin
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//Rising Edge Interrupt: Configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );		// Set FTSR
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );		// Clear RTSR
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//Falling Edge Interrupt: Configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );		// Set RTSR
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );		// Clear FTSR
		}
		else
		{
			//Rising or Falling Edge Interrupt: Configure FTSR and RTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );		// Set RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );		// Set FTSR
		}
	}
	temp = 0;									// clear temp

	EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );				// Enable interrupt delivery on EXTI line (based on pin number)


	uint8_t temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) / 4;			// Determine required EXTICR register
	uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 4;			// Determine group position in EXTICR register

	SYSCFG_PCLK_EN();															// Enable peripheral clock (SYSCFG)

	uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);				// Determine Interrupt Port
	SYSCFG->EXTICR[temp1] |= ( portCode << ( temp2 * 4 ) );						// Enable selected Interrupt Control

	temp1 = temp2 = 0;									// clear temps

	/* Configure the Speed Settings of the GPIO Pin */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));		// Shift the SPEED by 2 x pinNumber
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed);						// Clear OSPEEDR
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;									// clear temp

	/* Configure the Output Type Setting of the GPIO Pin */

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));		// Shift the OUTPUT TYPE by 1 x pinNumber
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType);						// Clear OTYPER
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;									// clear temp

	/* Configure the Pullup/Pulldown Settings of the GPIO Pin */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdCtrl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	// Shift the PUPD by 2 x pinNumber
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdCtrl);						// Clear PUPDR
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;									// clear temp

	/* IF Alt Fnc: Configure the Alternate Functionality Setting of the GPIO Pin */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;		//temp. registers
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;						// Determine AFR register.  0 = low, 1 = high
		temp2 =  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;					// Determine register position

		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFncMode << (4 * temp2);		// Shift the ALT FUNCTION MODE by 4 respective to register, pin number
		pGPIOHandle->pGPIOx->AFR[temp] &= ~(0xF << (4 * temp2));					// Clear AFR
		pGPIOHandle->pGPIOx->AFR[temp] |= temp2;									// Write to appropriate register

		temp = temp1 = temp2 = 0;				//clear temp
	}

}//GPIO_Init

/**********************************************************************
 * @fn					- GPIO_DeInit
 *
 * @brief				- De-Initialize the registers of the given GPIO (Reset to default)
 *
 * @param[in]			- Base address of GPIO peripheral Ex: pGPIOx = GPIOA
 * @param[in]			-
 *
 * @return				- void
 *
 * @note				-
 **********************************************************************/

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
		if(pGPIOx == GPIOA)
		{
			GPIOA_REG_RESET();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_REG_RESET();
		} else if (pGPIOx == GPIOB)
		{
			GPIOC_REG_RESET();
		} else if (pGPIOx == GPIOC)
		{
			GPIOD_REG_RESET();
		} else if (pGPIOx == GPIOD)
		{
			GPIOD_REG_RESET();
		} else if (pGPIOx == GPIOE)
		{
			GPIOE_REG_RESET();
		} else if (pGPIOx == GPIOF)
		{
			GPIOF_REG_RESET();
		} else if (pGPIOx == GPIOG)
		{
			GPIOG_REG_RESET();
		} else if (pGPIOx == GPIOH)
		{
			GPIOH_REG_RESET();
		} else if (pGPIOx == GPIOI)
		{
			GPIOI_REG_RESET();
		}

}//GPIO_DeInit


/**********************************************************************
 * @fn					- GPIO_ReadFromInputPin
 *
 * @brief				- Read digital output from GPIO Pin
 *
 * @param[in]			- Base address of GPIO peripheral Ex: pGPIOx = GPIOA
 * @param[in]			- Pin number of selected GPIO
 * @param[in]			-
 *
 * @return				- 0 or 1
 *
 * @note				-
 **********************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> pinNumber) & 0x1);
	return value;
}//GPIO_ReadFromInputPin


/**********************************************************************
 * @fn					- GPIO_ReadFromInputPort
 *
 * @brief				- Read port of GPIO Port
 *
 * @param[in]			- Base address of GPIO peripheral Ex: pGPIOx = GPIOA
 * @param[in]			-
 *
 * @return				- 16 bit value
 *
 * @note				-
 **********************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}//GPIO_ReadFromInputPort

/**********************************************************************
 * @fn					- GPIO_WriteToOutputPin
 *
 * @brief				- Read digital output from GPIO Pin
 *
 * @param[in]			- Base address of GPIO peripheral Ex: pGPIOx = GPIOA
 * @param[in]			- Pin number of selected GPIO
 * @param[in]			- Value to write - 1 or 0
 *
 * @return				- void
 *
 * @note				-
 **********************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		//write 1 to the ODR at the bit field corresponding to the pin value
		pGPIOx->ODR &= ~(1 << pinNumber);		// Clear the bit field
		pGPIOx->ODR |= (1 << pinNumber);		// Write the SET bit
	}
	else
	{
		//write 0
		pGPIOx->ODR &= ~(1 << pinNumber);		// Write the RESET bit
	}


}//GPIO_WriteToOutputPin

/**********************************************************************
 * @fn					- GPIO_WriteToOutputPort
 *
 * @brief				- Read digital output from GPIO Pin
 *
 * @param[in]			- Base address of GPIO peripheral Ex: pGPIOx = GPIOA
 * @param[in]			- 16 bit value to write
 * @param[in]			-
 *
 * @return				- void
 *
 * @note				-
 **********************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR &= ~(0xFF);		// Clear the register
	pGPIOx->ODR = value;		// Write Value to the ODR

}//GPIO_WriteToOutputPort

/**********************************************************************
 * @fn					- GPIO_ToggleOutputPin
 *
 * @brief				- Read digital output from GPIO Pin
 *
 * @param[in]			- Base address of GPIO peripheral Ex: pGPIOx = GPIOA
 * @param[in]			- Pin number of selected GPIO
 * @param[in]			-
 *
 * @return				- void
 *
 * @note				-
 **********************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	pGPIOx->ODR = pGPIOx->ODR ^ (1 << pinNumber);

}//GPIO_ToggleOutputPin

/**********************************************************************
 * @fn					- GPIO_IRQConfig
 *
 * @brief				- Configure Interrupt
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- void
 *
 * @note				-
 **********************************************************************/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
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

}//GPIO_IRQConfig

/**********************************************************************
 * @fn					- GPIO_IRQHandling
 *
 * @brief				- Configure Interrupt Handling for a pin number
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- void
 *
 * @note				-
 **********************************************************************/
void GPIO_IRQHandling(uint8_t pinNumber)
{

}//GPIO_IRQHandling
