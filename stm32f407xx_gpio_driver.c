/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jul 17, 2020
 *      Author: AbMan
 */

#include "stm32f407xx_gpio_driver.h"

/*********************************************************************
 * @fn						-	GPIO_PeriClockControl
 *
 * @brief					-	This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]				-	base address of the gpio peripheral
 * @param[in]				-	ENABLE or DISABLE macros
 * @param[in]				-
 *
 * @return					-	none
 *
 * @Note					-	none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		}
	} else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();
		}
	}
}

/**************************************************************************************
 * @fn						-	GPIO_Init
 *
 * @brief					-	This function initializes the various pin mode of a GPIO port
 *
 * @param[in]				-	base address of the gpio peripheral
 * @param[in]				-	ENABLE or DISABLE macros
 * @param[in]				-
 *
 * @return					-	none
 *
 * @Note					-	none

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	uint32_t temp, temp1, temp2 = 0;	//temp. registers

	//enable the peripheral clock

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. Configuring the mode of the GPIO pin

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		//non-interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
				<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3
				<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting
	} else {
		//This part code later. (interrupt mode)
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT) {
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT) {
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure the GPIO port selection in the SYSCFG_EXTICR.
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp] &= ~(0xF << (4 * temp1));
		temp2 = (uint32_t) pGPIOHandle->pGPIOx;
		switch (temp2) {
		case (uint32_t) GPIOA:
			SYSCFG->EXTICR[temp] |= (0 << (4 * temp1));
			break;
		case (uint32_t) GPIOB:
			SYSCFG->EXTICR[temp] |= (1 << (4 * temp1));
			break;
		case (uint32_t) GPIOC:
			SYSCFG->EXTICR[temp] |= (2 << (4 * temp1));
			break;
		case (uint32_t) GPIOD:
			SYSCFG->EXTICR[temp] |= (3 << (4 * temp1));
			break;
		case (uint32_t) GPIOE:
			SYSCFG->EXTICR[temp] |= (4 << (4 * temp1));
			break;
		case (uint32_t) GPIOF:
			SYSCFG->EXTICR[temp] |= (5 << (4 * temp1));
			break;
		case (uint32_t) GPIOG:
			SYSCFG->EXTICR[temp] |= (6 << (4 * temp1));
			break;
		case (uint32_t) GPIOH:
			SYSCFG->EXTICR[temp] |= (7 << (4 * temp1));
			break;
		case (uint32_t) GPIOI:
			if (!(temp == 3)) {
				SYSCFG->EXTICR[temp] |= (8 << (4 * temp1));
			}
			break;
		}

		//3. enable the exti interrupt delivery using IMR.
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	//2. Configuring the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//3. Configuring the pupdr settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl
			<< (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//4. Configuring the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1
			<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//5. Configure the alt functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |=
				(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	} else if (pGPIOx == GPIOI) {
		GPIOI_REG_RESET();
	}
}

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 1);
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value;
	value = (uint16_t) (pGPIOx->IDR);
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t Value) {
	if (Value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
	pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (IRQNumber <= 31) {
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber < 64) {
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		} else if (IRQNumber < 96) {
			*NVIC_ISER3 |= (1 << IRQNumber % 64);
		}
	} else {
		if (IRQNumber <= 31) {
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber < 64) {
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		} else if (IRQNumber < 96) {
			*NVIC_ICER3 |= (1 << IRQNumber % 64);
		}
	}
}

void GPIO_IRQHandling(uint8_t PinNumber) {
	//1. clear the exti pr register.
	if (EXTI->PR & (1 << PinNumber)) {
		EXTI->PR |= (1 << PinNumber); //PR is cleared by writing a 1.
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	//1. first lets find the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}
