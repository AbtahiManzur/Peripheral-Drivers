/*
 * stm32f07xx_spi_driver.c
 *
 *  Created on: Jul 24, 2020
 *      Author: AbMan
 */
#include "stm32f407xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t* pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t* pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t* pSPIHandle);

/*
 * Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//peripheral clock enable

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//first configure the SPI_CR1 register

	uint32_t tempreg = 0;

	//1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. configure the bus configuration
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//clear bidi mode
		tempreg &= ~(1 << SPI_CR1_BIDI_MODE);

	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//set bidi mode
		tempreg |= (1 << SPI_CR1_BIDI_MODE);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//clear bidi mode
		tempreg &= ~(1 << SPI_CR1_BIDI_MODE);
		//set rx only mode
		tempreg |= (1 << SPI_CR1_RX_ONLY);
	}

	//3. configure spi serial clock
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;
	pSPIHandle->pSPIx->CR1 = tempreg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if (pSPIx == SPI1)
	{
		SPI1_PERI_RESET();

	} else if (pSPIx == SPI2)
	{
		SPI2_PERI_RESET();
	} else if (pSPIx == SPI3)
	{
		SPI3_PERI_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/**************************************************************************************
 * @fn						-	SPI_SendData
 *
 * @brief					-	This function sends the data
 *
 * @param[in]				-
 * @param[in]				-
 * @param[in]				-
 *
 * @return					-	none
 *
 * @Note					-	This is a blocking call. It does not return unless all data is sent.

 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		//2. check the DFF BIT
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bit DFF
			//1. load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET);

		//2. check the DFF BIT
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bit DFF
			//1. load the data from Rx buffer into DR.
			 *((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else
		{
			//8 bit DFF
			 *pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	return state;
}


/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		*NVIC_ISER1 |= (1 << IRQNumber % 32);
	} else
	{
		*NVIC_ICER1 |= (1 << IRQNumber % 32);
	}
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;
	//first check for check TXE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//check for RXNE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	//check for overrun
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		//handle RXNE
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}
}

/*
 * Other peripheral control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

static void spi_txe_interrupt_handle(SPI_Handle_t* pSPIHandle)
{
	if ((pSPIHandle->pSPIx->CR1) & (1 << SPI_CR1_DFF))
	{
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(!(pSPIHandle->TxLen))
	{
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t* pSPIHandle)
{
	if ((pSPIHandle->pSPIx->CR1) & (1 << SPI_CR1_DFF))
	{
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pRxBuffer);
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}else
	{
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pRxBuffer);
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(!(pSPIHandle->RxLen))
	{
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t* pSPIHandle)
{
	uint8_t temp;
	//1. clear the ovr flag
	if(!(pSPIHandle->TxState == SPI_BUSY_IN_TX))
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void) temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR; //refer to user manual on how to clear ovr
	temp = pSPIx->SR;
	(void)temp;
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	//TxLen is over, close the spi communication an let application know its over
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	//RxLen is over, close the spi communication an let application know its over
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	//weak implementation
}

