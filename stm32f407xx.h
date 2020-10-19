/*
 * stm32f407xx.h
 *
 *  Created on: Jul 16, 2020
 *      Author: AbMan
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <string.h>
#include <stddef.h>

#define __vo volatile

/************************************START: Processor Specific Details*********************************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register addresses
 */
#define NVIC_ISER0				((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1				((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2				((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3				((__vo uint32_t*)0xE000E10c)

/*
 * ARM Cortex Mx Processor NVIC ICERx register addresses
 */
#define NVIC_ICER0				((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1				((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2				((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3				((__vo uint32_t*)0xE000E18c)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR		((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED 	4

/*
 * Base addresses of FLASH and SRAM memories.
 */
#define FLASH_BASEADDR			0x08000000U		/*Base address of FLASH module*/
#define SRAM1_BASEADDR			0x20000000U		/*Base address of SRAM1*/
#define SRAM2_BASEADDR			0x20001C00U		/*Base address of SRAM2 (16KB) after 112KB of SRAM1*/
#define ROM						0x1FFF0000U		/*Start of system memory onwards in FLASH module*/
#define SRAM					SRAM1_BASEADDR	/*SRAM module starts at SRAM1*/

/*
 * AHBx and APBx bus peripheral base addresses.
 */
#define PERIPH_BASEADDR			0x40000000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U
#define AHB3PERIPH_BASEADDR		0xA0000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U

/*
 * Base addresses of peripherals hanging on ABH1 bus.
 */
/*GPIO base addresses*/
#define GPIOA_BASEADDR			((AHB1PERIPH_BASEADDR) + (0x0000))
#define GPIOB_BASEADDR			((AHB1PERIPH_BASEADDR) + (0x0400))
#define GPIOC_BASEADDR			((AHB1PERIPH_BASEADDR) + (0x0800))
#define GPIOD_BASEADDR			((AHB1PERIPH_BASEADDR) + (0x0C00))
#define GPIOE_BASEADDR			((AHB1PERIPH_BASEADDR) + (0x1000))
#define GPIOF_BASEADDR			((AHB1PERIPH_BASEADDR) + (0x1400))
#define GPIOG_BASEADDR			((AHB1PERIPH_BASEADDR) + (0x1800))
#define GPIOH_BASEADDR			((AHB1PERIPH_BASEADDR) + (0x1C00))
#define GPIOI_BASEADDR			((AHB1PERIPH_BASEADDR) + (0x2000))
#define GPIOJ_BASEADDR			((AHB1PERIPH_BASEADDR) + (0x2400))
#define GPIOK_BASEADDR			((AHB1PERIPH_BASEADDR) + (0x2800))
/********************/
#define CRC_BASEADDR								((AHB1PERIPH_BASEADDR) + (0x3000))
#define RCC_BASEADDR								((AHB1PERIPH_BASEADDR) + (0x3800))
#define FLASH_INTERFACE_REGISTER_BASEADDR			((AHB1PERIPH_BASEADDR) + (0x3C00))
#define BKPSRAM_BASEADDR							((AHB1PERIPH_BASEADDR) + (0x4000))
#define DMA1_BASEADDR								((AHB1PERIPH_BASEADDR) + (0x6000))
#define DMA2_BASEADDR								((AHB1PERIPH_BASEADDR) + (0x6400))
#define ETHERNET_MAC_BASEADDR						((AHB1PERIPH_BASEADDR) + (0x8000))
#define DMA2D_BASEADDR								((AHB1PERIPH_BASEADDR) + (0xB000))
#define USB_OTG_HS_BASEADDR							((AHB1PERIPH_BASEADDR) + (0x00020000))

/*
 * Base addresses of peripherals hanging on AHB2 bus
 */
#define USB_OTG_FS_BASEADDR						((AHB2PERIPH_BASEADDR) + (0x0000))
#define DCMI_BASEADDR							((AHB2PERIPH_BASEADDR) + (0x00050000))
#define CRYP_BASEADDR							((AHB2PERIPH_BASEADDR) + (0x00060000))
#define HASH_BASEADDR							((AHB2PERIPH_BASEADDR) + (0x00060400))
#define RNG_BASEADDR							((AHB2PERIPH_BASEADDR) + (0x00060800))

/*
 * Base addresses of peripherals hanging on AHB3 bus
 */
#define FSMC_CR_BASEADDR						((AHB3PERIPH_BASEADDR) + (0x0000))

/*
 * Base addresses of peripherals hanging on APB1 bus
 */
#define I2S2ext_BASEADDR		((APB1PERIPH_BASEADDR) + (0x3400))
#define SPI2_BASEADDR			((APB1PERIPH_BASEADDR) + (0x3800))
#define SPI3_BASEADDR			((APB1PERIPH_BASEADDR) + (0x3C00))
#define I2S3ext_BASEADDR		((APB1PERIPH_BASEADDR) + (0x4000))
#define USART2_BASEADDR			((APB1PERIPH_BASEADDR) + (0x4400))
#define USART3_BASEADDR			((APB1PERIPH_BASEADDR) + (0x4800))
#define UART4_BASEADDR			((APB1PERIPH_BASEADDR) + (0x4C00))
#define UART5_BASEADDR			((APB1PERIPH_BASEADDR) + (0x5000))
#define I2C1_BASEADDR			((APB1PERIPH_BASEADDR) + (0x5400))
#define I2C2_BASEADDR			((APB1PERIPH_BASEADDR) + (0x5800))
#define I2C3_BASEADDR			((APB1PERIPH_BASEADDR) + (0x5C00))

/*
 * Base addresses of peripherals hanging on APB2 bus
 */
#define SPI1_BASEADDR			((APB2PERIPH_BASEADDR) + (0x3000))
#define USART1_BASEADDR			((APB2PERIPH_BASEADDR) + (0x1000))
#define USART6_BASEADDR			((APB2PERIPH_BASEADDR) + (0x1400))
#define EXTI_BASEADDR			((APB2PERIPH_BASEADDR) + (0x3C00))
#define SYSCFG_BASEADDR			((APB2PERIPH_BASEADDR) + (0x3800))

/************************************Peripheral Register Definition Structures*********************************************/

typedef struct {
	__vo uint32_t MODER; /*GPIO Port Mode Register										Address offset: 0x00	  */
	__vo uint32_t OTYPER; /*GPIO Port Output Type Register								Address offset: 0x04	  */
	__vo uint32_t OSPEEDR; /*GPIO Port Output Speed Register								Address offset: 0x08	  */
	__vo uint32_t PUPDR; /*GPIO Port Pull-Up/Pull-Down Register							Address offset: 0x0C	  */
	__vo uint32_t IDR; /*GPIO Port Input Data Register									Address offset: 0x10	  */
	__vo uint32_t ODR; /*GPIO Port Output Data Register								Address offset: 0x14	  */
	__vo uint16_t BSRRL; /*GPIO Port Bit Set/Reset Low Register							Address offset: 0x18	  */
	__vo uint16_t BSRRH; /*GPIO Port Bit Set/Reset High Register							Address offset: 0x1A	  */
	__vo uint32_t LCKR; /*GPIO Port Configuration Lock Register							Address offset: 0x1C	  */
	__vo uint32_t AFR[2]; /*GPIO Port Alternate Function Low/High Register				Address offset: 0x20-0x24 */
} GPIO_RegDef_t;

typedef struct {
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
} RCC_RegDef_t;

typedef struct {
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
} EXTI_RegDef_t;

typedef struct {
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED[2];
	__vo uint32_t CMPCR;
} SYSCFG_RegDef_t;

typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;

typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
}I2C_RegDef_t;

typedef struct {
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
}USART_RegDef_t;

/*
 * Peripheral Definitions
 */
#define GPIOA		((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI		((GPIO_RegDef_t*) GPIOI_BASEADDR)

#define RCC			((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI		((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG		((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define SPI1		((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*) SPI3_BASEADDR)

#define I2C1		((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3		((I2C_RegDef_t*) I2C3_BASEADDR)

#define USART1		((USART_RegDef_t) USART1_BASEADDR)
#define USART2		((USART_RegDef_t) USART2_BASEADDR)
#define USART3		((USART_RegDef_t) USART3_BASEADDR)
#define UART4		((USART_RegDef_t) UART4_BASEADDR)
#define UART5		((USART_RegDef_t) UART5_BASEADDR)
#define USART6		((USART_RegDef_t) USART6_BASEADDR)
/*
 * Clock Enable Macros for GPIO Peripherals
 */
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1<<8))

/*
 * Clock Enable Macros for I2C Peripheral
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1<<23))

/*
 * Clock Enable Macros for SPI Peripheral
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1<<15))

/*
 * Clock Enable Macros for USART Peripheral
 */
#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()		(RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()			(RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()			(RCC->APB1ENR |= (1<<20))
#define USART6_PCLK_EN()		(RCC->APB2ENR |= (1<<5))

/*
 * Clock Enable Macros for SYSCFG Peripheral
 */
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1<<14))

/*
 * Clock Disable Macros for GPIO Peripherals
 */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<8))

/*
 * Clock Disable Macros for I2C Peripheral
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<23))

/*
 * Clock Disable Macros for SPI Peripheral
 */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<15))

/*
 * Clock Disable Macros for USART Peripheral
 */
#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()			(RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()			(RCC->APB1ENR &= ~(1<<20))
#define USART6_PCLK_DI()		(RCC->APB2ENR &= ~(1<<5))

/*
 * Clock Disable Macros for SYSCFG Peripheral
 */
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1<<14))

/*
 * GPIO reset macros
 */
#define GPIOA_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<0)) ; (RCC->AHB1RSTR &= ~(1<<0));} while(0)
#define GPIOB_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<1)) ; (RCC->AHB1RSTR &= ~(1<<1));} while(0)
#define GPIOC_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<2)) ; (RCC->AHB1RSTR &= ~(1<<2));} while(0)
#define GPIOD_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<3)) ; (RCC->AHB1RSTR &= ~(1<<3));} while(0)
#define GPIOE_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<4)) ; (RCC->AHB1RSTR &= ~(1<<4));} while(0)
#define GPIOF_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<5)) ; (RCC->AHB1RSTR &= ~(1<<5));} while(0)
#define GPIOG_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<6)) ; (RCC->AHB1RSTR &= ~(1<<6));} while(0)
#define GPIOH_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<7)) ; (RCC->AHB1RSTR &= ~(1<<7));} while(0)
#define GPIOI_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<8)) ; (RCC->AHB1RSTR &= ~(1<<8));} while(0)

#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI5_10			40

/*
 * Generic Macros
 */
#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET 		SET
#define GPIO_PIN_RESET 		RESET
#define FLAG_RESET 			RESET
#define FLAG_SET 			SET
#define READ				1
#define WRITE				0

/*******************************************************************************************************
 * Bit position definitions of SPI peripheral
 *******************************************************************************************************/
/*
 * Bit positions of CR1 register
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSB_FIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RX_ONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRC_NEXT	12
#define SPI_CR1_CRC_EN		13
#define SPI_CR1_BIDI_OE		14
#define SPI_CR1_BIDI_MODE	15

/*
 * Bit positions of CR2 register
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/*
 * Bit positions of SR register
 */
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRC_ERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8

/*
 * SPI reset macros
 */
#define SPI1_PERI_RESET()		do{(RCC->APB2RSTR |= (1<<12)) ; (RCC->APB2RSTR &= ~(1<<12));} while(0)
#define SPI2_PERI_RESET()		do{(RCC->APB1RSTR |= (1<<14)) ; (RCC->APB1RSTR &= ~(1<<14));} while(0)
#define SPI3_PERI_RESET()		do{(RCC->APB1RSTR |= (1<<15)) ; (RCC->APB1RSTR &= ~(1<<15));} while(0)

/*
 * SPI IRQ numbers
 */

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51

/*******************************************************************************************************
 * Bit position definitions of I2C peripheral
 *******************************************************************************************************/
/*
 * I2C reset macros
 */
#define I2C1_PERI_RESET()		do{(RCC->APB1RSTR |= (1<<21)) ; (RCC->APB1RSTR &= ~(1<<21));} while(0)
#define I2C2_PERI_RESET()		do{(RCC->APB1RSTR |= (1<<22)) ; (RCC->APB1RSTR &= ~(1<<22));} while(0)
#define I2C3_PERI_RESET()		do{(RCC->APB1RSTR |= (1<<23)) ; (RCC->APB1RSTR &= ~(1<<23));} while(0)

/*
 * CR1 bit position definitions
 */
#define I2C_CR1_PE					0
#define I2C_CR1_NOSTRETCH			7
#define I2C_CR1_START				8
#define I2C_CR1_STOP				9
#define I2C_CR1_ACK					10
#define I2C_CR1_SWRST				15

/*
 * CR2 bit position definitions
 */
#define I2C_CR2_FREQ				0
#define I2C_CR2_ITERREN				8
#define I2C_CR2_ITEVTEN				9
#define I2C_CR2_ITBUFEN				10

/*
 * OAR1 bit position definitions
 */
#define I2C_OAR1_ADD0				0
#define I2C_OAR1_ADD71				1
#define I2C_OAR1_ADD98				8
#define I2C_OAR1_ADDMODE			15

/*
 * SR1 bit position definitions
 */
#define I2C_SR1_SB					0
#define I2C_SR1_ADDR				1
#define I2C_SR1_BTF					2
#define I2C_SR1_ADD10				3
#define I2C_SR1_STOPF				4
#define I2C_SR1_RXNE				6
#define I2C_SR1_TXE					7
#define I2C_SR1_BERR				8
#define I2C_SR1_ARLO				9
#define I2C_SR1_AF					10
#define I2C_SR1_OVR					11
#define I2C_SR1_TIMEOUT				14

/*
 * SR2 bit position definitions
 */
#define I2C_SR2_MSL					0
#define I2C_SR2_BUSY				1
#define I2C_SR2_TRA					2
#define I2C_SR2_GENCALL				4
#define I2C_SR2_DUALF				7

/*
 * CCR bit position definitions
 */
#define I2C_CCR_CCR					0
#define I2C_CCR_DUTY				14
#define I2C_CCR_FS					15

/*
 * I2C IRQ Numbers
 */
#define IRQ_NO_I2C1_EV				31
#define IRQ_NO_I2C1_ER				32
#define IRQ_NO_I2C2_EV				33
#define IRQ_NO_I2C2_ER				34
#define IRQ_NO_I2C3_EV				72
#define IRQ_NO_I2C3_ER				73

/*******************************************************************************************************
 * Bit position definitions of USART peripheral
 *******************************************************************************************************/
/*
 * USART reset macros
 */
#define USART1_PERI_RESET()		do{(RCC->APB2RSTR |= (1<<4)) ; (RCC->APB1RSTR &= ~(1<<4));} while(0)
#define USART2_PERI_RESET()		do{(RCC->APB1RSTR |= (1<<17)) ; (RCC->APB1RSTR &= ~(1<<17));} while(0)
#define USART3_PERI_RESET()		do{(RCC->APB1RSTR |= (1<<18)) ; (RCC->APB1RSTR &= ~(1<<18));} while(0)
#define UART4_PERI_RESET()		do{(RCC->APB1RSTR |= (1<<19)) ; (RCC->APB1RSTR &= ~(1<<19));} while(0)
#define UART5_PERI_RESET()		do{(RCC->APB1RSTR |= (1<<20)) ; (RCC->APB1RSTR &= ~(1<<20));} while(0)
#define USART6_PERI_RESET()		do{(RCC->APB1RSTR |= (1<<5)) ; (RCC->APB1RSTR &= ~(1<<5));} while(0)

/*
 * SR bit position macros
 */
#define USART_SR_PE				0
#define USART_SR_FE				1
#define USART_SR_NF				2
#define USART_SR_ORE			3
#define USART_SR_IDLE			4
#define USART_SR_RXNE			5
#define USART_SR_TC				6
#define USART_SR_TXE			7
#define USART_SR_LBD			8
#define USART_SR_CTS			9

/*
 * CR1 bit position macros
 */
#define USART_CR1_SBK			0
#define USART_CR1_RWU			1
#define USART_CR1_RE			2
#define USART_CR1_TE			3
#define USART_CR1_IDLEIE		4
#define USART_CR1_RXNEIE		5
#define USART_CR1_TCIE			6
#define USART_CR1_TXEIE			7
#define USART_CR1_PEIE			8
#define USART_CR1_PS			9
#define USART_CR1_PCE			10
#define USART_CR1_WAKE			11
#define USART_CR1_M				12
#define USART_CR1_UE			13
#define USART_CR1_OVER8			15

/*
 * CR2 bit position macros
 */
#define USART_CR2_LBDL			5
#define USART_CR2_LBDIE			6
#define USART_CR2_LBCL			8
#define USART_CR2_CPHA			9
#define USART_CR2_CPOL			10
#define USART_CR2_STOP			12
#define USART_CR2_CLKEN			11
#define USART_CR2_LINEN			14

/*
 * CR3 bit position macros
 */
#define USART_CR3_EIE			0
#define USART_CR3_IREN			1
#define USART_CR3_IRLP			2
#define USART_CR3_HDSEL			3
#define USART_CR3_NACK			4
#define USART_CR3_SCEN			5
#define USART_CR3_DMAR			6
#define USART_CR3_DMAT			7
#define USART_CR3_RTSE			8
#define USART_CR3_CTSE			9
#define USART_CR3_CTSIE			10
#define USART_CR3_ONEBIT		11

/*
 * USART IRQ Numbers
 */
#define IRQ_NO_USART1			37
#define IRQ_NO_USART1			38
#define IRQ_NO_USART1			39
#define IRQ_NO_UART4			52
#define IRQ_NO_UART5			53
#define IRQ_NO_USART6			71

#endif /* INC_STM32F407XX_H_ */

