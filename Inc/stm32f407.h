/*
 * stm32f407.h
 *
 *  Created on: Apr 9, 2024
 *      Author: leeinfy
 */

#ifndef DRIVERS_INC_STM32F407_H_
#define DRIVERS_INC_STM32F407_H_

#include <stdint.h>

//Arm Cortex Processor
#define ICTR_ADDR				0xE000E004UL	//Interrupt Controller Type Register, ICTR
//Interrupt Set-Enable Register
#define NVIC_ISER_BASE_ADDR			0xE000E100UL
#define NVIC_ISER(x)				(*(volatile uint32_t*) (NVIC_ISER_BASE_ADDR + 0x04*x))
//Interrupt Clear-Enable Register
#define NVIC_ICER_BASE_ADDR			0xE000E180UL
#define NVIC_ICER(x)				(*(volatile uint32_t*) (NVIC_ICER_BASE_ADDR + 0x04*x))
//Interrupt Clear-Enable Register
#define NVIC_IPR_BASE_ADDR			0xE000E400UL
#define NVIC_IPR(x)				(*(volatile uint32_t*) (NVIC_IPR_BASE_ADDR + 0x04*x))

/* base addresses of FLASH and SRAM memories*/
#define FLASH_BASE_ADDR			0x08000000UL
#define SRAM1_BASE_ADDR			0x20000000UL
#define SRAM2_BASE_ADDR			0x2001C000UL
#define ROM_BASE_ADDR			0x1FFF0000UL

/* peripheral bus addresses*/
#define PERIPHERALS_BASE_ADDR 	0x40000000UL
#define APB1_BASE_ADDR			0x40000000UL
#define APB2_BASE_ADDR			0x40010000UL
#define AHB1_BASE_ADDR			0x40020000UL
#define AHB2_BASE_ADDR			0x50000000UL
#define AHB3_BASE_ADDR			0x60000000UL

/* peripheral on AHB1 bus*/
#define USB_OTG_HS_BASE_ADDR		0x40040000UL
#define ETHERNET_MAC_BASE_ADDR		0x40028000UL
#define DMA2_BASE_ADDR				0x40026400UL
#define	DMA1_BASE_ADDR				0x40026000UL
#define BKPSRAM_BASE_ADDR			0x40024000UL
#define FLASH_INTERFACE_BASE_ADDR	0x40023C00UL
#define RCC_BASE_ADDR				0x40023800UL
#define CRC_BASE_ADDR				0x40023000UL
#define GPIOI_BASE_ADDR				0x40022000UL
#define GPIOH_BASE_ADDR				0x40021C00UL
#define GPIOG_BASE_ADDR				0x40021800UL
#define GPIOF_BASE_ADDR				0x40021400UL
#define GPIOE_BASE_ADDR				0x40021000UL
#define GPIOD_BASE_ADDR				0x40020C00UL
#define GPIOC_BASE_ADDR				0x40020800UL
#define GPIOB_BASE_ADDR				0x40020400UL
#define GPIOA_BASE_ADDR				0x40020000UL

/* peripheral on AHB2 bus*/
#define RNG_BASE_ADDR				0x50060800UL
#define DCMI_BASE_ADDR				0x50050000UL
#define USB_OTG_FS_BASE_ADDR		0x50000000UL

/* peripheral on AHB3 bus*/


/* peripheral on APB1 bus*/
#define TIM11_BASE_ADDR			0x40014800UL
#define TIM10_BASE_ADDR			0x40014400UL
#define TIM9_BASE_ADDR			0x40014000UL
#define EXTI_BASE_ADDR			0x40013C00UL
#define SYSCFG_BASE_ADDR		0x40013800UL
#define SPI1_BASE_ADDR			0x40013000UL
#define SDIO_BASE_ADDR			0x40012C00UL
#define ADC1_BASE_ADDR			0x40012000UL
#define ADC2_BASE_ADDR			ADC1_BASE_ADDR
#define ADC3_BASE_ADDR			ADC1_BASE_ADDR
#define USART6_BASE_ADDR		0x40011400UL
#define USART1_BASE_ADDR		0x40011000UL
#define TIM8_BASE_ADDR			0x40010400UL
#define TIM1_BASE_ADDR			0x40010000UL

/* peripheral on APB2 bus*/
#define DAC_BASE_ADDR			0x40007400UL
#define PWR_BASE_ADDR			0x40007000UL
#define CAN2_BASE_ADDR			0x40006800UL
#define CAN1_BASE_ADDR			0x40006400UL
#define I2C3_BASE_ADDR			0x40005C00UL
#define I2C2_BASE_ADDR			0x40005800UL
#define I2C1_BASE_ADDR			0x40005400UL
#define UART5_BASE_ADDR			0x40005000UL
#define UART4_BASE_ADDR			0x40004C00UL
#define USART3_BASE_ADDR		0x40004800UL
#define USART2_BASE_ADDR		0x40004400UL
#define I2S3EXT_BASE_ADDR		0x40004000UL
#define SPI3_BASE_ADDR			0x40003C00UL
#define I2S3_BASE_ADDR			SPI3_BASE_ADDR
#define SPI2_BASE_ADDR			0x40003800UL
#define I2S2_BASE_ADDR			SPI2_BASE_ADDR
#define I2S2EXT_BASE_ADDR		0x40003400UL
#define IWDG_BASE_ADDR			0x40003000UL
#define WWDG_BASE_ADDR			0x40002C00UL
#define RTC_BKP_BASE_ADDR		0x40002800UL
#define TIM14_BASE_ADDR			0x40002000UL
#define TIM13_BASE_ADDR			0x40001C00UL
#define TIM12_BASE_ADDR			0x40001800UL
#define TIM7_BASE_ADDR			0x40001400UL
#define TIM6_BASE_ADDR			0x40001000UL
#define TIM5_BASE_ADDR			0x40000C00UL
#define TIM4_BASE_ADDR			0x40000800UL
#define TIM3_BASE_ADDR			0x40000400UL
#define TIM2_BASE_ADDR			0x40000000UL

/***** RCC register map *****/
typedef struct{
	volatile uint32_t CR;			//clock control 			offset 0x00
	volatile uint32_t PLLCFGR;		//PLL configuration			offset 0x04
	volatile uint32_t CFGR;			//clock	configuration		offset 0x08
	volatile uint32_t CIR;			//clock interrupt			offset 0x0C
	volatile uint32_t AHB1RSTR;		//AHB1 peripheral reset		offset 0x10
	volatile uint32_t AHB2RSTR;		//AHB2 peripheral reset		offset 0x14
	volatile uint32_t AHB3RSTR;		//AHB3 peripheral reset		offset 0x18
	volatile uint32_t RESERVED1;		//							offset 0x1C
	volatile uint32_t APB1RSTR;		//AHB3 peripheral reset		offset 0x20
	volatile uint32_t APB2RSTR;		//AHB3 peripheral reset		offset 0x24
	volatile uint32_t RESERVED2[2];	//							offset 0x28,0x2C
	volatile uint32_t AHB1ENR;		//AHB1 peripheral clock enable		offset 0x30
	volatile uint32_t AHB2ENR;		//AHB2 peripheral clock enable		offset 0x34
	volatile uint32_t AHB3ENR;		//AHB3 peripheral clock enable		offset 0x38
	volatile uint32_t RESERVED3;		//									offset 0x3C
	volatile uint32_t APB1ENR;		//APB1 peripheral clock enable		offset 0x40
	volatile uint32_t APB2ENR;		//APB2 peripheral clock enable		offset 0x44
	volatile uint32_t RESERVED4[2];	//									offset 0x48,0x4C
	volatile uint32_t AHB1LPENR;		//AHB1 clock enable low power		offset 0x50
	volatile uint32_t AHB2LPENR;		//AHB2 clock enable low power		offset 0x54
	volatile uint32_t AHB3LPENR;		//AHB3 clock enable low power		offset 0x58
	volatile uint32_t RESERVED5;		//									offset 0x5C
	volatile uint32_t APB1LPENR;		//APB1 clock enable low power		offset 0x60
	volatile uint32_t APB2LPENR;		//APB2 clock enable low power		offset 0x64
	volatile uint32_t RESERVED6[2];	//									offset 0x68,0x6C
	volatile uint32_t BDCR;			//backup domain control				offset 0x70
	volatile uint32_t CSR;			//clock control & status register 	offset 0x74
	volatile uint32_t RESERVED7[2];	//									offset 0x78,0x7C
	volatile uint32_t SSCGR;			//spread spectrum clock generation	offset 0x80
	volatile uint32_t PLLI2SCFGR;	//PLLI2S configuration				offset 0x84
}RCC_RegDef_t;

#define RCC			((RCC_RegDef_t*) RCC_BASE_ADDR)

/***** GPIO Peripheral *****/
typedef struct{
	volatile uint32_t MODER;			//mode 						offset 0x00
	volatile uint32_t OTYPER;		//output type 				offset 0x04
	volatile uint32_t OSPEEDR;		//output speed				offset 0x08
	volatile uint32_t PUPDR;			//pull-up/pull-down			offset 0x0C
	volatile uint32_t IDR;			//input data				offset 0x10
	volatile uint32_t ODR;			//output data				offset 0x14
	volatile uint32_t BSRR;			//bit set/reset				offset 0x18
	volatile uint32_t LCKR;			//configuration lock 		offset 0x1C
	volatile uint32_t AFR[2];		//alternate function 		offset 0x20,0x24
}GPIOx_RegDef_t;

#define GPIOA		((GPIOx_RegDef_t*) GPIOA_BASE_ADDR)
#define GPIOB		((GPIOx_RegDef_t*) GPIOB_BASE_ADDR)
#define GPIOC		((GPIOx_RegDef_t*) GPIOC_BASE_ADDR)
#define GPIOD		((GPIOx_RegDef_t*) GPIOD_BASE_ADDR)
#define GPIOE		((GPIOx_RegDef_t*) GPIOE_BASE_ADDR)
#define GPIOF		((GPIOx_RegDef_t*) GPIOF_BASE_ADDR)
#define GPIOG		((GPIOx_RegDef_t*) GPIOG_BASE_ADDR)
#define GPIOH		((GPIOx_RegDef_t*) GPIOH_BASE_ADDR)
#define GPIOI		((GPIOx_RegDef_t*) GPIOI_BASE_ADDR)

/***** EXTI Peripheral****/
typedef struct{
	volatile uint32_t IMR;			//interrupt mask 			offset 0x00
	volatile uint32_t EMR;			//event mask				offset 0x04
	volatile uint32_t RTSR;			//rising trigger selection	offset 0x08
	volatile uint32_t FTSR;			//falling trigger selection	offset 0x0C
	volatile uint32_t SWIER;		//software interrupt event	offset 0x10
	volatile uint32_t PR;			//pending					offset 0x14
}EXTI_RegDef_t;

#define EXTI 		((EXTI_RegDef_t*) EXTI_BASE_ADDR)

/****** system configuration controller***********/
typedef struct{
	volatile uint32_t MEMRMP;		//memory remap						offset 0x00
	volatile uint32_t PMC;		  	//peripheral mode configuration		offset 0x04
	volatile uint32_t EXTICR[4];		//external interrupt configuration	offset 0x08,0x0C,0x10,0x14
	volatile uint32_t CMPCR;			//compensation cell control			offset 0x20
}SYSCFG_RegDef_t;

#define SYSCFG		((SYSCFG_RegDef_t*) SYSCFG_BASE_ADDR)

#endif /* DRIVERS_INC_STM32F407_H_ */
