/*
 * gpio.c
 *
 *  Created on: Apr 9, 2024
 *      Author: leeinfy
 */
#include "gpio.h"

static uint8_t get_gpio_index(GPIOx_RegDef_t* pGPIOx);

/* @fn 		GPIO_PeriClockControl
 * @brief	enable or disable peripheral clock for given GPIO peripheral
 * */
void GPIO_PeriClockControl(GPIOx_RegDef_t* pGPIOx, uint8_t enable){
	uint8_t index = get_gpio_index(pGPIOx);
	if(enable){
		GPIOx_PERI_CLK_EN(index);
	}else{
		GPIOx_PERI_CLK_DIS(index);
	}
}

/* @fn 		GPIO_Reset
 * @brief	reset the gpio peripheral
 * */
void GPIO_Peri_Reset(GPIOx_RegDef_t* pGPIOx){
	if (pGPIOx == GPIOA) GPIOx_RESET(0);
	else if(pGPIOx == GPIOB) GPIOx_RESET(1);
	else if(pGPIOx == GPIOC) GPIOx_RESET(2);
	else if(pGPIOx == GPIOD) GPIOx_RESET(3);
	else if(pGPIOx == GPIOE) GPIOx_RESET(4);
	else if(pGPIOx == GPIOF) GPIOx_RESET(5);
	else if(pGPIOx == GPIOG) GPIOx_RESET(6);
	else if(pGPIOx == GPIOH) GPIOx_RESET(7);
	else if(pGPIOx == GPIOI) GPIOx_RESET(8);
}

/* @fn 		GPIO_Init
 * @brief	init target gpio device
 * */
void GPIO_Init(gpio_dev_t* gpio_dev, gpio_port_config_t* port_config){
	GPIOx_RegDef_t* pGPIOx = gpio_dev -> gpio_peri;
	uint8_t port = gpio_dev -> pin;
	uint8_t port_mode =  port_config -> mode;
	if(port_mode < 4){
		//config the none interrupt mode
		pGPIOx -> MODER &= ~(0b11 << (2*port));
		pGPIOx -> MODER |= (port_mode << (2*port));\

	}else{
		//Interrupt always input mode
		pGPIOx -> MODER &= ~(0b11 << (2*port));
		pGPIOx -> MODER |= (GPIO_MODE_INPUT << (2*port));

		//edge detection
		if(port_mode == GPIO_MODE_IR_FE){
			EXTI -> FTSR |= (1 << port);
			EXTI -> RTSR &= ~(1 << port);
		}else if (port_mode == GPIO_MODE_IR_RE){
			EXTI -> RTSR |= (1 << port);
			EXTI -> FTSR &= ~(1 << port);
		}
		else if (port_mode == GPIO_MODE_IR_FRE){
			EXTI -> RTSR |= (1 << port);
			EXTI -> FTSR |= (1 << port);
		}

		//enable hardware interrupt mask
		EXTI -> IMR |= (1 << port);

		//turn on the EXTIx of target GPIOy_x, only one should be enabled if the port shard the same index
		SYSCFG_PERI_CLK_EN();
		uint8_t temp1 = port / 4;
		uint8_t temp2 = port % 4;
		uint8_t gpio_index =  get_gpio_index(pGPIOx);
		SYSCFG ->EXTICR[temp1] &= ~(0xff << temp2*4);
		SYSCFG ->EXTICR[temp1] |= (gpio_index << temp2*4);
	}

	//setup pull up or pull down register
	pGPIOx -> PUPDR &= ~(0b11 << (2*port));
	pGPIOx -> PUPDR |= (port_config->pull_up_down << (2*port));

	//setup output
	pGPIOx -> OTYPER &= ~(0b1 << port);
	pGPIOx -> OTYPER |= (port_config->output_type << port);

	//setup output speed
	pGPIOx -> OSPEEDR &= ~(0b11 << (2*port));
	pGPIOx -> OSPEEDR |= (port_config->output_speed << (2*port));

	//Alternate function mode select
	if(port_mode == GPIO_MODE_ALTER_FUNC){
		uint8_t tmp1 = port/8;
		uint8_t tmp2 = port%8;
		pGPIOx -> AFR[tmp1] &= ~(0b11 << (4*tmp2));
		pGPIOx -> AFR[tmp1] |= (port_config->pull_up_down << (4*tmp2));
	}
}

/* @fn 		GPIO_Get_Input
 * @brief	get the input of entire gpio peripheral
 * */
uint16_t GPIO_Get_Input(GPIOx_RegDef_t* pGPIOx){
	return (uint16_t) pGPIOx -> IDR;
}

/* @fn 		GPIO_Get_Port_Input
 * @brief	get the input of target port
 * */
uint8_t GPIO_Get_Port_Input(gpio_dev_t* gpio_dev){
	GPIOx_RegDef_t* pGPIOx = gpio_dev -> gpio_peri;
	uint8_t port = gpio_dev -> pin;
	return ((pGPIOx -> IDR) >> port) & 0x0001;
}

/* @fn 		GPIO_Update_Output
 * @brief	update the output value of entire gpio peripheral
 * @param 	value, bit mask of entire 16 port
 * */
void GPIO_Update_Output(gpio_dev_t* gpio_dev, uint16_t value){
	GPIOx_RegDef_t* pGPIOx = gpio_dev -> gpio_peri;
	pGPIOx -> ODR &= 0x0000;
	pGPIOx -> ODR |= value;
}

/* @fn 		GPIO_Update_Port_Output
 * @brief	update the value on a port, use atomic approach
 * */
void GPIO_Update_Port_Output(gpio_dev_t* gpio_dev, uint8_t value){
	GPIOx_RegDef_t* pGPIOx = gpio_dev -> gpio_peri;
	uint8_t port = gpio_dev -> pin;
	if (value == 0)	(pGPIOx -> BSRR) |= (1 << (port + 16));
	else if (value == 1) (pGPIOx -> BSRR) |= (1 << port);
}

/* @fn 		GPIO_toggle_Port_Output
 * @brief	toggle value on a gpio device, use atomic approach

 * */
void GPIO_toggle_Port_Output(gpio_dev_t* gpio_dev){
	GPIOx_RegDef_t* pGPIOx = gpio_dev -> gpio_peri;
	uint8_t port = gpio_dev -> pin;
	uint8_t value = (pGPIOx-> ODR >> port) & 0b0001;
	if (value == 0) (pGPIOx -> BSRR) |= (1 << port);
	else if (value == 1) (pGPIOx -> BSRR) |= (1 << (port + 16));
}

/* @fn 		get_gpio_index
 * @brief	get the gpio index base on address
 * @param	pGPIOx, base address of the GPIO peripheral
 * @return 	index of target gpio peripheral
 * */
uint8_t get_gpio_index(GPIOx_RegDef_t* pGPIOx){
	if (pGPIOx == GPIOA) return 0;
	else if(pGPIOx == GPIOB) return 1;
	else if(pGPIOx == GPIOC) return 2;
	else if(pGPIOx == GPIOD) return 3;
	else if(pGPIOx == GPIOE) return 4;
	else if(pGPIOx == GPIOF) return 5;
	else if(pGPIOx == GPIOG) return 6;
	else if(pGPIOx == GPIOH) return 7;
	else if(pGPIOx == GPIOI) return 8;
	else return 0;
}

/* @fn 		GPIO_IRQHandler
 * @brief	The IRQ handler of GPIO target pin, only 1 pin of different GPIO could be enable in 1 EXIT
 * 			This so only pin number is need for control target EXIT
 * @param	pinNumber, the pin of GPIO IRQ
 * */
void GPIO_IRQHandler(uint8_t PinNumber){
	//clear the EXTI pr register
	if(EXTI->PR & (1 << PinNumber)){
		EXTI->PR |= (1<<PinNumber);
	}
}
