/*
 * gpio.h
 *
 *  Created on: Apr 9, 2024
 *      Author: leeinfy
 */

#ifndef DRIVERS_INC_GPIO_H_
#define DRIVERS_INC_GPIO_H_

#include "stm32f407.h"
#include "syscfg.h"
#include "irq.h"
#include <stdint.h>

//GPIO Peripheral clock enable
#define GPIOx_PERI_CLK_EN(index)		(RCC->AHB1ENR |= (1 << index))

//GPIO Peripheral clock disable
#define GPIOx_PERI_CLK_DIS(index)		(RCC->AHB1ENR &= ~(1 << index))

//GPIO Peripheral Reset
#define GPIOx_RESET(index)		do{RCC->AHB1RSTR |= (1 << index); RCC->AHB1RSTR &= ~(1 << index);}while(0)

//gpio mode
#define GPIO_MODE_INPUT				0x00U
#define GPIO_MODE_GP_OUTPUT			0x01U
#define GPIO_MODE_ALTER_FUNC		0x02U
#define GPIO_MODE_ANALOG			0x03U
#define GPIO_MODE_IR_FE				0x04U				//interrupt mode falling edge detection
#define GPIO_MODE_IR_RE				0x05U				//interrupt mode rising edge detection
#define GPIO_MODE_IR_FRE			0x06U				//interrupt mode rising and falling edge detection

//gpio output type
#define GPIO_OUTPUT_PUSH_PULL		0b0U
#define GPIO_OUTPUT_OPEN_DRAIN		0b1U

//gpio output speed
#define GPIO_OUTPUT_LOW_SPEED			0b00U
#define GPIO_OUTPUT_MED_SPEED			0b01U
#define GPIO_OUTPUT_HIGH_SPEED			0b10U
#define GPIO_OUTPUT_VERY_HIGH_SPEED		0b11U

//GPIO pull up or pull down
#define GPIO_NO_PUPD			0b00U
#define GPIO_PULL_UP			0b01U
#define GPIO_PULL_DOWN 			0b10U

//gpio dev
typedef struct{
	GPIOx_RegDef_t* gpio_peri;
	uint8_t pin;
}gpio_dev_t;

//gpio port config
typedef struct{
	uint8_t mode;
	uint8_t output_type;
	uint8_t output_speed;
	uint8_t pull_up_down;
	uint8_t alt_func;
}gpio_port_config_t;

void GPIO_PeriClockControl(GPIOx_RegDef_t* pGPIOx, uint8_t enable);

void GPIO_Init(gpio_dev_t* gpio_dev, gpio_port_config_t* port_config);

void GPIO_Peri_Reset(GPIOx_RegDef_t* pGPIOx);

uint16_t GPIO_Get_Input(GPIOx_RegDef_t* pGPIOx);

uint8_t GPIO_Get_Port_Input(gpio_dev_t* gpio_dev);

void GPIO_Update_Output(gpio_dev_t* gpio_dev, uint16_t value);

void GPIO_Update_Port_Output(gpio_dev_t* gpio_dev, uint8_t value);

void GPIO_toggle_Port_Output(gpio_dev_t* gpio_dev);

void GPIO_IRQHandler(uint8_t PinNumber);

#endif /* DRIVERS_INC_GPIO_H_ */
