/*
 * irq.h
 *
 *  Created on: Apr 12, 2024
 *      Author: leeinfy
 */

#ifndef INC_IRQ_H_
#define INC_IRQ_H_
#include "stm32f407.h"

#define NUM_PR_BIT			4

#define IRQ_NUM_EXTI0		6
#define IRQ_NUM_EXTI1		7
#define IRQ_NUM_EXTI2		8
#define IRQ_NUM_EXTI3		9
#define IRQ_NUM_EXTI4		10
#define IRQ_NUM_EXTI5		23
#define IRQ_NUM_EXTI6		23
#define IRQ_NUM_EXTI7		23
#define IRQ_NUM_EXTI8		23
#define IRQ_NUM_EXTI9		23
#define IRQ_NUM_EXTI10		40
#define IRQ_NUM_EXTI11		40
#define IRQ_NUM_EXTI12		40
#define IRQ_NUM_EXTI13		40
#define IRQ_NUM_EXTI14		40

void IRQConfig(uint8_t IRQNumber, uint8_t enable);

void IRQ_Priority_Config(uint8_t IRQNumber, uint8_t IRQPriority);

#endif /* INC_IRQ_H_ */
