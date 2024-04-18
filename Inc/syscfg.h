/*
 * syscfg.h
 *
 *  Created on: Apr 12, 2024
 *      Author: leeinfy
 */

#ifndef INC_SYSCFG_H_
#define INC_SYSCFG_H_

#include "stm32f407.h"

#define SYSCFG_PERI_CLK_EN()	(RCC->APB2ENR |= (1 << 14))

#define SYSCFG_PERI_CLK_DIS()	(RCC->APB2ENR &= ~(1 << 14))


#endif /* INC_SYSCFG_H_ */
