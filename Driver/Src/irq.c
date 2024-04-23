/*
 * irq.c
 *
 *  Created on: Apr 16, 2024
 *      Author: leeinfy
 */
#include "irq.h"

/* @fn 		IRQConfig
 * @brief	enable or disable the IRQ in Arm Cortex NVIC
 * @param	IRQNumber, the corresponding IRQnumber of the peripheral
 * @param 	enable, 0 or 1
 * */
void IRQConfig(uint8_t IRQNumber, uint8_t enable){
	uint8_t temp1 = IRQNumber / 32;
	uint8_t temp2 = IRQNumber % 32;
	//enable corresponding Interrupt in NVIC
	if(enable == 1){
		NVIC_ISER(temp1) |= (1 << temp2);
	}else{
		NVIC_ICER(temp1) |= (1 << temp2);
	}

}

/* @fn 		IRQ_Priority_Config
 * @brief	set the IRQ priority of that IRQ number
 * @param	IRQNumber, the corresponding IRQnumber of the peripheral
 * @param 	IRQPriority, priority
 * */
void IRQ_Priority_Config(uint8_t IRQNumber, uint8_t IRQPriority){
	uint8_t temp1 = IRQNumber / 4;
	uint8_t temp2 = IRQNumber % 4;
	//set the interrupt priority
	NVIC_IPR(temp1) |= (IRQPriority << (temp2 * 8 + (8 - NUM_PR_BIT)));
}

