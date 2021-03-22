/*
 * Select_Color.c
 *
 *  Created on: 22 mar. 2021
 *      Author: Alexis
 */

#include "Select_Color.h"

void select_init()
{
	PIT_enable();
	PIT_clock_gating();
	NVIC_enable_interrupt_and_priotity(PIT_CH0_IRQ, PRIORITY_3);
	PIT_enable();
	PIT_enable_interrupt(PIT_0);
	SW2_init();
	NVIC_enable_interrupt_and_priotity(PORTC_IRQ, PRIORITY_4);
	SW3_init();
	NVIC_enable_interrupt_and_priotity(PORTA_IRQ, PRIORITY_4);
	rgb_init();
	NVIC_global_enable_interrupts;
}
