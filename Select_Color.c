/*
 * Select_Color.c
 *
 *  Created on: 22 mar. 2021
 *      Author: Alexis
 */

#include "Select_Color.h"

#define DELAY (1.0F)
static uint8_t sec_counter = 0;
static uint8_t white_flag = FALSE;
void select_init()
{
	PIT_clock_gating();
	PIT_enable();
	NVIC_enable_interrupt_and_priotity(PIT_CH0_IRQ, PRIORITY_3);
	PIT_enable();
	PIT_enable_interrupt(PIT_0);
	PIT_callback_init(PIT_0,white_blink);
	SW2_init();
	NVIC_enable_interrupt_and_priotity(PORTC_IRQ, PRIORITY_4);
	GPIO_callback_init(GPIO_C,SM_init);
	SW3_init();
	NVIC_enable_interrupt_and_priotity(PORTA_IRQ, PRIORITY_4);
	rgb_init();
	NVIC_global_enable_interrupts;
}

void SM_init()
{
	PIT_delay(PIT_0,SYSTEM_CLOCK,DELAY);

}
void white_blink()
{
	if(sec_counter >= 3)
	{
		rgb_off();
		white_flag =FALSE;
	}
	if(white_flag)
	{
		rgb_off();
		white_flag = FALSE;
	}
	else
	{
		white_on();
		white_flag = TRUE;
	}
	sec_counter++;
}
