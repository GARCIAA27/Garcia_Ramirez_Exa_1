/*
 * Select_Color.c
 *
 *  Created on: 22 mar. 2021
 *      Author: Alexis
 */

#include "Select_Color.h"

#define DELAY (1.0F)

static uint8_t sec_counter = 0;
static uint8_t push_count = 0;

static uint8_t white_flag = FALSE;
static uint8_t current_state;
static uint8_t out;
void (*colors[]) (void) = {blue_on, green_on, red_on, yellow_on, purple_on, cyan_on};

typedef enum {
	WAIT,
	SW2,
	SEND_OUT
}State_name_t;

void select_init()
{
	PIT_clock_gating();
	PIT_enable();
	NVIC_enable_interrupt_and_priotity(PIT_CH0_IRQ, PRIORITY_3);
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
	current_state=WAIT;


}
void white_blink()
{
	if(sec_counter <= 3)
	{
		rgb_off();
		white_flag =FALSE;
		PIT_stop(PIT_0);
		sec_counter=0;
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

void count_push()
{
	if(push_count > 6)
	{

	}
	else{
		push_count++;
		out=push_count;
	}

}

void count_five(){
	if(sec_counter <= 5)
	{
		sec_counter++;
	}
	else
	{
		current_state=SEND_OUT;
	}
}
void state_machine_run()
{

	switch (current_state)
	{
		case WAIT:

			rgb_off();
			sec_counter=0;
			push_count=0;
		break;

		case SW2:
			GPIO_callback_init(GPIO_A, count_push);
			PIT_callback_init(PIT_0,count_five);
			PIT_delay(PIT_0,SYSTEM_CLOCK,DELAY);
		break;

		case SEND_OUT:
			colors[out]();
			current_state= WAIT;
		break;
		default:
		break;
		}
}


