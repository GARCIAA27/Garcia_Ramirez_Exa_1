/*
 * Delay.c
 *
 *  Created on: 11 feb. 2021
 *      Author: LUISPIZANO
 */
#include "delay.h"

void delay(uint32_t delay)
{
	volatile uint32_t j;

	for(j = delay; j > 0; j--)
	{
		__asm("nop");

	}
}

/********** END OF FILE **********/
