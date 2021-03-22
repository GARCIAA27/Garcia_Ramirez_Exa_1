/*
 * main.c
 *
 *  Created on: 22 mar. 2021
 *      Author: Alexis
 */



#include "Bits.h"
#include "GPIO.h"
#include "NVIC.h"
#include "PIT.h"
#include "Select_Color.h"
#include <stdint.h>


int main(void)
{
	select_init();

	return 0;
}

