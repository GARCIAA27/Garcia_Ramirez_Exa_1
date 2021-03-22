/*
 * RGB.c
 *
 *  Created on: 17 feb. 2021
 *      Author: Alexis
 */

#include "RGB.h"
#include "GPIO.h"
#include "Bits.h"
#include "Delay.h"

/**TURNING DIFFERENT LED COLOURS ON FUNCTIONS
 *! WE NEED TO CLEAR THE PIN*/
void rgb_init()
{
	gpio_pin_control_register_t pcr_gpiob_pin_21 = GPIO_MUX1;
	gpio_pin_control_register_t pcr_gpiob_pin_22 = GPIO_MUX1;
	gpio_pin_control_register_t pcr_gpioe_pin_26 = GPIO_MUX1;

	GPIO_clock_gating(GPIO_B);
	GPIO_clock_gating(GPIO_E);

	/**Pin control configuration of GPIOB pin21 (Blue LED) as GPIO*/
	GPIO_pin_control_register(GPIO_B,bit_21, &pcr_gpiob_pin_21);

	/**Pin control configuration of GPIOB pin22 (Red LED) as GPIO*/
	GPIO_pin_control_register(GPIO_B,bit_22, &pcr_gpiob_pin_22);

	/**Pin control configuration of GPIOE pin26 (Green LED) as GPIO*/
	GPIO_pin_control_register(GPIO_E,bit_26, &pcr_gpioe_pin_26);

	//Assigns a safe value to the output pin
	GPIO_write_port(GPIO_B, GPIO_OFF_CONST);
	GPIO_write_port(GPIO_E, GPIO_OFF_CONST);

	//Assigns the port pin as output
	GPIO_data_direction_pin(GPIO_B, GPIO_OUTPUT, bit_21);
	GPIO_data_direction_pin(GPIO_B, GPIO_OUTPUT, bit_22);
	GPIO_data_direction_pin(GPIO_E, GPIO_OUTPUT, bit_26);
}

void red_on()
{
	GPIO_clear_pin(GPIO_B,bit_22);

	return;

} /* red_on() */

void blue_on()
{
	GPIO_clear_pin(GPIO_B,bit_21);

	return;

} /* blue_on() */

void green_on()
{
	GPIO_clear_pin(GPIO_E,bit_26);

	return;

} /* green_on() */

void yellow_on()
{
	green_on();
	red_on();

	return;

} /* yellow_on() */

void purple_on()
{
	red_on();
	blue_on();

	return;

} /* purple_on() */

void white_on()
{
	green_on();
	blue_on();
	red_on();

	return;

} /* white_on() */

/**TURNING DIFFERENT LED COLOURS OFF FUNCTIONS
 *! WE NEED TO SET THE PIN*/

void red_off()
{
	GPIO_set_pin(GPIO_B,bit_22);

	return;

}  /* red_off() */

void blue_off()
{
	GPIO_set_pin(GPIO_B,bit_21);

	return;

} /* blue_off() */

void green_off()
{
	GPIO_set_pin(GPIO_E,bit_26);

	return;

} /* green_off() */

void yellow_off()
{
	green_off();
	red_off();

	return;

} /* yellow_off() */

void purple_off()
{
	red_off();
	blue_off();

	return;

} /* purple_off() */

void rgb_off()
{
	green_off();
	blue_off();
	red_off();

	return;

} /* rgb_off() */

void boot_routine(uint32_t const DELAY)
{
	green_on();

	return;

} /* boot_routine() */

void sw2_sequence()
{
	rgb_off();
	yellow_on();
	delay(DELAY_CONST);
	yellow_off();
	red_on();
	delay(DELAY_CONST);
	red_off();
	purple_on();
	delay(DELAY_CONST);
	purple_off();
	blue_on();
	delay(DELAY_CONST);
	blue_off();
	green_on();
	delay(DELAY_CONST);
	green_off();

	return;

} /* sw2_sequence() */

void sw3_sequence()
{
	rgb_off();
	green_on();
	delay(DELAY_CONST);
	green_off();
	blue_on();
	delay(DELAY_CONST);
	blue_off();
	purple_on();
	delay(DELAY_CONST);
	purple_off();
	red_on();
	delay(DELAY_CONST);
	red_off();
	yellow_on();
	delay(DELAY_CONST);
	yellow_off();

	return;

} /* sw3_sequence() */

/********** END OF FILE **********/
