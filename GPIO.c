/**
	\file
	\brief
		This is the source file for the GPIO device driver for Kinetis K64.
		It contains all the implementation for configuration functions and runtime functions.
		i.e., this is the application programming interface (API) for the GPIO peripheral.
	\author J. Luis Pizano Escalante, luispizano@iteso.mx
	\date	18/02/2019
	\todo
	    Interrupts are not implemented in this API implementation.
 */
#include "MK64F12.h"
#include "GPIO.h"
#include "Bits.h"



static void (*gpio_A_callback)(void) = 0;
static void (*gpio_B_callback)(void) = 0;
static void (*gpio_C_callback)(void) = 0;
static void (*gpio_D_callback)(void) = 0;
static void (*gpio_E_callback)(void) = 0;

static gpio_interrupt_flags_t g_intr_status_flag = {0};


uint8_t GPIO_clock_gating(gpio_port_name_t port_name)
{
	switch(port_name)/** Selecting the GPIO for clock enabling*/
	{
		// GPIO A is selected
		//
		case GPIO_A:
			SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTA; /** Bit 9 of SIM_SCGC5 is  set*/
		break;

		// GPIO B is selected
		//
		case GPIO_B:
			SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTB; /** Bit 10 of SIM_SCGC5 is set*/
		break;

		// GPIO C is selected
		//
		case GPIO_C:
			SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTC; /** Bit 11 of SIM_SCGC5 is set*/
		break;

		// GPIO D is selected
		//
		case GPIO_D:
			SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTD; /** Bit 12 of SIM_SCGC5 is set*/
		break;

		// GPIO E is selected
		//
		case GPIO_E:
			SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTE; /** Bit 13 of SIM_SCGC5 is set*/
		break;

		// If does not exist the option
		//
		default:
			return(FALSE);
	}

	//Successful configuration
	//
	return(TRUE);

} /* GPIO_clock_gating() */

uint8_t GPIO_pin_control_register(gpio_port_name_t port_name, uint8_t pin, const gpio_pin_control_register_t*  pin_control_register)
{
	switch(port_name)
	{
		// GPIO A is selected
		//
		case GPIO_A:
			PORTA->PCR[pin] = *pin_control_register;
		break;

		// GPIO B is selected
		//
		case GPIO_B:
			PORTB->PCR[pin] = *pin_control_register;
		break;

		// GPIO C is selected
		//
		case GPIO_C:
			PORTC->PCR[pin] = *pin_control_register;
		break;

		// GPIO D is selected
		//
		case GPIO_D:
			PORTD->PCR[pin] = *pin_control_register;
		break;

		// GPIO E is selected
		//
		case GPIO_E:
			PORTE->PCR[pin] = *pin_control_register;
		break;

		// If does not exist the option
		//
		default:
			return(FALSE);
	}

	//Successful configuration
	//
	return(TRUE);
}

void GPIO_write_port(gpio_port_name_t port_name, uint32_t data)
{
	switch(port_name)
	{
		// GPIO A is selected
		//
		case GPIO_A:
			GPIOA->PDOR |= data; /** All data is written in the Port Data Output Register from GPIOA*/
		break;

		// GPIO B is selected
		//
		case GPIO_B:
			GPIOB->PDOR |= data; /** All data is written in the Port Data Output Register from GPIOB*/
		break;

		// GPIO C is selected
		//
		case GPIO_C:
			GPIOC->PDOR |= data; /** All data is written in the Port Data Output Register from GPIOC*/
		break;

		// GPIO D is selected
		//
		case GPIO_D:
			GPIOD->PDOR |= data; /** All data is written in the Port Data Output Register from GPIOD*/
		break;

		// GPIO E is selected
		//
		case GPIO_E:
			GPIOE->PDOR |= data; /** All data is written in the Port Data Output Register from GPIOE*/
		break;

		// If does not exist the option
		//
		default:
			return;
	}

	//Successful configuration
	//
	return;

} /* GPIO_write_port() */

uint32_t GPIO_read_port(gpio_port_name_t port_name)
{
	switch(port_name)
	{
		// GPIO A is selected
		//
		case GPIO_A:
			return(GPIOA->PDIR); /** All data is read in the Port Data Input Register from GPIOA*/
		break;

		// GPIO B is selected
		//
		case GPIO_B:
			return(GPIOB->PDIR); /** All data is read in the Port Data Input Register from GPIOB*/
		break;

		// GPIO C is selected
		//
		case GPIO_C:
			return(GPIOC->PDIR); /** All data is read in the Port Data Input Register from GPIOC*/
		break;

		// GPIO D is selected
		//
		case GPIO_D:
			return(GPIOD->PDIR); /** All data is read in the Port Data Input Register from GPIOD*/
		break;

		// GPIO E is selected
		//
		case GPIO_E:
			return(GPIOE->PDIR); /** All data is read in the Port Data Input Register from GPIOE*/
		break;

		// If does not exist the option
		//
		default:
			return(FALSE);
	}

	//If port_name receive an invalid parameter
	//
	return(FALSE);

} /* GPIO_read_port() */

uint8_t GPIO_read_pin(gpio_port_name_t port_name, uint8_t pin)
{
	uint32_t static read_pin = 0;

	read_pin = 1 << pin;

	switch(port_name)
	{
		// GPIO A is selected
		//
		case GPIO_A:
			read_pin &= GPIOA->PDIR;
		break;

		// GPIO B is selected
		//
		case GPIO_B:
			read_pin &= GPIOB->PDIR;
		break;

		// GPIO C is selected
		//
		case GPIO_C:
			read_pin &= GPIOC->PDIR;
		break;

		// GPIO D is selected
		//
		case GPIO_D:
			read_pin &= GPIOD->PDIR;
		break;

		// GPIO E is selected
		//
		case GPIO_E:
			read_pin &= GPIOE->PDIR;
		break;

		// If does not exist the option
		//
		default:
			return(FALSE);
	}

	//If read_pin read something, put one in read_pin, if not, put a zero
	//
	read_pin = (read_pin != 0) ? TRUE:FALSE;

	//Return the pin value (zero or one)
	return( (uint8_t)read_pin );

} /* GPIO_read_pin() */

void GPIO_set_pin(gpio_port_name_t port_name, uint8_t pin)
{
	switch(port_name)
	{
		// GPIO A is selected
		//
		case GPIO_A:
			GPIOA->PSOR |= (BIT_ON << pin);
		break;

		// GPIO B is selected
		//
		case GPIO_B:
			GPIOB->PSOR |= (BIT_ON << pin);
		break;

		// GPIO C is selected
		//
		case GPIO_C:
			GPIOC->PSOR |= (BIT_ON << pin);
		break;

		// GPIO D is selected
		//
		case GPIO_D:
			GPIOD->PSOR |= (BIT_ON << pin);
		break;

		// GPIO E is selected
		//
		case GPIO_E:
			GPIOE->PSOR |= (BIT_ON << pin);
		break;

		// If does not exist the option
		//
		default:
			return;
	}

	//Successful configuration
	//
	return;

}  /* GPIO_set_pin() */

void GPIO_clear_pin(gpio_port_name_t port_name, uint8_t pin)
{
	switch(port_name)
	{
		// GPIO A is selected
		//
		case GPIO_A:
			GPIOA->PCOR |= BIT_ON << pin;
		break;

		// GPIO B is selected
		//
		case GPIO_B:
			GPIOB->PCOR |= BIT_ON << pin;
		break;

		// GPIO C is selected
		//
		case GPIO_C:
			GPIOC->PCOR |= BIT_ON << pin;
		break;

		// GPIO D is selected
		//
		case GPIO_D:
			GPIOD->PCOR |= BIT_ON << pin;
		break;

		// GPIO E is selected
		//
		case GPIO_E:
			GPIOE->PCOR |= BIT_ON << pin;
		break;

		// If does not exist the option
		//
		default:
			return;
	}

	//Successful configuration
	//
	return;

} /* GPIO_clear_pin() */

void GPIO_toogle_pin(gpio_port_name_t port_name, uint8_t pin)
{
	switch(port_name)
	{
		// GPIO E is selected
		//
		case GPIO_A:
			GPIOA->PTOR ^= (BIT_ON << pin);
		break;

		// GPIO B is selected
		//
		case GPIO_B:
			GPIOB->PTOR ^=  (BIT_ON << pin);
		break;

		// GPIO C is selected
		//
		case GPIO_C:
			GPIOC->PTOR ^=  (BIT_ON << pin);
		break;

		// GPIO D is selected
		//
		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PTOR ^=  (BIT_ON << pin);
		break;

		// GPIO E is selected
		//
		case GPIO_E:
			GPIOE->PTOR ^= (BIT_ON << pin);
		break;

		// If does not exist the option
		//
		default:
			return;
	}

	//Successful configuration
	//
	return;

} /* GPIO_toogle_pin() */

void GPIO_data_direction_port(gpio_port_name_t port_name ,uint32_t direction)
{
	switch(port_name)
	{
		// GPIO A is selected
		//
		case GPIO_A:
			GPIOA->PDDR |= direction; /** Data Direction is pointed from the Port Data Direction Register from GPIOA*/
		break;

		// GPIO B is selected
		//
		case GPIO_B:
			GPIOB->PDDR |= direction; /** Data Direction is pointed from the Port Data Direction Register from GPIOB*/
		break;

		// GPIO C is selected
		//
		case GPIO_C:
			GPIOC->PDDR |= direction; /** Data Direction is pointed from the Port Data Direction Register from GPIOC*/
		break;

		// GPIO D is selected
		//
		case GPIO_D:
			GPIOD->PDDR |= direction; /** Data Direction is pointed from the Port Data Direction Register from GPIOD*/
		break;

		// GPIO E is selected
		//
		case GPIO_E:
			GPIOE->PDDR |= direction; /** Data Direction is pointed from the Port Data Direction Register from GPIOE*/
		break;

		// If does not exist the option
		//
		default:
			return;
	}

	//Successful configuration
	//
	return;

} /* GPIO_data_direction_port() */

void GPIO_data_direction_pin(gpio_port_name_t port_name, uint8_t state, uint8_t pin)
{
	switch(port_name)
	{
		// GPIO A is selected
		//
		case GPIO_A:
			GPIOA->PDDR |= state << pin; /** Data Direction Pin is pointed to be Input or Output from GPIOA*/
		break;

		// GPIO B is selected
		//
		case GPIO_B:
			GPIOB->PDDR |= state << pin; /** Data Direction Pin is pointed to be Input or Output from GPIOB*/
		break;

		// GPIO C is selected
		//
		case GPIO_C:
			GPIOC->PDDR |= state << pin; /** Data Direction Pin is pointed to be Input or Output from GPIOC*/
		break;

		// GPIO D is selected
		//
		case GPIO_D:
			GPIOD->PDDR |= state << pin; /** Data Direction Pin is pointed to be Input or Output from GPIOD*/
		break;

		// GPIO E is selected
		//
		case GPIO_E:
			GPIOE->PDDR |= state << pin; /** Data Direction Pin is pointed to be Input or Output from GPIOE*/
		break;

		// If does not exist the option
		//
		default:
			return;
	}

	//Successful configuration
	//
	return;

} /* GPIO_data_direction_pin() */

void SW3_init(void)
{
	/**Pin control configuration of GPIOA pin4 (SW3) as GPIO with Pull-Up Enable*/
	gpio_pin_control_register_t pcr_gpioa_pin_4 = GPIO_MUX1 | GPIO_PE | GPIO_PS| INTR_FALLING_EDGE;
	GPIO_clock_gating(GPIO_A);
	/**Pin control configuration of GPIOA pin4 (SW3)*/
	GPIO_pin_control_register(GPIO_A,bit_4,  &pcr_gpioa_pin_4);
	GPIO_data_direction_pin(GPIO_A, GPIO_INPUT,  bit_4);
}

void SW2_init(void)
{
	/**Pin control configuration of GPIOC pin6 (SW2) as GPIO with Pull-Up Enable*/
	gpio_pin_control_register_t pcr_gpioc_pin_6 = GPIO_MUX1 | GPIO_PE | GPIO_PS | INTR_FALLING_EDGE;
	GPIO_clock_gating(GPIO_C);
	/**Pin control configuration of GPIOC pin6 (SW2)*/
	GPIO_pin_control_register(GPIO_C,bit_6,  &pcr_gpioc_pin_6);
	GPIO_data_direction_pin(GPIO_C, GPIO_INPUT,  bit_6);
}

void external_leds_init()
{
	gpio_pin_control_register_t pcr_gpioc_pin_0 = GPIO_MUX1;
	gpio_pin_control_register_t pcr_gpioc_pin_9 = GPIO_MUX1;
	GPIO_clock_gating(GPIO_C);
	GPIO_pin_control_register(GPIO_C,bit_0, &pcr_gpioc_pin_0);
	GPIO_pin_control_register(GPIO_C,bit_9, &pcr_gpioc_pin_9);
	GPIO_clear_pin(GPIO_C,bit_0);
	GPIO_clear_pin(GPIO_C,bit_9);
	GPIO_data_direction_pin(GPIO_C, GPIO_OUTPUT, bit_0);
	GPIO_data_direction_pin(GPIO_C, GPIO_OUTPUT, bit_9);

}
/***************************** INTERRPUTS *******************************/

void GPIO_callback_init(gpio_port_name_t port_name,void (*handler)(void))
{
	switch(port_name)/** Selecting the GPIO*/
	{
		case GPIO_A: /** GPIO A is selected*/
		gpio_A_callback = handler;
		break;
		case GPIO_B: /** GPIO B is selected*/
			gpio_B_callback = handler;
		break;
		case GPIO_C: /** GPIO C is selected*/
			gpio_C_callback = handler;
		break;
		case GPIO_D: /** GPIO D is selected*/
			gpio_D_callback = handler;
		break;
		case GPIO_E:  /** GPIO E is selected*/
		gpio_E_callback = handler;
		break;
		default:
		break;

	}// end switch
}




void GPIO_clear_irq_status(gpio_port_name_t port_name)
{
	switch(port_name)/** Selecting the GPIO*/
	{
		case GPIO_A: /** GPIO A is selected*/
		g_intr_status_flag.flag_port_a = FALSE;
		break;
		case GPIO_B: /** GPIO B is selected*/
		g_intr_status_flag.flag_port_b = FALSE;
		break;
		case GPIO_C: /** GPIO C is selected*/
		g_intr_status_flag.flag_port_c = FALSE;
		break;
		case GPIO_D: /** GPIO D is selected*/
		g_intr_status_flag.flag_port_d = FALSE;
		break;
		case GPIO_E:  /** GPIO E is selected*/
		g_intr_status_flag.flag_port_e = FALSE;
		break;
		default:
		break;

	}// end switch
}

uint8_t GPIO_get_irq_status(gpio_port_name_t port_name)
{
	uint8_t status = 0;
	switch(port_name)/** Selecting the GPIO*/
		{
			case GPIO_A: /** GPIO A is selected*/
			status = g_intr_status_flag.flag_port_a;
			break;
			case GPIO_B: /** GPIO B is selected*/
			status = g_intr_status_flag.flag_port_b;
			break;
			case GPIO_C: /** GPIO C is selected*/
			status = g_intr_status_flag.flag_port_c;
			break;
			case GPIO_D: /** GPIO D is selected*/
			status = g_intr_status_flag.flag_port_d;
			break;
			case GPIO_E:  /** GPIO E is selected*/
			status = g_intr_status_flag.flag_port_e;
			break;
			default:
			break;

		}// end switch

	return(status);
}



void GPIO_clear_interrupt(gpio_port_name_t port_name)
{
	switch(port_name)/** Selecting the GPIO for clock enabling*/
	{
		case GPIO_A: /** GPIO A is selected*/
		PORTA->ISFR=0xFFFFFFFF;
		break;
		case GPIO_B: /** GPIO B is selected*/
		PORTB->ISFR=0xFFFFFFFF;
		break;
		case GPIO_C: /** GPIO C is selected*/
		PORTC->ISFR = 0xFFFFFFFF;
		break;
		case GPIO_D: /** GPIO D is selected*/
		PORTD->ISFR=0xFFFFFFFF;
		break;
		case GPIO_E: /** GPIO E is selected*/
		PORTE->ISFR=0xFFFFFFFF;
		break;
		default:
		break;

	}// end switch
}


void PORTA_IRQHandler(void)
{
	if(gpio_A_callback)
	{
		gpio_A_callback();
	}

	GPIO_clear_interrupt(GPIO_A);
}

void PORTB_IRQHandler(void)
{
	if(gpio_B_callback)
	{
		gpio_B_callback();
	}

	GPIO_clear_interrupt(GPIO_B);
}

void PORTC_IRQHandler(void)
{
	if(gpio_C_callback)
	{
		gpio_C_callback();
	}

	GPIO_clear_interrupt(GPIO_C);

}

void PORTD_IRQHandler(void)
{
	if(gpio_D_callback)
	{
		gpio_D_callback();
	}

	GPIO_clear_interrupt(GPIO_D);
}

void PORTE_IRQHandler(void)
{
	if(gpio_E_callback)
	{
		gpio_E_callback();
	}

	GPIO_clear_interrupt(GPIO_E);
}
/********** END OF FILE **********/
