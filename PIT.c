/*
 * PIT.c
 *
 *  Created on: 28 feb. 2021
 *      Author: Alexis
 */
#include "PIT.h"

/*Callbacks for each channel*/
static void (*PIT_CH0_callback)(void) = 0;
static void (*PIT_CH1_callback)(void) = 0;
static void (*PIT_CH2_callback)(void) = 0;
static void (*PIT_CH3_callback)(void) = 0;

/*Initialize flags at 0*/
pit_interrupt_flags_t pit_interrupt_flags = {0};

void PIT_enable(void)
{

	PIT->MCR &= ~PIT_MCR_MDIS_MASK;
	PIT->MCR |= PIT_MCR_FRZ_MASK;

	return;

} /* PIT_enable() */

void PIT_delay(PIT_timer_t pit_timer, My_float_pit_t system_clock , My_float_pit_t delay)
{

	switch (pit_timer)
	{
		case PIT_0:
			PIT->CHANNEL[0].LDVAL = (uint32_t)((delay * (system_clock / 2)) - 1);
		break;

		case PIT_1:
			PIT->CHANNEL[1].LDVAL = (uint32_t)((delay * (system_clock / 2)) - 1);
		break;

		case PIT_2:
			PIT->CHANNEL[2].LDVAL = (uint32_t)((delay * (system_clock / 2)) - 1);
		break;

		case PIT_3:
			PIT->CHANNEL[3].LDVAL = (uint32_t)((delay * (system_clock / 2)) - 1);
		break;

		default:
		break;
	}

	PIT_start(pit_timer);
	return;

} /* PIT_delay() */

void PIT_stop(PIT_timer_t pit_timer)
{

	switch (pit_timer)
	{
		case PIT_0:
		PIT->CHANNEL[0].TCTRL &= ~( PIT_TCTRL_TEN_MASK );
		break;
		case PIT_1:
		PIT->CHANNEL[1].TCTRL &= ~( PIT_TCTRL_TEN_MASK );
		break;
		case PIT_2:
		PIT->CHANNEL[2].TCTRL &= ~( PIT_TCTRL_TEN_MASK );
		break;
		case PIT_3:
		PIT->CHANNEL[3].TCTRL &= ~( PIT_TCTRL_TEN_MASK );
		break;
		default:
		break;
	}

	return;

} /* PIT_stop() */

void PIT_start(PIT_timer_t pit_timer)
{

	switch (pit_timer)
	{
		case PIT_0:
			PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
		break;

		case PIT_1:
			PIT->CHANNEL[1].TCTRL |= PIT_TCTRL_TEN_MASK;
		break;

		case PIT_2:
			PIT->CHANNEL[2].TCTRL |= PIT_TCTRL_TEN_MASK;
		break;

		case PIT_3:
			PIT->CHANNEL[3].TCTRL |= PIT_TCTRL_TEN_MASK;
		break;

		default:
		break;
	}

	return;

} /* PIT_start */

void PIT_clock_gating(void)
{

	SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;

	return;

} /* PIT_clock_gating() */

uint8_t PIT_get_interrupt_flag_status(PIT_timer_t pit)
{
	uint8_t flag_status=0;
	//Select flag status of a specific channel
	//!These flags are programmer flags not hardware flags
	switch (pit)
	{
	case PIT_0:
	flag_status = pit_interrupt_flags.flag_PIT_CH0;
	break;
	case PIT_1:
	flag_status = pit_interrupt_flags.flag_PIT_CH1;
	break;
	case PIT_2:
	flag_status = pit_interrupt_flags.flag_PIT_CH2;
	break;
	case PIT_3:
	flag_status = pit_interrupt_flags.flag_PIT_CH3;
	break;
	default:
	break;
	}
	//Return flag status of selected channel
	return flag_status;
}/*PIT_get_interrupt_flag_status*/

void PIT_clear_interrupt_flag(PIT_timer_t pit)
{
	//Clear flag status of a specific channel
	//!These flags are programmer flags not hardware flags
	switch (pit)
	{
	case PIT_0:
	pit_interrupt_flags.flag_PIT_CH0 = 0;
	break;
	case PIT_1:
	pit_interrupt_flags.flag_PIT_CH1 = 0;
	break;
	case PIT_2:
	pit_interrupt_flags.flag_PIT_CH2 = 0;
	break;
	case PIT_3:
	pit_interrupt_flags.flag_PIT_CH3 = 0;
	break;
	default:
	break;
	}
}/*PIT_clear_interrupt_flag*/

void PIT_enable_interrupt(PIT_timer_t pit)
{
	//Enable interrupt of a specific channel
	switch (pit)
	{
	case PIT_0:
	PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TIE_MASK;
	break;
	case PIT_1:
	PIT->CHANNEL[1].TCTRL = PIT_TCTRL_TIE_MASK;
	break;
	case PIT_2:
	PIT->CHANNEL[2].TCTRL = PIT_TCTRL_TIE_MASK;
	break;
	case PIT_3:
	PIT->CHANNEL[3].TCTRL = PIT_TCTRL_TIE_MASK;
	break;
	default:
	break;
	}
}/*PIT_enable_interrupt*/

void PIT_callback_init(PIT_timer_t pit, void(*handler)(void))
{
	//Set the handler of the specific channel interrupt
	switch (pit)
	{
	case PIT_0:
	PIT_CH0_callback = handler;
	break;
	case PIT_1:
	PIT_CH1_callback = handler;
	break;
	case PIT_2:
	PIT_CH2_callback = handler;
	break;
	case PIT_3:
	PIT_CH3_callback = handler;
	break;
	default:
	break;
	}
}/*PIT_callback_init */

void PIT_clear_interrupt(PIT_timer_t pit)
{
	uint32_t dummyRead=0;
	//Clear the hardware flag of a channel
	switch (pit)
	{
	case PIT_0:
	PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;
	dummyRead = PIT->CHANNEL[0].TCTRL;//read control register for clear PIT flag, this is silicon bug
	break;
	case PIT_1:
	PIT->CHANNEL[1].TFLG |= PIT_TFLG_TIF_MASK;
	dummyRead = PIT->CHANNEL[1].TCTRL;//read control register for clear PIT flag, this is silicon bug
	break;
	case PIT_2:
	PIT->CHANNEL[2].TFLG |= PIT_TFLG_TIF_MASK;
	dummyRead = PIT->CHANNEL[2].TCTRL;//read control register for clear PIT flag, this is silicon bug
	break;
	case PIT_3:
	PIT->CHANNEL[3].TFLG |= PIT_TFLG_TIF_MASK;
	dummyRead = PIT->CHANNEL[3].TCTRL;//read control register for clear PIT flag, this is silicon bug
	break;
	default:
	break;
	}

}/*PIT_clear_interruptr */

/*These functions are declared in startup_mk64f12.c*/
void PIT0_IRQHandler(void)
{
	//If callback exists
	if(PIT_CH0_callback)
	{
		//Do the callback routine
		PIT_CH0_callback();
	}
	//Clear both programmer and hardware flags
	PIT_clear_interrupt_flag(PIT_0);
	PIT_clear_interrupt(PIT_0);
}/*PIT0_IRQHandler */

void PIT1_IRQHandler(void)
{
	//If callback exists
	if(PIT_CH1_callback)
	{
		//Do the callback routine
		PIT_CH1_callback();
	}

	//Clear both programmer and hardware flags
	PIT_clear_interrupt_flag(PIT_1);
	PIT_clear_interrupt(PIT_1);
}/*PIT1_IRQHandler */

void PIT2_IRQHandler(void)
{
	//If callback exists
	if(PIT_CH2_callback)
	{
		//Do the callback routine
		PIT_CH2_callback();
	}

	//Clear both programmer and hardware flags
	PIT_clear_interrupt_flag(PIT_2);
	PIT_clear_interrupt(PIT_2);
}/*PIT2_IRQHandler */

void PIT3_IRQHandler(void)
{
	//If callback exists
	if(PIT_CH3_callback)
	{
		//Do the callback routine
		PIT_CH3_callback();
	}

	//Clear both programmer and hardware flags
	PIT_clear_interrupt_flag(PIT_3);
	PIT_clear_interrupt(PIT_3);
}/*PIT3_IRQHandler */

/********END_OF_FILE**************/
