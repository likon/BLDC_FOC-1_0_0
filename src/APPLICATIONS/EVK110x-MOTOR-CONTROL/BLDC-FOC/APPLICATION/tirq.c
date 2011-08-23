/**********************************************************************
 * File: tirq.c
 * Author: Marcus Jansson
 * Copyright (C) 2011 Marcus Jansson <mjansson256@gmail.com>
 * and Thomas Eberharter, Eberharter Elektronik GmbH
 *
 * This file is the replacement for the Hall detector code.
 *********************************************************************/

#include "tirq.h"

#define TC_CHANNEL_0 0

//TODO: Remove MLED and BUT definitions and relevant debug code
//~ #define MLED0 AVR32_PIN_PB27
//~ #define MLED1 AVR32_PIN_PB28
//~ #define MLED2 AVR32_PIN_PB29
//~ #define MLED3 AVR32_PIN_PB30
//~
//~ #define BUT0 AVR32_PIN_PX16
//~ #define BUT1 AVR32_PIN_PX19
//~ #define BUT2 AVR32_PIN_PX22

//TODO: move away
void tirq_int_handler(void);

volatile unsigned short first_interrupt = 0;
volatile unsigned short nieme = 0;
volatile U32 tirq_demi_period = tirq_demi_period_init ;
volatile unsigned short teta0 = 0;
volatile U32 tirq_tj;
volatile U32 tirq_ti;                //!<  last hall period value

void tirq_estimator_update_teta_and_speed(volatile unsigned short *teta_elec, volatile unsigned short *vitesse_elec)
{
   nieme++;
   // 180*Fcpu*100e-6
  *teta_elec = (unsigned short)((U32)(vitesse_inst * 1 * nieme) / (U32)tirq_demi_period);
  if(*teta_elec > 360) {
    *teta_elec = 360;
    nieme = 0;
  }
  *vitesse_elec = PI_X_FCPU / tirq_demi_period;  //pi*Fcpu  (Fcpu=48Mhz)
}

void tirq_estimator_init_teta(volatile unsigned short teta)
{
    nieme = (int)(((teta * tirq_demi_period) / vitesse_inst) + 2);
}
//------------------------------------------------------------------------------
/*! \name Initialization function
 */
//! @{
void tirq_estimator_init(void)
{
    nieme = 0;
}

//------------------------------------------------------------------------------
/*! \name Interrupt intitialization function
 */
//! @{
void tirq_estimator_init_interrupt(void)
{
	INTC_register_interrupt(&tirq_int_handler, AVR32_TC_IRQ0, AVR32_INTC_INT0);
    //~ INTC_register_interrupt(&tirq_int_handler, HALL_GPIO_IRQ+HALL_1_PIN/8, AVR32_INTC_INT0);
}

//------------------------------------------------------------------------------
/*! \name Start function
 */
//! @{
void tirq_estimator_start(void)
{
	tc_start(&AVR32_TC, TC_CHANNEL_0);
    //~ gpio_enable_pin_pull_up(HALL_1_PIN );	// HALL_1_PIN
    //~ gpio_enable_pin_interrupt(HALL_1_PIN , GPIO_PIN_CHANGE);	// HALL_1_PIN
    tirq_ti = Get_sys_count();
    nieme = 0;
}

//------------------------------------------------------------------------------
/*! \name Stop function
 */
//! @{
void tirq_estimator_stop(void)
{
    tc_stop(&AVR32_TC, TC_CHANNEL_0);
    //~ gpio_disable_pin_interrupt(HALL_1_PIN);	// HALL_1_PIN

}
//@}


__attribute__((__interrupt__))
void tirq_int_handler(void)
{
	unsigned int sr;
	sr = tc_read_sr(&AVR32_TC, TC_CHANNEL_0);
	if(sr && (1 << AVR32_TC_CPCS)) {
		//~ gpio_tgl_gpio_pin(MLED0);	//TODO: Remove, debug code
		tirq_tj= Get_sys_count();
		tirq_demi_period = tirq_tj - tirq_ti;
		tirq_ti = tirq_tj; // arm for next period
		//~ gpio_clear_pin_interrupt_flag(HALL_1_PIN);   //PB0

		if (first_interrupt)
		{
			nieme=0;
			first_interrupt=0;
		} else {
			first_interrupt=1;
		}
	}

	//~ gpio_tgl_gpio_pin(MLED1);
}

void m_tc_init(void)
{
	static tc_waveform_opt_t waveform_opt =
	{
		.channel = TC_CHANNEL_0,
		.wavsel = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,
		.enetrg = FALSE,
		.eevt = 0,
		.eevtedg = TC_SEL_NO_EDGE,
		.cpcdis = FALSE,
		.cpcstop = FALSE,
		.burst = FALSE,
		.clki = FALSE,
		.tcclks = TC_CLOCK_SOURCE_TC5
	};

	//Configure timer, for interrupt
	tc_init_waveform(&AVR32_TC, &waveform_opt);
	tc_write_rc(&AVR32_TC, TC_CHANNEL_0, 960);	//TODO: RANDOM CONSTANT!
	//~ tc_start(&AVR32_TC, TC_CHANNEL_0);
}

void tirq_init(void)
{
	// Define which timer signals that will cause interrupts
	static const tc_interrupt_t TC_INTERRUPT_OPT =
	{
					// we interrupt on RC compare match with CV
		.cpcs = 1,	// RC compare status
	};

	Disable_global_interrupt();
	m_tc_init();
	INTC_init_interrupts();
	tirq_estimator_init_interrupt(); 	//~ was: INTC_register_interrupt(&tirq_int_handler, AVR32_TC_IRQ0, AVR32_INTC_INT0);
	tc_configure_interrupts(&AVR32_TC, TC_CHANNEL_0, &TC_INTERRUPT_OPT);
	Enable_global_interrupt();
}
