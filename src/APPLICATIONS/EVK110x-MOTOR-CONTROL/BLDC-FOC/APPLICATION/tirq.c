/**********************************************************************
 * File: tirq.c
 * Author: Marcus Jansson
 * Copyright (C) 2011 Marcus Jansson <mjansson256@gmail.com>
 * and Thomas Eberharter, Eberharter Elektronik GmbH
 *********************************************************************/

#include "tirq.h"


#define TC_CHANNEL_0 0

//TODO: Remove MLED and BUT definitions and relevant debug code
#define MLED0 AVR32_PIN_PB27
#define MLED1 AVR32_PIN_PB28
#define MLED2 AVR32_PIN_PB29
#define MLED3 AVR32_PIN_PB30

#define BUT0 AVR32_PIN_PX16
#define BUT1 AVR32_PIN_PX19
#define BUT2 AVR32_PIN_PX22

__attribute__((__interrupt__))
void tc_irq(void)
{
	tc_read_sr(&AVR32_TC, TC_CHANNEL_0);
	gpio_tgl_gpio_pin(MLED1);
}

void m_tc_init(void)
{
	//~ volatile avr32_pm_t * pm = &AVR32_PM;
	//~ volatile avr32_tc_t * tc = &AVR32_TC;

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
	tc_write_rc(&AVR32_TC, TC_CHANNEL_0, 0xffff);
	tc_start(&AVR32_TC, TC_CHANNEL_0);
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
	INTC_register_interrupt(&tc_irq, AVR32_TC_IRQ0, AVR32_INTC_INT0);
	tc_configure_interrupts(&AVR32_TC, TC_CHANNEL_0, &TC_INTERRUPT_OPT);
	Enable_global_interrupt();
}
