/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief BLDC FOC Motor Control application for AVR32 UC3.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices with a USART module can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *                       Adapted by Marcus Jansson for Eberharter Elektronik, GmbH
 *
 ******************************************************************************/

/*! \page License
 * Copyright (C) 2006-2008, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 * \mainpage
 * \section section1 Description
 *   This application is demonstrate a sensor FOC algorithm on a BLDC Motor
 *   through a real-time motor control application. \n
 *   It runs on an EVK1101 board \n
 *   The main source files are :
 *
 *          - \ref main.c  Main loop
 *
 * \section section2 Configuration
 * The configuration of the application is stored in different files :
 *
 *          - \ref conf_foc.h                 Contains the configuration of the application
 *
 * \section section3 Main Peripheral Use
 *   - Six PWM channels are used to control motor
 *   - Three GPIO are used for hall sensors acquisition
 *   - USB is used for communication with the PC GUI
 *
 * \section section4 User Manual
 *   - See the associated application note available at http://www.atmel.com
 *
 * \subsection subsection1 List of Material
 *   - EVK1101 (UC3B)
 *
 * \section section5 Supported Compiler
 *  - AVR32-GCC 4.2.2 - atmel 1.1.2 with AVR32 GNU TOOLCHAIN 1.4.0
 *  - IAR C/C++ Compiler for Atmel AVR32 3.10A/W32 (3.10.1.3)
 */
#include <stddef.h>
#include <stdio.h>
#include <math.h>

#include "avr32/io.h"

#if BOARD == PMSM
#include "adcifb.h"
#include "pm_uc3l.h"
#else
#include "adc.h"
#include "pm.h"
#endif

#include "board.h"
#include "compiler.h"
#include "conf_foc.h"
//~ #include "cycle_counter.h"
//~ #include "flashc.h"
#include "gpio.h"
#include "intc.h"

#include "tc.h"
#include "tirq.h"

#include "mc_driver.h"
#include "mc_control.h"
#include "conf_motor_driver.h"
#include "motor_startup.h"

//~ #include "math.h"	//TODO: REMOVE redundant
#include "usart.h"

#if __GNUC__ && __AVR32__
  #include "nlao_cpu.h"
  #include "nlao_usart.h"
#elif __ICCAVR32__
  #include "usart.h"
#endif

//~ #include "conf_usb.h"
//~ #include "usb_task.h"
//~ #include "device_cdc_task.h"
//~ #include "usb_standard_request.h"	//TODO: REMOVE

int pic_main ( void );
void test_pwm(void);	//TODO: Move/remove test functions
void test_init_motor_drive(void);
void test_gpio(void);
void test_usart(void);
void test_thomas_program(void);

/*! Simple test of motor output driver */
void test_init_motor_drive(void)
{
	#if BOARD != PMSM
	gpio_set_gpio_pin(MOTEN);
	gpio_set_gpio_pin(STBINV);
	#else
	gpio_set_gpio_pin(OUT_EN);
	#endif
	//~ gpio_tgl_gpio_pin(PWM_YL_PIN_NUMBER);
	//~ gpio_enable_module_pin(PWM_XL_PIN_NUMBER, PWM_XL_PWM_FUNCTION);
	//~ gpio_enable_module_pin(PWM_YL_PIN_NUMBER, PWM_YL_PWM_FUNCTION);
	//~ gpio_enable_module_pin(PWM_ZL_PIN_NUMBER, PWM_ZL_PWM_FUNCTION);
	mc_global_init();
	mc_lowlevel_start();
}

/*! Simple functional test of PWM signal, for debugging */
void pwm_test(void)
{
	test_init_motor_drive();
	while(1);
}

/*! Simple test of GPIO pins, for debugging */
void test_gpio(void)
{
	//~ gpio_set_gpio_pin(MOTEN); //Activate motor driver IC output
	//~ gpio_set_gpio_pin(STBINV);
	//Initialize timer interrupt
	tirq_init();
	tirq_estimator_start();
	//~ tc_start(&AVR32_TC, TC_CHANNEL_0);
	while(1) {
		//~ gpio_tgl_gpio_pin(AVR32_PIN_PB19);
		//~ gpio_tgl_gpio_pin(AVR32_PIN_PB20);
		//gpio_tgl_gpio_pin(AVR32_PIN_PB21);
	}
}

/*! Simple USART test */
void test_usart(void)
{
	//Direct output
	usart_write_line(&AVR32_USART0, "Hello serial!\n\r");
}

extern volatile unsigned short tick;

/*! \brief Initializes MCU exceptions.
 */
static void init_exceptions(void)
{
#if __GNUC__ && __AVR32__
  // Import the Exception Vector Base Address.
  extern void _evba;

  // Load the Exception Vector Base Address in the corresponding system
  // register.
  Set_system_register(AVR32_EVBA, (int)&_evba);
#endif

  // Enable exceptions globally.
  Enable_global_exception();
}


/*! \brief Initializes the HSB bus matrix.
 */
static void init_hmatrix(void)
{
  // For the internal-flash HMATRIX slave, use last master as default.
  union
  {
    unsigned long                 scfg;
    avr32_hmatrix_scfg_t          SCFG;
  } u_avr32_hmatrix_scfg = {AVR32_HMATRIX.scfg[AVR32_HMATRIX_SLAVE_FLASH]};
  u_avr32_hmatrix_scfg.SCFG.defmstr_type = AVR32_HMATRIX_DEFMSTR_TYPE_LAST_DEFAULT;
  AVR32_HMATRIX.scfg[AVR32_HMATRIX_SLAVE_FLASH] = u_avr32_hmatrix_scfg.scfg;
}


/*! \brief Initializes the MCU system clocks.
 */
static void init_sys_clocks(void)
{
	#if BOARD != PMSM
volatile avr32_pm_t* pm = &AVR32_PM;

  /* Switch the main clock to OSC0 */
  pm_switch_to_osc0(pm, FOSC0, OSC0_STARTUP);
  /* Setup PLL0 on OSC0 */
  pm_pll_setup(pm,  // volatile avr32_pm_t* pm
               0,   // unsigned int pll
               11,  // unsigned int mul
               1,   // unsigned int div,
               0,   // unsigned int osc, Sel Osc0/PLL0 or Osc1/PLL1
               16); // unsigned int lockcount

  pm_pll_set_option(pm, 0, 1, 1, 0);	//Max 60MHz, TODO: Update FCPU_HZ in usb11.h
  /* Enable PLL0 */
  pm_pll_enable(pm,0);
  /* Wait for PLL0 locked */
  pm_wait_for_pll0_locked(pm) ;
  /* set divider to 4 (CPU = CLK / 2; PBA = CPU / 2) for PBA bus */
  pm_cksel(pm,1,0,0,0,0,0);
  /* switch to clock */
  pm_switch_to_clock(pm, AVR32_PM_MCCTRL_MCSEL_PLL0);

#if __GNUC__ && __AVR32__
  // Give the used PBA clock frequency to Newlib, so it can work properly.
  set_cpu_hz(FPBA_HZ);
#endif
#endif
}


/*! \brief Initializes MCU interrupts.
 */
static void init_interrupts(void)
{
  // Initialize interrupt handling.
  INTC_init_interrupts();

  // Enable interrupts globally.
  Enable_global_interrupt();
}


/*! \brief Low-level initialization routine called during startup, before the
 *         \ref main function.
 */
#if __GNUC__ && __AVR32__
int _init_startup(void)
#elif __ICCAVR32__
int __low_level_init(void)
#endif
{
  init_exceptions();
  init_hmatrix();
  init_sys_clocks();
  init_interrupts();

  // EWAVR32: Request initialization of data segments.
  // GCC: Don't-care value.
  return 1;
}

/*! Initialize debug USART */
void m_usart_init(void)
{
	/* Set up USART for debug output */
	#define USART_RX STDIO_USART_RX_PIN
	#define USART_TX STDIO_USART_TX_PIN

	volatile avr32_gpio_port_t * gpio;
	gpio = (avr32_gpio_port_t *)&AVR32_GPIO;
	gpio->gperc = 1 << (USART_RX & 0x1f);
	gpio->gperc = 1 << (USART_TX & 0x1f);

	usart_options_t usart_opt = {
		.charlength = 8,
		.paritytype = USART_NO_PARITY,
		.stopbits = USART_1_STOPBIT,
		.channelmode = USART_NORMAL_CHMODE,
		.baudrate = STDIO_USART_BAUDRATE
	};

	usart_init_rs232(&AVR32_USART0, &usart_opt, 2 * FPBA_HZ);
	set_usart_base((void *) &AVR32_USART0);
	printf("Usart initialized!\n\r");
}

int main(void)
{
	Disable_global_interrupt();
	while(1);
  // Configure standard I/O streams as unbuffered.
	#if __GNUC__ && __AVR32__
	setbuf(stdin, NULL);
	#endif
	setbuf(stdout, NULL);
	m_usart_init();	//TODO: REMOVE

	//Enable the motor driver circuit
	#if BOARD == PMSM
	gpio_set_gpio_pin(OUT_EN);
	#else
	gpio_set_gpio_pin(MOTEN);
	gpio_set_gpio_pin(STBINV);
	#endif
	gpio_tgl_gpio_pin(PWM_YL_PIN_NUMBER);
	gpio_enable_module_pin(PWM_XL_PIN_NUMBER, PWM_XL_PWM_FUNCTION);
	gpio_enable_module_pin(PWM_YL_PIN_NUMBER, PWM_YL_PWM_FUNCTION);
	gpio_enable_module_pin(PWM_ZL_PIN_NUMBER, PWM_ZL_PWM_FUNCTION);

//-----===
	mc_global_init();
	mc_lowlevel_start();
	//~ while(1);
//-----===


   // Initialize control task
   mc_control_task_init();

   // Initialize direction
   mc_set_motor_direction(MC_CW);

	//~ motor_startup();
	//~ pwm_test();

#if BOARD == USB11
	gpio_enable_pin_pull_up(J13_10);
	gpio_enable_pin_pull_up(J13_11);
	gpio_enable_pin_pull_up(J13_12);
	gpio_enable_pin_pull_up(J13_13);
#elseif BOARD == PMSM
	gpio_enable_pin_pull_up(J4_1);
	gpio_enable_pin_pull_up(J4_2);
#endif

	//~ gpio_tgl_gpio_pin(J13_11);
	//~ gpio_tgl_gpio_pin(J13_11);

	//~ pic_main();
	//~ while(1);

	//~ tirq_init();
   while(1)
   {
	//~ if(gpio_get_pin_value(J13_13) != 0) {
		//~ gpio_tgl_gpio_pin(J13_10);
		mc_control_task();
	//~ }
   }
}