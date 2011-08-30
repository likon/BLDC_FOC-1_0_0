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
#include "adc.h"
#include "board.h"
#include "compiler.h"
#include "cycle_counter.h"
#include "flashc.h"
#include "gpio.h"
#include "intc.h"
#include "pm.h"
#include "tc.h"
#include "tirq.h"

#include "mc_driver.h"
#include "mc_control.h"
#include "conf_motor_driver.h"
#include "motor_startup.h"

#include "math.h"
#include "usart.h"

#if __GNUC__ && __AVR32__
  #include "nlao_cpu.h"
  #include "nlao_usart.h"
#elif __ICCAVR32__
  #include "usart.h"
#endif

#include "conf_usb.h"
#include "usb_task.h"
#include "device_cdc_task.h"
#include "conf_foc.h"
#include "usb_standard_request.h"

int pic_main ( void );
void test_pwm(void);	//TODO: Move/remove test functions
void test_init_motor_drive(void);
void test_gpio(void);
void test_usart(void);
void test_thomas_program(void);

/*! Simple test program supplied by Thomas */
void test_thomas_program(void)
{
	int i;
	double phi;
	int si1,si2,si3;
	volatile unsigned short adc_value_ia;
	volatile unsigned short adc_value_ib;
	volatile unsigned short adc_value_ic;

	AVR32_PWM.mr = 1|(1<<16)|(3<<24)|(3<<8);	//clka,b ohne div, MCLK/8
	for(i=0; i<3; i++){
	  AVR32_PWM.channel[i].cmr= 2|(1<<8);   // Channel mode. MCLK/4 center aligned
	  AVR32_PWM.channel[i].cdty= 50; // Duty cycle, should be < CPRD.
	  AVR32_PWM.channel[i].cprd= 80; // Channel period.
	}

	AVR32_PWM.ena = 0x07;	//channel 0..2 enable
	//~ delay_init(FCPU_HZ);
	//~ #define OFFSET 127
	#define OFFSET 40
	#define AMP 30.0
	#define DPHI 0.03
	float dphi = DPHI;
	unsigned int a = 0;

	while(1){
		phi += dphi;
		if(a++ == 0xff) {
			a = 0;
			if(dphi >= 0.3) {

			} else {
				dphi += 0.001;
			}
			//~ printf("dphi = %e\n\r", dphi);
		}
		si1=OFFSET+(int)(AMP*sin(phi));
		si2=OFFSET+(int)(AMP*sin(phi+2.0943));	//120°
		si3=OFFSET+(int)(AMP*sin(phi+4.1888));	//240°
		AVR32_PWM.channel[0].cdty=si1;
		AVR32_PWM.channel[1].cdty=si2;
		AVR32_PWM.channel[2].cdty=si3;
		adc_value_ia = mc_get_ia();
		adc_value_ib = mc_get_ib();
		adc_value_ic = mc_get_ic();

	//~ printf("si1 = %i\n\r", si1);
	#ifdef DEBUG
	//~ printf("ia = 0x%04x\n\r", adc_value_ia);
	//~ printf("ib = 0x%04x\n\r", adc_value_ib);
	//~ printf("ic = 0x%04x\n\r", adc_value_ic);
	#endif
	}
}

/*! Simple test of motor output driver */
void test_init_motor_drive(void)
{
	gpio_set_gpio_pin(MOTEN);
	gpio_set_gpio_pin(STBINV);
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
	//Output via printf
	printf("Hello Kitty!\n\r");
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

  pm_pll_set_option(pm, 0, 1, 1, 0);//Max 60MHz, TODO: Update FCPU_HZ in usb11.h

//~ // Supported frequencies:
//~ // Fosc0 mul div PLL div2_en cpu_f pba_f   Comment
//~ //  12   15   1  192     1     12    12
//~ //  12    9   3   40     1     20    20    PLL out of spec
//~ //  12   15   1  192     1     24    12
//~ //  12    9   1  120     1     30    15
//~ //  12    9   3   40     0     40    20    PLL out of spec
//~ //  12   15   1  192     1     48    12
//~ //  12   15   1  192     1     48    24
//~ //  12    8   1  108     1     54    27
//~ //  12    9   1  120     1     60    15   <--- We set up PLL0 according to this, mjan 20101228
//~ //  12    9   1  120     1     60    30
//~ //  12   10   1  132     1     66    16.5
//~ /*!
 //~ * \brief This function will setup a PLL.
 //~ * \param pm Base address of the Power Manager (i.e. &AVR32_PM)
 //~ * \param pll PLL number(0 for PLL0, 1 for PLL1)
 //~ * \param mul PLL MUL in the PLL formula
 //~ * \param div PLL DIV in the PLL formula
 //~ * \param osc OSC number (0 for osc0, 1 for osc1)
 //~ * \param lockcount PLL lockount
 //~ */
//~ extern void pm_pll_setup(volatile avr32_pm_t *pm,
//~ unsigned int pll,
//~ unsigned int mul,
//~ unsigned int div,
//~ unsigned int osc,
//~ unsigned int lockcount);

/*!
 //~ * \brief This function will set a PLL option.
 //~ * \param pm Base address of the Power Manager (i.e. &AVR32_PM)
 //~ * \param pll PLL number(0 for PLL0, 1 for PLL1)
 //~ * \param pll_freq Set to 1 for VCO frequency range 80-180MHz, set to 0 for VCO frequency range 160-240Mhz.
 //~ * \param pll_div2 Divide the PLL output frequency by 2 (this settings does not change the FVCO value)
 //~ * \param pll_wbwdisable 1 Disable the Wide-Bandith Mode (Wide-Bandwith mode allow a faster startup time and out-of-lock time). 0 to enable the Wide-Bandith Mode.
 //~ */
//~ extern void pm_pll_set_option(volatile avr32_pm_t *pm,
								//~ unsigned int pll,
								//~ unsigned int  pll_freq,
								//~ unsigned int  pll_div2,
								//~ unsigned int  pll_wbwdisable);
/*!
 * \brief This function will select all the power manager clocks.
 * \param pm Base address of the Power Manager (i.e. &AVR32_PM)
 * \param pbadiv Peripheral Bus A clock divisor enable
 * \param pbasel Peripheral Bus A select
 * \param pbbdiv Peripheral Bus B clock divisor enable
 * \param pbbsel Peripheral Bus B select
 * \param hsbdiv High Speed Bus clock divisor enable (CPU clock = HSB clock)
 * \param hsbsel High Speed Bus select (CPU clock = HSB clock )
 */
//~ extern void pm_cksel(volatile avr32_pm_t *pm,
								//~ unsigned int pbadiv,
								//~ unsigned int pbasel,
								//~ unsigned int pbbdiv,
								//~ unsigned int pbbsel,
								//~ unsigned int hsbdiv,
								//~ unsigned int hsbsel);

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

void init_usb(void)
{
  // Initialize USB clock
  pm_configure_usb_clock();

  // Initialize USB task
  usb_task_init();

  // Initialize device CDC USB task
  device_cdc_task_init();

}
int main(void)
{
  // Configure standard I/O streams as unbuffered.
	#if __GNUC__ && __AVR32__
	setbuf(stdin, NULL);
	#endif
	setbuf(stdout, NULL);

	/* Set up USART for debug output */
	#define USART_RX STDIO_USART_RX_PIN	//AVR32_USART0_RXD_0_0_PIN
	#define USART_TX STDIO_USART_TX_PIN	//AVR32_USART0_TXD_0_0_PIN

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

	usart_init_rs232(&AVR32_USART0, &usart_opt, FPBA_HZ);
	set_usart_base((void *) &AVR32_USART0);
	printf("Hello Kitty!\n\r");

#ifdef USB_DEBUG
  init_usb();

   // Wait enumeration
   do
   {
     usb_task();
   }while(!Is_device_enumerated());

#endif

	//Enable the motor driver circuit
	gpio_set_gpio_pin(MOTEN);
	gpio_set_gpio_pin(STBINV);
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

	gpio_enable_pin_pull_up(J13_10);
	gpio_enable_pin_pull_up(J13_11);
	gpio_enable_pin_pull_up(J13_12);
	gpio_enable_pin_pull_up(J13_13);
	//~ gpio_tgl_gpio_pin(J13_11);
	//~ gpio_tgl_gpio_pin(J13_11);

	//~ pic_main();
	//~ while(1);

	//~ tirq_init();
   while(1)
   {
#ifdef USB_DEBUG
#error We are not supposed to use USB
      usb_task();
      device_cdc_task();
#endif

	//~ if(gpio_get_pin_value(J13_13) != 0) {
		gpio_tgl_gpio_pin(J13_10);
		mc_control_task();
	//~ }
   }
}
