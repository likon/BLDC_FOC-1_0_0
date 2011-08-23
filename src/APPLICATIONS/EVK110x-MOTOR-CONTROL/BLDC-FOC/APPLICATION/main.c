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
#include "board.h"
#include "gpio.h"
#include "flashc.h"
#include "pm.h"
#include "gpio.h"
#include "cycle_counter.h"
#include "intc.h"
#include "tc.h"
#include "tirq.h"
#include "adc.h"
#include "compiler.h"

#include "mc_driver.h"
#include "mc_control.h"
#include "conf_motor_driver.h"

#include "math.h"

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
               7,   // unsigned int mul
               1,   // unsigned int div, Sel Osc0/PLL0 or Osc1/Pll1
               0,   // unsigned int osc
               16); // unsigned int lockcount

  pm_pll_set_option(pm, 0, 1, 1, 0);//Max 60MHz, TODO: Update FCPU_HZ in foc.h

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
int main (void)
{
  // Configure standard I/O streams as unbuffered.
#if __GNUC__ && __AVR32__
  setbuf(stdin, NULL);
#endif
  setbuf(stdout, NULL);

#ifdef USB_DEBUG
  init_usb();

   // Wait enumeration
   do
   {
     usb_task();
   }while(!Is_device_enumerated());

#endif

	//Initialize timer interrupt
	//~ tirq_init();

   // Initialize control task
   mc_control_task_init();

   // Initialize direction
   mc_set_motor_direction(MC_CW);

	//Enable the motor driver circuit
	gpio_set_gpio_pin(MOTEN);
	gpio_set_gpio_pin(STBINV);
	gpio_tgl_gpio_pin(PWM_YL_PIN_NUMBER);
	gpio_enable_module_pin(PWM_XL_PIN_NUMBER, PWM_XL_PWM_FUNCTION);
	gpio_enable_module_pin(PWM_YL_PIN_NUMBER, PWM_YL_PWM_FUNCTION);
	gpio_enable_module_pin(PWM_ZL_PIN_NUMBER, PWM_ZL_PWM_FUNCTION);

//-----------
int i;
double phi;
int si1,si2,si3;

	AVR32_PWM.mr = 1|(1<<16)|(3<<24)|(3<<8);	//clka,b ohne div, MCLK/8
	for(i=0; i<3; i++){
	  AVR32_PWM.channel[i].cmr= 2|(1<<8);   // Channel mode. MCLK/4 center aligned
	  AVR32_PWM.channel[i].cdty= 180; // Duty cycle, should be < CPRD.
	  AVR32_PWM.channel[i].cprd= 256; // Channel period.
	}

	AVR32_PWM.ena = 0x07;	//channel 0..2 enable
	delay_init(48000000UL);
#define OFFSET 127
#define AMP 30.0
#define DPHI 0.03
	while(1){
		phi+=DPHI;
		si1=OFFSET+(int)(AMP*sin(phi));
		si2=OFFSET+(int)(AMP*sin(phi+2.0943));	//120°
		si3=OFFSET+(int)(AMP*sin(phi+4.1888));	//240°
		AVR32_PWM.channel[0].cdty=si1;
		AVR32_PWM.channel[1].cdty=si2;
		AVR32_PWM.channel[2].cdty=si3;
		//delay_ms(1);
		//for(i = 0; i < 0x8000; i++) {
		//	asm volatile("nop");
		//}

	}
//------------------

   while(1)
   {
#ifdef USB_DEBUG
#error We are not supposed to use USB
      usb_task();
      device_cdc_task();
#endif
      mc_control_task();
   }
}

