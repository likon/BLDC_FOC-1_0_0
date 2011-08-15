/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief HALL ESTIMATOR for AVR32 UC3.
 *
 * AVR32 HALL ESTIMATOR module.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices with a TC module can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 ******************************************************************************/

/* Copyright (C) 2006-2008, Atmel Corporation All rights reserved.
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

//_____  I N C L U D E S ___________________________________________________

#include <avr32/io.h>
#include "board.h"
#include "compiler.h"
#include "intc.h"
#include "gpio.h"
#include "cycle_counter.h"
#include "hall_estimator.h"
#include "CONF/conf_foc.h"
#include "CONF/mc300.h"
//_____ M A C R O S ________________________________________________________


//_____ D E F I N I T I O N S ______________________________________________


//_____ D E C L A R A T I O N S ____________________________________________

volatile unsigned short first_interrupt=0;
volatile unsigned short nieme=0;
volatile U32 hall_demi_period=hall_demi_period_init ;
volatile unsigned short teta0=0;
volatile U32 hall_tj;
volatile U32 hall_ti;                //!<  last hall period value

void hall_estimator_update_teta_and_speed(volatile unsigned short *teta_elec, volatile unsigned short *vitesse_elec)
{
   nieme++;
   // 180*Fcpu*100e-6
  *teta_elec=(unsigned short)((U32)(vitesse_inst*1*nieme)/(U32)hall_demi_period);
  if(*teta_elec>360)
  {
    *teta_elec=360;
    nieme=0; 
  }
  *vitesse_elec=pi_Fcpu/hall_demi_period;  //pi*Fcpu  (Fcpu=48Mhz)      
}

// interrupt handler due à l'interruption generée par  HALL
#if __GNUC__
__attribute__((__interrupt__)) void hall_int_handler( void )
#elif __ICCAVR32__
/* TC Interrupt  */
__interrupt void hall_int_handler( void )
#endif
{	
   // determine hall period
  if(gpio_get_pin_interrupt_flag(HALL_1_PIN))
  {
     hall_tj= Get_sys_count();
     hall_demi_period = hall_tj - hall_ti;
     hall_ti = hall_tj; // arm for next period     
     gpio_clear_pin_interrupt_flag(HALL_1_PIN);   //PB0
  
      if (first_interrupt)
      {
        nieme=0;
        first_interrupt=0;
      }
      else
      {
        first_interrupt=1;
      }
   }
}
void hall_estimator_init_teta(volatile unsigned short teta)
{
    nieme = (int)(((teta*hall_demi_period)/vitesse_inst)+2);
}
//------------------------------------------------------------------------------
/*! \name Initialization function
 */
//! @{
void hall_estimator_init(void)
{
    nieme = 0;
}

//------------------------------------------------------------------------------
/*! \name Interrupt intitialization function
 */
//! @{
void hall_estimator_init_interrupt(void)
{
    INTC_register_interrupt(&hall_int_handler, HALL_GPIO_IRQ+HALL_1_PIN/8, AVR32_INTC_INT0);
}

//------------------------------------------------------------------------------
/*! \name Start function
 */
//! @{
void hall_estimator_start(void)
{
    gpio_enable_pin_pull_up(HALL_1_PIN );	// HALL_1_PIN 
    gpio_enable_pin_interrupt(HALL_1_PIN , GPIO_PIN_CHANGE);	// HALL_1_PIN
    hall_ti = Get_sys_count();
    nieme = 0;
}

//------------------------------------------------------------------------------
/*! \name Stop function
 */
//! @{
void hall_estimator_stop(void)
{
    gpio_disable_pin_interrupt(HALL_1_PIN);	// HALL_1_PIN
}
//@}
