/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief DC Motor driver for AVR32 UC3.
 *
 * AVR32 DC Motor driver module.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices with a TC module can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *                       Adapted by Marcus Jansson for Eberharter Elektronik, GmbH
 *****************************************************************************/

/* Copyright (c) 2007, Atmel Corporation All rights reserved.
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
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/



//_____  I N C L U D E S ___________________________________________________

#include <avr32/io.h>
#include "compiler.h"
#include "board.h"
#include "gpio.h"
#include "CONF/conf_motor_driver.h"
#include "CONF/conf_foc.h"
#include "pwm_drv.h"

//_____ M A C R O S ________________________________________________________
//_____ D E F I N I T I O N S ______________________________________________
//_____ D E C L A R A T I O N S ____________________________________________
//------------------------------------------------------------------------------


/*! \name Initialization function
 */
//! @{
void pwm_drv_init(volatile pwm_drv_options_t *pwm_drv_options)
{
/*
  volatile avr32_pwma_t *pwm = &AVR32_PWMA;

  // select GPIO alternate function
   static const gpio_map_t PWM_GPIO_MAP  =
  {
    { PWM_XL_PIN_NUMBER, PWM_XL_PWM_FUNCTION },
    { PWM_YL_PIN_NUMBER, PWM_YL_PWM_FUNCTION },
    { PWM_ZL_PIN_NUMBER, PWM_ZL_PWM_FUNCTION },
  };

  // Assign GPIO to PWM.
  gpio_enable_module(PWM_GPIO_MAP, sizeof(PWM_GPIO_MAP ) / sizeof(PWM_GPIO_MAP[0]));

  // set PWM mode register
  pwm->mr =
    (0<<AVR32_PWM_DIVA) |
    (0<<AVR32_PWM_DIVB) |
    (0<<AVR32_PWM_PREA) |
    (0<<AVR32_PWM_PREB) ;

  // channel XL_PWM_CHANNEL
  pwm->channel[PWM_XL_PWM_CHANNEL].cmr=
    (0<<AVR32_PWM_CMR_CPRE) | // MCK % 1
    (1<<AVR32_PWM_CMR_CALG) | // center aligned
    (1<<AVR32_PWM_CMR_CPOL) | // start with 1
    (0<<AVR32_PWM_CMR_CPD)
    ;   // channel mode
  pwm->channel[PWM_XL_PWM_CHANNEL].cprd= pwm_drv_options->max_pwm_value; // channel period
  pwm->channel[PWM_XL_PWM_CHANNEL].cdty= pwm_drv_options->max_pwm_value - 10; // duty cycle, should be < CPRD //~ TODO: -10 is a random value. Should be changed to something more descriptive.

  // channel YL_PWM_CHANNEL
  pwm->channel[PWM_YL_PWM_CHANNEL].cmr=
    (0<<AVR32_PWM_CMR_CPRE) | // MCK % 1
    (1<<AVR32_PWM_CMR_CALG) | // center aligned
    (1<<AVR32_PWM_CMR_CPOL) | // start with 1
    (0<<AVR32_PWM_CMR_CPD)
    ;   // channel mode
  pwm->channel[PWM_YL_PWM_CHANNEL].cprd= pwm_drv_options->max_pwm_value; // channel period
  pwm->channel[PWM_YL_PWM_CHANNEL].cdty= pwm_drv_options->max_pwm_value - 10; // duty cycle, should be < CPRD

  // channel ZL_PWM_CHANNEL
  pwm->channel[PWM_ZL_PWM_CHANNEL].cmr=
    (0<<AVR32_PWM_CMR_CPRE) | // MCK % 1
    (1<<AVR32_PWM_CMR_CALG) | // center aligned
    (1<<AVR32_PWM_CMR_CPOL) | // start with 1
    (0<<AVR32_PWM_CMR_CPD)
    ;   // channel mode
  pwm->channel[PWM_ZL_PWM_CHANNEL].cprd= pwm_drv_options->max_pwm_value; // channel period

  pwm->channel[PWM_ZL_PWM_CHANNEL].cdty= pwm_drv_options->max_pwm_value - 10; // duty cycle, should be < CPRD
*/
}

void pwm_drv_start(void)
{
  volatile avr32_pwma_t *pwm = &AVR32_PWMA;
  pwm->ier = (1<<PWM_XL_PWM_CHANNEL );

  #if BOARD != PMSM
  pwm->ena = (1<<PWM_XL_PWM_CHANNEL )|(1<<PWM_YL_PWM_CHANNEL )|(1<<PWM_ZL_PWM_CHANNEL ); // enable channel 0 to 3
  #endif
  //TODO: BOARD == PMSM

}

void pwm_drv_stop(void)
{
}

//------------------------------------------------------------------------------
/*! \name PWM Driver Update
 */
//! @{
void pwm_drv_duty_cycle(volatile pwm_drv_options_t * pwm_drv_options, U32 pwm0, U32 pwm1, U32 pwm2)
{
	//TODO: This code should be looked over together with the code in svpwm.c
	#define MAX PWM_PERIOD
	//Divide to get into PWM period range
	pwm0 /= 2;
	pwm1 /= 2;
	pwm2 /= 2;
	//Make sure we dont write 0 or MAX value into the register, just as a precaution to avoid potential problems. Should be removed later.
	pwm0 |=1;
	pwm1 |=1;
	pwm2 |=1;
	if(pwm0 > MAX)  { pwm0 = MAX-1; }
	if(pwm1 > MAX)  { pwm1 = MAX-1; }
	if(pwm2 > MAX)  { pwm2 = MAX-1; }

#ifdef DEBUG
	//~ printf("%i, %i, %i\n\r", pwm0, pwm1, pwm2);
	//return; //TODO: REMOVE!!!! Debug code
#endif

  //~ volatile avr32_pwm_t *pwm = &AVR32_PWM; //TODO: !!!! Fix!!!!

  //~ pwm->channel[PWM_XL_PWM_CHANNEL].cupd= pwm_drv_options->max_pwm_value - pwm0;
  //~ pwm->channel[PWM_YL_PWM_CHANNEL].cupd= pwm_drv_options->max_pwm_value - pwm1;
  //~ pwm->channel[PWM_ZL_PWM_CHANNEL].cupd= pwm_drv_options->max_pwm_value - pwm2;
}

//@}

