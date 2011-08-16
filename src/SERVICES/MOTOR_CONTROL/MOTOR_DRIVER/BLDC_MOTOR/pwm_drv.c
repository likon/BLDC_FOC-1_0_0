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
#include "pwm_drv.h"
#include "CONF/conf_motor_driver.h"
//~ #include "CONF/mc300.h"	//TODO: Remove

//_____ M A C R O S ________________________________________________________
//_____ D E F I N I T I O N S ______________________________________________
//_____ D E C L A R A T I O N S ____________________________________________
//------------------------------------------------------------------------------
/*! \name Initialization function
 */
//! @{
void pwm_drv_init(volatile pwm_drv_options_t *pwm_drv_options)
{
   volatile avr32_pwm_t *pwm = &AVR32_PWM;

  // select GPIO alternate function
   static const gpio_map_t PWM_GPIO_MAP  =
  {
    {PWM_XL_PIN_NUMBER, PWM_XL_PWM_FUNCTION},
    //~ {PWM_XH_PIN_NUMBER, PWM_XH_PWM_FUNCTION},
    {PWM_YL_PIN_NUMBER, PWM_YL_PWM_FUNCTION},
    //~ {PWM_YH_PIN_NUMBER, PWM_YH_PWM_FUNCTION},
    {PWM_ZL_PIN_NUMBER, PWM_ZL_PWM_FUNCTION},
    //~ {PWM_ZH_PIN_NUMBER, PWM_ZH_PWM_FUNCTION}
  };
  // Assign GPIO to PWM.
  gpio_enable_module(PWM_GPIO_MAP, sizeof(PWM_GPIO_MAP ) / sizeof(PWM_GPIO_MAP[0]));

  // set PWM mode register
  pwm->mr =
    (0<<AVR32_PWM_DIVA) |
    (0<<AVR32_PWM_DIVB) |
    (0<<AVR32_PWM_PREA) |
    (0<<AVR32_PWM_PREB) ;

  // channel XH_PWM_CHANNEL	//TODO: remove
  //~ pwm->channel[PWM_XH_PWM_CHANNEL].cmr=
    //~ (0<<AVR32_PWM_CMR_CPRE) | // MCK % 1
    //~ (1<<AVR32_PWM_CMR_CALG) | // center aligned
    //~ (0<<AVR32_PWM_CMR_CPOL) | // start with 0
    //~ (0<<AVR32_PWM_CMR_CPD)
    //~ ;   // channel mode
  //~ pwm->channel[PWM_XH_PWM_CHANNEL].cprd= pwm_drv_options->max_pwm_value; // channel period
  //~ pwm->channel[PWM_XH_PWM_CHANNEL].cdty= pwm_drv_options->max_pwm_value - 10; // duty cycle, should be < CPRD

  // channel XL_PWM_CHANNEL
  pwm->channel[PWM_XL_PWM_CHANNEL].cmr=
    (0<<AVR32_PWM_CMR_CPRE) | // MCK % 1
    (1<<AVR32_PWM_CMR_CALG) | // center aligned
    (1<<AVR32_PWM_CMR_CPOL) | // start with 1
    (0<<AVR32_PWM_CMR_CPD)
    ;   // channel mode
  pwm->channel[PWM_XL_PWM_CHANNEL].cprd= pwm_drv_options->max_pwm_value; // channel period
  pwm->channel[PWM_XL_PWM_CHANNEL].cdty= pwm_drv_options->max_pwm_value - 10; // duty cycle, should be < CPRD

  // channel YH_PWM_CHANNEL //TODO: remove
  //~ pwm->channel[PWM_YH_PWM_CHANNEL].cmr=
    //~ (0<<AVR32_PWM_CMR_CPRE) | // MCK % 1
    //~ (1<<AVR32_PWM_CMR_CALG) | // center aligned
    //~ (0<<AVR32_PWM_CMR_CPOL) | // start with 0
    //~ (0<<AVR32_PWM_CMR_CPD)
    //~ ;   // channel mode
  //~ pwm->channel[PWM_YH_PWM_CHANNEL].cprd= pwm_drv_options->max_pwm_value; // channel period
  //~ pwm->channel[PWM_YH_PWM_CHANNEL].cdty= pwm_drv_options->max_pwm_value - 10; // duty cycle, should be < CPRD

  // channel YL_PWM_CHANNEL
  pwm->channel[PWM_YL_PWM_CHANNEL].cmr=
    (0<<AVR32_PWM_CMR_CPRE) | // MCK % 1
    (1<<AVR32_PWM_CMR_CALG) | // center aligned
    (1<<AVR32_PWM_CMR_CPOL) | // start with 1
    (0<<AVR32_PWM_CMR_CPD)
    ;   // channel mode
  pwm->channel[PWM_YL_PWM_CHANNEL].cprd= pwm_drv_options->max_pwm_value; // channel period
  pwm->channel[PWM_YL_PWM_CHANNEL].cdty= pwm_drv_options->max_pwm_value - 10; // duty cycle, should be < CPRD

  // channel ZH_PWM_CHANNEL //TODO: remove
  //~ pwm->channel[PWM_ZH_PWM_CHANNEL].cmr=
    //~ (0<<AVR32_PWM_CMR_CPRE) | // MCK % 1
    //~ (1<<AVR32_PWM_CMR_CALG) | // center aligned
    //~ (0<<AVR32_PWM_CMR_CPOL) | // start with 0
    //~ (0<<AVR32_PWM_CMR_CPD)
    //~ ;   // channel mode
  //~ pwm->channel[PWM_ZH_PWM_CHANNEL].cprd= pwm_drv_options->max_pwm_value; // channel period
  //~ pwm->channel[PWM_ZH_PWM_CHANNEL].cdty= pwm_drv_options->max_pwm_value - 10; // duty cycle, should be < CPRD

  // channel ZL_PWM_CHANNEL
  pwm->channel[PWM_ZL_PWM_CHANNEL].cmr=
    (0<<AVR32_PWM_CMR_CPRE) | // MCK % 1
    (1<<AVR32_PWM_CMR_CALG) | // center aligned
    (1<<AVR32_PWM_CMR_CPOL) | // start with 1
    (0<<AVR32_PWM_CMR_CPD)
    ;   // channel mode
  pwm->channel[PWM_ZL_PWM_CHANNEL].cprd= pwm_drv_options->max_pwm_value; // channel period

  pwm->channel[PWM_ZL_PWM_CHANNEL].cdty= pwm_drv_options->max_pwm_value - 10; // duty cycle, should be < CPRD
}
void pwm_drv_start(void)
{
  volatile avr32_pwm_t *pwm = &AVR32_PWM;
  pwm->ier = (1<<PWM_XL_PWM_CHANNEL );
  pwm->ena = (1<<PWM_XL_PWM_CHANNEL )|(1<<PWM_YL_PWM_CHANNEL )|(1<<PWM_ZL_PWM_CHANNEL ); // enable channel 0 to 6
  //~ pwm->ena = (1<<PWM_XL_PWM_CHANNEL )|(1<<PWM_XH_PWM_CHANNEL )|(1<<PWM_YL_PWM_CHANNEL )|(1<<PWM_YH_PWM_CHANNEL )|(1<<PWM_ZL_PWM_CHANNEL )|(1<<PWM_ZH_PWM_CHANNEL ); // enable channel 0 to 6  //TODO: remove
}
void pwm_drv_stop(void)
{
}
//------------------------------------------------------------------------------
/*! \name PWM Driver Update
 */
//! @{
void pwm_drv_duty_cycle(volatile pwm_drv_options_t * pwm_drv_options,U32 pwm0,U32 pwm1,U32 pwm2)
//~ void pwm_drv_duty_cycle(volatile pwm_drv_options_t *pwm_drv_options,U32 pwm0,U32 pwm1,U32 pwm2,U32 pwm3,U32 pwm4,U32 pwm5) //TODO: remove
{
  volatile avr32_pwm_t *pwm = &AVR32_PWM;

  //~ pwm->channel[PWM_XH_PWM_CHANNEL].cupd= pwm_drv_options->max_pwm_value - pwm0; //TODO: remove comment
  pwm->channel[PWM_XL_PWM_CHANNEL].cupd= pwm_drv_options->max_pwm_value - pwm0;	//was -pwm1
  //~ pwm->channel[PWM_YH_PWM_CHANNEL].cupd= pwm_drv_options->max_pwm_value - pwm2; //TODO: remove comment
  pwm->channel[PWM_YL_PWM_CHANNEL].cupd= pwm_drv_options->max_pwm_value - pwm1;	//was -pwm3
  //~ pwm->channel[PWM_ZH_PWM_CHANNEL].cupd= pwm_drv_options->max_pwm_value - pwm4; //TODO: remove comment
  pwm->channel[PWM_ZL_PWM_CHANNEL].cupd= pwm_drv_options->max_pwm_value - pwm2;	//was -pwm5
}
//@}

