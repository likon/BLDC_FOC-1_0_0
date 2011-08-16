/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file ******************************************************************
 *
 * \brief Motor Control file with PWM peripheral
 *
 * This file contains the driver for Motor Control.
 *   It uses the PWM interface
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices can be used.
 * - AppNote:            AVR32 - Using the AVR32 FAT
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support email: avr32@atmel.com
 *                       Adapted by Marcus Jansson for Eberharter Elektronik, GmbH
 *
 ***************************************************************************/

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
#include "flashc.h"
#include "pm.h"
#include "cycle_counter.h"
#include "tc.h"
#include "gpio.h"
#include "adc.h"
#include "intc.h"
#include "pwm_drv.h"
#include "uart_usb_lib.h"
#include "usb_standard_request.h"
#include "mc_control.h"
#include "mc_driver.h"
//~ #include "hall_estimator.h" //TODO: Remove
#include "tirq.h"	//TODO: Rename? tirq_estimator?
#include "CONF/conf_foc.h"
#include "CONF/conf_motor_driver.h"
//~ #include "CONF/mc300.h"
//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

//_____ D E C L A R A T I O N S ____________________________________________

//!< Current pwm value
volatile pwm_drv_options_t pwm_drv_options;
//!< ADC Peripheral
volatile avr32_adc_t *adc = &AVR32_ADC;
//!< ADC Channel for IA
volatile unsigned short adc_channel_ia;
//!< ADC Channel for IB
volatile unsigned short adc_channel_ib;
//!< ADC Channel for IC
volatile unsigned short adc_channel_ic;


//! Output parameters.
static struct
{
  void (*motor_appli_callback)(void);
} motor_control_params =
{
  .motor_appli_callback     = NULL
};

extern volatile MC_BLDC_motor_t MC_BLDC_motor;
extern volatile unsigned short FOC_tick_speed;

volatile int g_mc_tick=0;

//------------------------------------------------------------------------------
/*! \name Tick Handler
 */
#if __GNUC__
__attribute__((__interrupt__)) void pwm_int_handler( void )
#elif __ICCAVR32__
__interrupt void pwm_int_handler( void )
#endif
{
    // Clear Interrupt Handler
    AVR32_PWM.isr;
    if (motor_control_params.motor_appli_callback!=NULL){
        if (MC_BLDC_motor.mc_state == RUN)
        {
          // Call application
          motor_control_params.motor_appli_callback();
          g_mc_tick++;
          if (g_mc_tick == TICK_SPEED_FACTOR) { g_mc_tick = 0 ; FOC_tick_speed = 1; }
          else FOC_tick_speed = 0;
        }
    }
}
// Register Motor Control Application
void mc_register_callback(void (*motor_appli_callback)(void))
{
    motor_control_params.motor_appli_callback = motor_appli_callback;
}
// Unregister Motor Control Application
void mc_unregister_callback()
{
    motor_control_params.motor_appli_callback = NULL;
}
//------------------------------------------------------------------------------
/*! \name Global Low level initialization
 */
//! @{
void mc_global_init(void)
{
  // --------------------- ADC Initialization ----------------------------------
  static const gpio_map_t ADC_GPIO_MAP =
  {
    {CURRENT_IA_ADC_PIN, CURRENT_IA_ADC_FUNCTION },
    {CURRENT_IB_ADC_PIN, CURRENT_IB_ADC_FUNCTION },
    {CURRENT_IC_ADC_PIN, CURRENT_IC_ADC_FUNCTION },

  };
  gpio_enable_module(ADC_GPIO_MAP, sizeof(ADC_GPIO_MAP) / sizeof(ADC_GPIO_MAP[0]));
  // configure ADC
  adc_configure(adc);
  // --------------------- Hall Sensors Initialization -------------------------
  //~ hall_estimator_init();	//TODO: remove
  //~ hall_estimator_init_interrupt();	//TODO: remove
  //~ TODO: Better name?
  tirq_estimator_init();
  tirq_estimator_init_interrupt();
  // --------------------- PWM Initialization ----------------------------------
  pwm_drv_options.max_pwm_value = 1200;    // Cprd
  pwm_drv_init(&pwm_drv_options);
  INTC_register_interrupt(&pwm_int_handler, AVR32_PWM_IRQ, AVR32_INTC_INT0);
}

void mc_lowlevel_start(void)
{
  // --------------------- ADC Start ----------------------------------
  if (MC_BLDC_motor.mc_motor_direction == MC_CW)
  {
    adc_channel_ia = CURRENT_IB_ADC_CHANNEL;
    adc_channel_ib = CURRENT_IC_ADC_CHANNEL;
    adc_channel_ic = CURRENT_IA_ADC_CHANNEL;
  }
  else
  {
    adc_channel_ia = CURRENT_IC_ADC_CHANNEL;
    adc_channel_ib = CURRENT_IB_ADC_CHANNEL;
    adc_channel_ic = CURRENT_IA_ADC_CHANNEL;
  }
  // Enable the ADC channels.
  adc_enable(adc,adc_channel_ia);
  adc_enable(adc,adc_channel_ib);
  adc_enable(adc,adc_channel_ic);
  // --------------------- Hall Sensors Start -------------------------
  //~ hall_estimator_start();	//TODO: remove
  tirq_estimator_start();
  // --------------------- PWM Start ----------------------------------
  pwm_drv_start(); // Start PWM Channels
  pwm_drv_duty_cycle(&pwm_drv_options,500,550,500);
  //~ pwm_drv_duty_cycle(&pwm_drv_options,500,550,500,550,500,550);
}

void mc_lowlevel_stop(void)
{
  // --------------------- ADC Stop ----------------------------------
  // Disable the ADC channels.
  adc_disable(adc,adc_channel_ia);
  adc_disable(adc,adc_channel_ib);
  adc_disable(adc,adc_channel_ic);
  // --------------------- Hall Sensors Stop -------------------------
  //~ hall_estimator_stop();
  tirq_estimator_stop();
  // --------------------- PWM Stop ----------------------------------
  pwm_drv_stop(); // Stop PWM Channels
  mc_update_duty_cycle((volatile U16)500, (volatile U16)550, (volatile U16)500);
  //~ mc_update_duty_cycle(500,550,500,550,500,550);
}
//@}

unsigned long mc_get_ia(void)
{
    adc_start(adc);
    return adc_get_value(adc, adc_channel_ia);
}

unsigned long mc_get_ib(void)
{
    adc_start(adc);
    return adc_get_value(adc, adc_channel_ib);
}

unsigned long mc_get_ic(void)
{
    adc_start(adc);
    return adc_get_value(adc, adc_channel_ic);
}

//~ void mc_update_duty_cycle( volatile U16 mc_duty0,
                           //~ volatile U16 mc_duty1,
                           //~ volatile U16 mc_duty2,
                           //~ volatile U16 mc_duty3,
                           //~ volatile U16 mc_duty4,
                           //~ volatile U16 mc_duty5)
void mc_update_duty_cycle( volatile U16 mc_duty0,
                           volatile U16 mc_duty1,
                           volatile U16 mc_duty2)
{
     pwm_drv_duty_cycle(&pwm_drv_options,
                          mc_duty0,
                          mc_duty1,
                          mc_duty2);
                          //~ mc_duty3,
                          //~ mc_duty4,
                          //~ mc_duty5);
}
