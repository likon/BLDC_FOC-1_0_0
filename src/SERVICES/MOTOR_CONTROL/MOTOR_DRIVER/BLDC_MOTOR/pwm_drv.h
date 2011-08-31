/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief PWM Advance driver for AVR32 UC3.
 *
 * AVR32 PWM Advance module.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices with a TC module can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *                       Adapted by Marcus Jansson for Eberharter Elektronik, GmbH
 *
 ******************************************************************************/

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
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef _PWM_DRV_H_
#define _PWM_DRV_H_
extern void __attribute__((interrupt)) pwm_interrupt_pic(void);
__attribute__((__interrupt__)) void pwm_int_handler( void );
//! Input parameters.
typedef struct
{
  //! PWM frequency.
  U32 max_pwm_value;

} pwm_drv_options_t;
//------------------------------------------------------------------------------
/*! \name Initialization Functions
 */
//! @{
/*! \brief Init PWM_DRV service
 * \param *pwm_drv_options PWM_DRV struct param.
 */
extern void pwm_drv_init(volatile pwm_drv_options_t *pwm_drv_options);
/*! \brief START pwm channel PWM_DRV service
 */
extern void pwm_drv_start(void);
/*! \brief STOP pwm function PWM_DRV  service
 */
extern void pwm_drv_stop(void);
//------------------------------------------------------------------------------
/*! \name Set/Get Functions
 */
//! @{
/*! \brief Set PWM_DRV duty cycle services
 * \param *pwm_drv_options PWM_DRV struct param.
 * \param pwm0      Duty cycle to apply on PWM0
 * \param pwm1      Duty cycle to apply on PWM1
 * \param pwm2      Duty cycle to apply on PWM2
 //~ * \param pwm3      Duty cycle to apply on PWM3
 //~ * \param pwm4      Duty cycle to apply on PWM4
 //~ * \param pwm5      Duty cycle to apply on PWM5
 */
extern void pwm_drv_duty_cycle( volatile pwm_drv_options_t * pwm_drv_options,
                                U32 pwm0,
                                U32 pwm1,
                                U32 pwm2);

#endif  // _PWM_DRV_H_
//@}
