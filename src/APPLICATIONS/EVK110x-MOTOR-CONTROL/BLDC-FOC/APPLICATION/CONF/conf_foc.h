/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file ******************************************************************
 *
 * \brief FOC configuration file.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
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


#ifndef _CONF_FOC_H_
#define _CONF_FOC_H_

#include <avr32/io.h>
#include "board.h"

#define SPEED_REGULATION
//#define USB_DEBUG
#undef USB_DEBUG
// #define SENSOR_TASK
#define TICK_SPEED_FACTOR          6// Min Value = 360*Te
//~ #define FTICK_HZ                    20000	//Unused, Mjan 20110823
#define    PI             3.141592	//3.141592654

//zero_adc: seem to be bias value to satisfy adc_value_ia + adc_value_ib + adc_value_ic = zero_adc
# define  zero_adc                              1680
//erreur_max: seem to be a sqrtt iteration value
# define  erreur_max                            300
# define  Un_1                                  2147483648  // 2^31
//echelle_adc and offset: Seem to be scale values, ia= (adc_value_ia-offset)*echelle_adc;
# define  echelle_adc                           768955
# define  offset                                560

//rayon_limitation: Seem to be used at Park transformation for Vd and Vq, seem to be close to 2^31 * 1 / sqrt(8)
# define  rayon_limitation                      751619276  // 1/sqrt8
# define  rayon_carre_limitation                263066746  // 1/8
# define  vitesse_inst                          864000

//~ # define  pi_Fcpu                               150796447
#define PI_X_FCPU                                 ((int)(FCPU_HZ * PI))
# define  tirq_demi_period_init                 300000
//transf_v: Seem to have something to do with 2500, as 2^31/858992 = 2500.001336
# define  transf_v                              858993
//Te: Seem to have something to do with 10000
//~ #define    Te                                   214748    // 100 us
//~ #define    Fe                                   10000     // 100 us
#define   IQREF_RAMPUP                          89478485        //!< (Icrete*2^31)/(2E)
#define   IQREF_REGULAR                         (IQREF_RAMPUP>>1)   //!< (Icrete*2^31)/(2E)
//! Motor Parameters
// !!!!!!!!! Current and Voltages are divided by 2E  !!!!!!!!
//R is (more or less) 2^31 * 0.22
#define    R              472446402 // R=0.22 ohms
#define    Lc             547608    // Lc =0.5(0.51e-3).2^31;  en general Lc= Lc .2^31
#define    Kcn            1503238   //Kcn=Kc/2E .2^31 = (0.0168/2.12).2^31
#define    P              4
#define    E              12

//Section 4.4
#define     Kp_speed      741092              //!< Kdv = (0.04*J*R)/(Lc)      //1778545
#define     Ki_speed      1598                //!< Kiv = (0.01*J*R^2)/(Lc^2)  //3836

#define     Kp_id         4*R                 //<!Theorical Value  : 7R
#define     Ki_id         (163040327>>4)      //<! Theorical Value : 4 *R^2/Lc

#define     Kp_iq         4*R                 //<!Theorical Value  : 7R
#define     Ki_iq         (163040327>>4)      //<! Theorical Value : 4 *R^2/Lc
/*! \name USART Settings
 */
//! @{
#if BOARD == EVK1100
#  define STDIO_USART               (&AVR32_USART1)
#  define STDIO_USART_BAUDRATE      57600
#  define STDIO_USART_RX_PIN        AVR32_USART1_RXD_0_0_PIN
#  define STDIO_USART_RX_FUNCTION   AVR32_USART1_RXD_0_0_FUNCTION
#  define STDIO_USART_TX_PIN        AVR32_USART1_TXD_0_0_PIN
#  define STDIO_USART_TX_FUNCTION   AVR32_USART1_TXD_0_0_FUNCTION
#elif BOARD == EVK1101
#  define STDIO_USART               (&AVR32_USART1)
#  define STDIO_USART_BAUDRATE      57600
#  define STDIO_USART_RX_PIN        AVR32_USART1_RXD_0_0_PIN
#  define STDIO_USART_RX_FUNCTION   AVR32_USART1_RXD_0_0_FUNCTION
#  define STDIO_USART_TX_PIN        AVR32_USART1_TXD_0_0_PIN
#  define STDIO_USART_TX_FUNCTION   AVR32_USART1_TXD_0_0_FUNCTION
#elif BOARD == USB11
#  define STDIO_USART               (&AVR32_USART1)
#  define STDIO_USART_BAUDRATE      57600
#  define STDIO_USART_RX_PIN        AVR32_USART1_RXD_0_0_PIN
#  define STDIO_USART_RX_FUNCTION   AVR32_USART1_RXD_0_0_FUNCTION
#  define STDIO_USART_TX_PIN        AVR32_USART1_TXD_0_0_PIN
#  define STDIO_USART_TX_FUNCTION   AVR32_USART1_TXD_0_0_FUNCTION
#endif
//! @}


#endif  // _CONF_FOC_H_
