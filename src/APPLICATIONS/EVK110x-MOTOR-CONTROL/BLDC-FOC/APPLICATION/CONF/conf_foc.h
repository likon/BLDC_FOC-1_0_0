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

#define Un_1                                  2147483648  // 2^31
#define PI                                    3.141592    //3.141592654...
#define PWM_HZ                                22000
#define PWM_PERIOD (FPBA_HZ / PWM_HZ)
//~ #define MAX_PWM_VALUE 1200	//Original value. Now called PWM_PERIOD instead, Mjan

//Te: Seem to have something to do with 10000, not used in the code anyway, well, it is the PWM period
#define    Te                                   	(Un_1 * (1 / PWM_HZ))    // PWM_HZ = 20kHz = 50 us -> 107374
//~ #define    Te                                   214748    // 100 us
//~ #define    Fe                                   10000     // 100 us



// #define SENSOR_TASK
//TICK_SPEED_FACTOR Decides how often FOC_regulate_torque(); will run in the PWM interrupt. Te is a time variable.
#define TICK_SPEED_FACTOR          1// Min Value = 360*Te,

//zero_adc: seem to be bias value to satisfy adc_value_ia + adc_value_ib + adc_value_ic = zero_adc
//TODO: ZERO_ADC should be used, and something descriptive as 'adc_sum_zero' should be defined to (3 * ZERO_ADC)
//~ # define  zero_adc                              1680
#define ZERO_ADC 0x200
#define zero_adc (3 * ZERO_ADC)

//echelle_adc and offset: Seem to be scale values, ia= (adc_value_ia-offset)*echelle_adc;
//TODO: The 'offset' value should be coupled to zero_adc. (zero_adc = 3 * offset) or (offset = zero_adc / 3), doesnt make sense to have them separate
//~ # define  echelle_adc                           768955      //echelle = scale
# define  echelle_adc                           (768955*1)      //echelle = scale
//~ # define  offset                                560
#define offset ZERO_ADC


/* Section 5.9: Limitations of the sensor field oriented control algorithm:
 * For example, when computing current regulation, every ADC samples are scaled by a variable named E. This variable matches a ratio of the bus voltage. In the current implementation, this variable has been fixed to half of the nominal voltage of the motor. In that case, the nominal speed is equal to 2000 rpm. */
#define    E                                    12

//rayon_limitation: Seem to be used at Park transformation for Vd and Vq, seem to be close to 2^31 * 1 / sqrt(8)
# define  rayon_limitation                      751619276   // 1/sqrt8  rayon = radius
# define  rayon_carre_limitation                263066746   // 1/8      carre = square
//~ # define  rayon_limitation                      (1753413056/ (2 * E))//751619276   // sqrt(2/3)	//1/sqrt8  rayon = radius
//~ # define  rayon_carre_limitation                (1431655765 / (2 * E))//263066746   // 2/3      carre = square
# define  vitesse_inst                          (864000)    //          vitesse = speed

//~ # define  tirq_demi_period_init                 300000
# define  tirq_demi_period_init                     350000	//demi = half
//transf_v: Seem to have something to do with 2500, as 2^31/858992 = 2500.001336, and with speedm.
# define  transf_v                              858993

#define   IQREF_RAMPUP    ((int)((0.4 * Un_1) / (2*E))) // was: (89478485*24)       //!< (Icrete*2^31)/(2E)
#define   IQREF_REGULAR   (IQREF_RAMPUP>>1)   //!< (Icrete*2^31)/(2E)
//! Motor Parameters
// !!!!!!!!! Current and Voltages are divided by 2E  !!!!!!!!//E seem to be 12, Icrete seem to be 1
//*
//R is 2^31 * 0.22
//~ #define    R              472446402 // R=0.22 ohms
#define    R                  ((int)(0.99 * Un_1)) // R=1 ohms?????
//~ #define    Lc             547608    // Lc =0.5(0.51e-3).2^31;  en general Lc= Lc .2^31
//~ #define    Kcn            1503238   //Kcn=Kc/2E .2^31 = (0.0168/2.12).2^31

//~ #define Lc 100
//~ #define Kcn 1000
//~ #define    P              4
#define    P                  50	//Is this really the nr of pole pairs?

//Section 4.4, AVR32723 appnote.
//J seem to be ~1000
#define     Kp_speed      741092              //!< Kdv = (0.04*J*R)/(Lc)      //1778545 //Change this, make motor spin faster
#define     Ki_speed      1598                //!< Kiv = (0.01*J*R^2)/(Lc^2)  //3836

#define     Kp_id         ((int)(0.01 * Un_1))//(2*R)               //<!Theorical Value  : 7R //Proportional constant
#define     Ki_id         ((int)(0.005 * Un_1))//(163040327>>4)      //<! Theorical Value : 4 *R^2/Lc	//Integral constant

#define     Kp_iq         ((int)(0.01 * Un_1))//(2*R)               //<!Theorical Value  : 7R	//Proportional constant
#define     Ki_iq         ((int)(0.005 * Un_1))//(163040327>>4)      //<! Theorical Value : 4 *R^2/Lc //Integral constant
#define Lc ((int)(0.0051 * Un_1) / 2)
//~ #define Lc (21904333 / 1000)
#define Kcn Ki_iq

#endif  // _CONF_FOC_H_
