/*
 * usb11.h
 *
 *  Created on: Aug 8, 2011
 *  Copyright (C) 2011 Marcus Jansson <mjansson256@gmail.com>
 */

#ifndef __USB11_H__
#define __USB11_H__
/*! \file *********************************************************************
 *
 * \brief USB11 board header file.
 *
 * This file contains definitions and services related to the features of the
 * USB11 board.
 *
 *  Compiler: GNU GCC for AVR32
 *  Copyright (C) 2011 Marcus Jansson <mjansson256@gmail.com>\n
 *
 ******************************************************************************/
#include "compiler.h"

#if BOARD == USB11

/*! \name USB11 definitions
 */
//! @{

#define MOTEN AVR32_PIN_PA11
#define POWER_DOWN_INT AVR32_PIN_PB01

//! @}



/*! \name PWM Connections of Actuators
 */
//! @{
#define PWM_PORT_NUMBER             0
#define PWM_PORT                    (AVR32_GPIO.port[PWM_PORT_NUMBER])

/* TODO: Not used yet. Remove if still unused at end of project.
#define MOTPWMA AVR32_PWM_0_0_PIN	//AVR32_PIN_PB19 for AVR32UC3A1512
#define MOTPWMB AVR32_PWM_0_0_PIN	//AVR32_PIN_PB20 for AVR32UC3A1512
#define MOTPWMC AVR32_PWM_0_0_PIN	//AVR32_PIN_PB21 for AVR32UC3A1512
*/

#define PWM_XL_PIN_NUMBER           AVR32_PWM_0_PIN
#define PWM_XL                      (PWM_XL_PIN_NUMBER - PWM_PORT_NUMBER * 0x20)
#define PWM_XL_PWM_FUNCTION         AVR32_PWM_0_FUNCTION
#define PWM_XL_PWM_CHANNEL          0

#define PWM_YL_PIN_NUMBER           AVR32_PWM_1_PIN
#define PWM_YL                      (PWM_YL_PIN_NUMBER - PWM_PORT_NUMBER * 0x20)
#define PWM_YL_PWM_FUNCTION         AVR32_PWM_1_FUNCTION
#define PWM_YL_PWM_CHANNEL          1

#define PWM_ZL_PIN_NUMBER           AVR32_PWM_2_PIN
#define PWM_ZL                      (PWM_ZL_PIN_NUMBER - PWM_PORT_NUMBER * 0x20)
#define PWM_ZL_PWM_FUNCTION         AVR32_PWM_2_FUNCTION
#define PWM_ZL_PWM_CHANNEL          2

//! @}

/*! \name Current Sensor pin definitions
 */
//! @{

#define MOTIA AVR32_ADC_AD_0_PIN
#define MOTIB AVR32_ADC_AD_1_PIN
#define MOTIC AVR32_ADC_AD_2_PIN

//! @}

/*! \name ADC Connections of Current Measurement
 */
//! @{
// PB19
#define CURRENT_IA_ADC_CHANNEL      0
#define CURRENT_IA_ADC_PIN          AVR32_ADC_AD_0_PIN
#define CURRENT_IA_ADC_FUNCTION     AVR32_ADC_AD_0_FUNCTION

// PB20
#define CURRENT_IB_ADC_CHANNEL      1
#define CURRENT_IB_ADC_PIN          AVR32_ADC_AD_1_PIN
#define CURRENT_IB_ADC_FUNCTION     AVR32_ADC_AD_1_FUNCTION

// PB21
#define CURRENT_IC_ADC_CHANNEL      2
#define CURRENT_IC_ADC_PIN          AVR32_ADC_AD_2_PIN
#define CURRENT_IC_ADC_FUNCTION     AVR32_ADC_AD_2_FUNCTION

//! @}


#else
#  error Wrong extension board. Try USB11 instead.
#endif

#endif /* __USB11_H__ */
