/*
 * usb11.h
 *
 *  Created on: Aug 9, 2011
 *  Copyright (C) 2011 Marcus Jansson <mjansson256@gmail.com>
 */
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

#ifndef __BOARD_USB11_H__
#define __BOARD_USB11_H__

#include "compiler.h"

//Debug output on USART? remove to get rid of usart output
//~ #define DEBUG

/*! \name Oscillator Definitions
 */
//! @{

// RCOsc has no custom calibration by default. Set the following definition to
// the appropriate value if a custom RCOsc calibration has been applied to your
// part.
//#define FRCOSC          AVR32_PM_RCOSC_FREQUENCY              //!< RCOsc frequency: Hz.

#define FOSC32          32768                                 //!< Osc32 frequency: Hz.
#define OSC32_STARTUP   AVR32_PM_OSCCTRL32_STARTUP_8192_RCOSC //!< Osc32 startup time: RCOsc periods.

#define FOSC0           8000000UL                             //!< Osc0 frequency: Hz.
#define FCPU_HZ        48000000UL                             //!< PLL frequency: Hz.
#define FPBA_HZ        (FCPU_HZ / 4)


#define OSC0_STARTUP    AVR32_PM_OSCCTRL0_STARTUP_2048_RCOSC  //!< Osc0 startup time: RCOsc periods.

//! @}

#define TC_CHANNEL_0 0

//TODO: Remove MLED and BUT definitions and relevant debug code
//~ #define MLED0 AVR32_PIN_PB27
//~ #define MLED1 AVR32_PIN_PB28
//~ #define MLED2 AVR32_PIN_PB29
//~ #define MLED3 AVR32_PIN_PB30
//~
//~ #define BUT0 AVR32_PIN_PX16
//~ #define BUT1 AVR32_PIN_PX19
//~ #define BUT2 AVR32_PIN_PX22
//! @}

/* TODO: Not used yet. Remove if still unused at end of project. */
#define MOTPWMA AVR32_PWM_0_PIN	//AVR32_PIN_PB19 for AVR32UC3A1512
#define MOTPWMB AVR32_PWM_0_PIN	//AVR32_PIN_PB20 for AVR32UC3A1512
#define MOTPWMC AVR32_PWM_0_PIN	//AVR32_PIN_PB21 for AVR32UC3A1512


/*! \name USB11 definitions
 */
//! @{

#define MOTEN AVR32_PIN_PA11
#define POWER_DOWN_INT AVR32_PIN_PB01
#define STBINV AVR32_PIN_PB23

//! @}


/*! \name PWM Connections of Actuators
 */
//! @{
#define PWM_PORT_NUMBER             0
#define PWM_PORT                    (AVR32_GPIO.port[PWM_PORT_NUMBER])

/* TODO: Not used yet. Remove if still unused at end of project.
#define MOTPWMA AVR32_PWM_0_PIN	//AVR32_PIN_PB19 for AVR32UC3A1512
#define MOTPWMB AVR32_PWM_0_PIN	//AVR32_PIN_PB20 for AVR32UC3A1512
#define MOTPWMC AVR32_PWM_0_PIN	//AVR32_PIN_PB21 for AVR32UC3A1512
*/
//TODO: Order of channels?
//PB19 - MOTPWMA - J1-2
#define PWM_XL_PIN_NUMBER           AVR32_PWM_0_PIN
#define PWM_XL                      (PWM_XL_PIN_NUMBER - PWM_PORT_NUMBER * 0x20)
#define PWM_XL_PWM_FUNCTION         AVR32_PWM_0_FUNCTION
#define PWM_XL_PWM_CHANNEL          0

//PB20 - MOTPWMB -J1-3
#define PWM_YL_PIN_NUMBER           AVR32_PWM_1_PIN
#define PWM_YL                      (PWM_YL_PIN_NUMBER - PWM_PORT_NUMBER * 0x20)
#define PWM_YL_PWM_FUNCTION         AVR32_PWM_1_FUNCTION
#define PWM_YL_PWM_CHANNEL          1

//PB21 - MOTPWMC - J1-1
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
// PA21
#define CURRENT_IA_ADC_CHANNEL      0
#define CURRENT_IA_ADC_PIN          AVR32_ADC_AD_0_PIN
#define CURRENT_IA_ADC_FUNCTION     AVR32_ADC_AD_0_FUNCTION

// PA22
#define CURRENT_IB_ADC_CHANNEL      1
#define CURRENT_IB_ADC_PIN          AVR32_ADC_AD_1_PIN
#define CURRENT_IB_ADC_FUNCTION     AVR32_ADC_AD_1_FUNCTION

// PA23
#define CURRENT_IC_ADC_CHANNEL      2
#define CURRENT_IC_ADC_PIN          AVR32_ADC_AD_2_PIN
#define CURRENT_IC_ADC_FUNCTION     AVR32_ADC_AD_2_FUNCTION

//! @}

/* Connectors defines used during debug */
#define J13_10 AVR32_PIN_PA05
#define J13_11 AVR32_PIN_PA06
#define J13_12 AVR32_PIN_PA14
#define J13_13 AVR32_PIN_PA13

/* Usart definitions */
#define STDIO_USART               (&AVR32_USART0)
#define STDIO_USART_BAUDRATE      115200	//57600
#define STDIO_USART_RX_PIN        AVR32_USART0_RXD_0_0_PIN
#define STDIO_USART_RX_FUNCTION   AVR32_USART0_RXD_0_0_FUNCTION
#define STDIO_USART_TX_PIN        AVR32_USART0_TXD_0_0_PIN
#define STDIO_USART_TX_FUNCTION   AVR32_USART0_TXD_0_0_FUNCTION


#endif /* __BOARD_USB11_H__ */
