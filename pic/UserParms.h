 /**********************************************************************
 *                                                                     *
 *                        Software License Agreement                   *
 *                                                                     *
 *    The software supplied herewith by Microchip Technology           *
 *    Incorporated (the "Company") for its dsPIC controller            *
 *    is intended and supplied to you, the Company's customer,         *
 *    for use solely and exclusively on Microchip dsPIC                *
 *    products. The software is owned by the Company and/or its        *
 *    supplier, and is protected under applicable copyright laws. All  *
 *    rights are reserved. Any use in violation of the foregoing       *
 *    restrictions may subject the user to criminal sanctions under    *
 *    applicable laws, as well as to civil liability for the breach of *
 *    the terms and conditions of this license.                        *
 *                                                                     *
 *    THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION.  NO           *
 *    WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING,    *
 *    BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND    *
 *    FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE     *
 *    COMPANY SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,  *
 *    INCIDENTAL OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.  *
 *                                                                     *
 ***********************************************************************
 *                                                                     *
 *    Filename:       UserParms.h                                      *
 *    Date:           10/01/08                                         *
 *                                                                     *
 *    Tools used:     MPLAB IDE -> 8.14                                *
 *                    C30 -> 3.10                                      *
 *    Linker File:    p33FJ32MC204.gld                                 *
 *                                                                     *
 **********************************************************************/
#ifndef UserParms_H
#define UserParms_H

//************** Start-Up Parameters **************

#define LOCKTIMEINSEC  0.25		// Initial rotor lock time in seconds
								// Make sure LOCKTIMEINSEC*(1.0/LOOPTIMEINSEC)
								// is less than 65535.
#define OPENLOOPTIMEINSEC 5.0	// Open loop time in seconds. This is the time that
								// will take from stand still to closed loop.
								// Optimized to overcome the brake inertia.
								// (Magtrol AHB-3 brake inertia = 6.89 kg x cm2).
#define INITIALTORQUE	1.0		// Initial Torque demand in Amps.
								// Enter initial torque demand in Amps using REFINAMPS() 
								// macro. Maximum Value for reference is defined by 
								// shunt resistor value and differential amplifier gain.
								// Use this equation to calculate maximum torque in 
								// Amperes:
								// 
								// Max REFINAMPS = (VDD/2)/(RSHUNT*DIFFAMPGAIN)
								//
								// For example:
								//
								// RSHUNT = 0.005
								// VDD = 3.3
								// DIFFAMPGAIN = 75
								//
								// Maximum torque reference in Amps is:
								//
								// (3.3/2)/(.005*75) = 4.4 Amperes, or REFINAMPS(4.4)
#define ENDSPEEDOPENLOOP MINSPEEDINRPM

// Values used to test Hurst Motor "NT Dynamo DMB0224C10002" at 24VDC input. Motor datasheet at www.hurstmfg.com
#define POLEPAIRS      	5       // Number of pole pairs
#define PHASERES		((float)2.67)	// Phase resistance in Ohms.
#define PHASEIND		((float)0.00192)// Phase inductance in Henrys.
#define NOMINALSPEEDINRPM 2700	// Make sure NOMINALSPEEDINRPM generates a MAXOMEGA < 1.0
								// Use this formula:
								// MAXOMEGA = NOMINALSPEEDINRPM*SPEEDLOOPTIME*POLEPAIRS*2/60
								// If MAXOMEGA > 1.0, reduce NOMINALSPEEDINRPM or execute
								// speed loop faster by reducing SpeedLoopTime.
								// Maximum position of POT will set a reference of 
								// NOMINALSPEEDINRPM.
#define MINSPEEDINRPM	500	// Minimum speed in RPM. Closed loop will operate at this
								// speed. Open loop will transition to closed loop at
								// this minimum speed. Minimum POT position (CCW) will set
								// a speed reference of MINSPEEDINRPM
#define FIELDWEAKSPEEDRPM 5300 	// Make sure FIELDWEAKSPEEDRPM generates a MAXOMEGA < 1.0
								// Use this formula:
								// MAXOMEGA = FIELDWEAKSPEEDRPM*SPEEDLOOPTIME*POLEPAIRS*2/60
								// If MAXOMEGA > 1.0, reduce FIELDWEAKSPEEDRPM or execute
								// speed loop faster by reducing SpeedLoopTime.
								// Maximum position of POT will set a reference of 
								// FIELDWEAKSPEEDRPM.
								
/*
// Values used to test Shinano Kenshi Motor "LA052-040E" at 24VDC input. Motor datasheet at www.shinano.com
#define POLEPAIRS      			2
#define PHASERES				((float)0.60)	// Phase resistance in Ohms.
#define PHASEIND				((float)0.0022)// Phase inductance in Henrys.
#define NOMINALSPEEDINRPM 		2700
#define MINSPEEDINRPM			500
#define FIELDWEAKSPEEDRPM 		5300
*/
//************** Oscillator Parameters **************

#define PLLIN		8000000		// External Crystal or Clock Frequency (Hz)
#define DESIREDMIPS	40000000	// Enter desired MIPS. If RTDM is used, copy
								// this value to RTDM_FCY in RTDMUSER.h file

//************** PWM and Control Timing Parameters **********

#define PWMFREQUENCY	20000		// PWM Frequency in Hertz
#define DEADTIMESEC		0.000002	// Deadtime in seconds
#define	BUTPOLLOOPTIME	0.100		// Button polling loop period in sec
#define SPEEDLOOPFREQ	1000		// Speed loop Frequency in Hertz. This value must
									// be an integer to avoid pre-compiler error

//************** Slide Mode Controller Parameters **********

#define SMCGAIN			0.85		// Slide Mode Controller Gain (0.0 to 0.9999)
#define MAXLINEARSMC    0.005		// If measured current - estimated current
								// is less than MAXLINEARSMC, the slide mode
								// Controller will have a linear behavior
								// instead of ON/OFF. Value from (0.0 to 0.9999)
//************** Hardware Parameters ****************

#define RSHUNT			0.005	// Value in Ohms of shunt resistors used.
#define DIFFAMPGAIN		75		// Gain of differential amplifier.
#define VDD				3.3		// VDD voltage, only used to convert torque
								// reference from Amps to internal variables

//************** Real Time Data Monitor, RTDM *******************

#define RTDM		// This definition enabled Real Time Data Monitor, UART interrupts
					// to handle RTDM protocol, and array declarations for buffering
					// information in real time

#undef DMCI_DEMO	// Define this if a demo with DMCI is done. Start/stop of motor
					// and speed variation is done with DMCI instead of push button
					// and POT. Undefine "DMCI_DEMO" is user requires operating
					// this application with no DMCI

#ifdef RTDM
#define DATA_BUFFER_SIZE 100  //Size in 16-bit Words
#define SNAPDELAY	5 // In number of PWM Interrupts
#define	SNAP1		ParkParm.qIa
#define	SNAP2		ParkParm.qIq
#define SNAP3	    smc1.OmegaFltred
#define SNAP4	    smc1.Theta
#endif


#define SPEEDDELAY 10 // Delay for the speed ramp.
					  // Necessary for the PI control to work properly at high speeds.

//*************** Optional Modes **************
//#define TORQUEMODE
//#define ENVOLTRIPPLE

//************** PI Coefficients **************

//******** D Control Loop Coefficients *******
#define     DKP        Q15(0.05)
#define     DKI        Q15(0.01)
#define     DKC        Q15(0.99999)
#define     DOUTMAX    Q15(0.99999)

//******** Q Control Loop Coefficients *******
#define     QKP        Q15(0.01)
#define     QKI        Q15(0.005)
#define     QKC        Q15(0.99999)
#define     QOUTMAX    Q15(0.99999)

//*** Velocity Control Loop Coefficients *****
#define     WKP        Q15(0.12)
#define     WKI        Q15(0.01)
#define     WKC        Q15(0.99999)
#define     WOUTMAX    Q15(0.95)

//************** ADC Scaling **************
// Scaling constants: Determined by calibration or hardware design. 
#define     DQK        Q15((OMEGA10 - OMEGA1)/2.0)	// POT Scaling
#define     DQKA       Q15(0.5)	// Current feedback software gain
#define     DQKB       Q15(0.5)	// Current feedback software gain

//************** Field Weakening **************
// Enter flux demand Amperes using REFINAMPS() macro. Maximum Value for
// reference is defined by shunt resistor value and differential amplifier gain.
// Use this equation to calculate maximum torque in Amperes:
// 
// Max REFINAMPS = (VDD/2)/(RSHUNT*DIFFAMPGAIN)
//
// For example:
//
// RSHUNT = 0.005
// VDD = 3.3
// DIFFAMPGAIN = 75
//
// Maximum torque reference in Amps is:
//
// (3.3/2)/(.005*75) = 4.4 Amperes, or REFINAMPS(4.4)
//
// in order to have field weakening, this reference value should be negative,
// so maximum value in this example is -4.4, or REFINAMPS(-4.4)
#define     dqKFw0  REFINAMPS(0) //3000
#define     dqKFw1  REFINAMPS(-0.435) //3166
#define     dqKFw2  REFINAMPS(-0.650) //3333
#define     dqKFw3  REFINAMPS(-0.915) //3500
#define     dqKFw4  REFINAMPS(-1.075) //3666
#define     dqKFw5  REFINAMPS(-1.253) //3833
#define     dqKFw6  REFINAMPS(-1.432) //4000
#define     dqKFw7  REFINAMPS(-1.670) //4166
#define     dqKFw8  REFINAMPS(-1.838) //4333
#define     dqKFw9  REFINAMPS(-1.952) //4500
#define     dqKFw10  REFINAMPS(-2.042) //4666
#define     dqKFw11  REFINAMPS(-2.064) //4833
#define     dqKFw12  REFINAMPS(-2.100) //5000
#define     dqKFw13  REFINAMPS(-2.100) //5166
#define     dqKFw14  REFINAMPS(-2.100) //5333
#define     dqKFw15  REFINAMPS(-2.100) //5500


//************** Derived Parameters ****************

#define DPLL		(unsigned int)(2.0*DESIREDMIPS/PLLIN)	// PLL ratio
#define FOSC		(PLLIN*DPLL)	// Clock frequency (Hz)
#define DFCY        (FOSC/2)		// Instruction cycle frequency (Hz)
#define DTCY        (1.0/DFCY)		// Instruction cycle period (sec)
#define DDEADTIME   (unsigned int)(DEADTIMESEC*DFCY)	// Dead time in dTcys
#define LOOPTIMEINSEC (1.0/PWMFREQUENCY) // PWM Period = 1.0 / PWMFREQUENCY
#define IRP_PERCALC (unsigned int)(SPEEDLOOPTIME/LOOPTIMEINSEC)	// PWM loops per velocity calculation
#define SPEEDLOOPTIME (float)(1.0/SPEEDLOOPFREQ) // Speed Control Period
#define LOOPINTCY	(LOOPTIMEINSEC/DTCY)   // Basic loop period in units of Tcy
#define LOCKTIME	(unsigned int)(LOCKTIMEINSEC*(1.0/LOOPTIMEINSEC))
// Time it takes to ramp from zero to MINSPEEDINRPM. Time represented in seconds
#define DELTA_STARTUP_RAMP	(unsigned int)(MINSPEEDINRPM*POLEPAIRS*LOOPTIMEINSEC* \
							LOOPTIMEINSEC*65536*65536/(60*OPENLOOPTIMEINSEC))
// Number of control loops that must execute before the button routine is executed.
#define	BUTPOLLOOPCNT	(unsigned int)(BUTPOLLOOPTIME/LOOPTIMEINSEC)

// This pre-processor condition will generate an error if maximum speed is out of
// range on Q15 when calculating Omega.
#if (FIELDWEAKSPEEDRPM < NOMINALSPEEDINRPM)
	#error FIELDWEAKSPEEDRPM must be greater than NOMINALSPEEDINRPM for field weakening.
	#error if application does not require Field Weakening, set FIELDWEAKSPEEDRPM value
	#error equal to NOMINALSPEEDINRPM
#else
	#if ((FIELDWEAKSPEEDRPM*POLEPAIRS*2/(60*SPEEDLOOPFREQ)) >= 1)
		#error FIELDWEAKSPEEDRPM will generate an Omega value greater than 1 which is the
		#error maximum in Q15 format. Reduce FIELDWEAKSPEEDRPM value, or increase speed
		#error control loop frequency, SPEEDLOOPFREQ
	#endif
#endif
 
#endif
