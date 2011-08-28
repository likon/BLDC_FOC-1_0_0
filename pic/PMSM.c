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
 *    Filename:       PMSM.c                                           *
 *    Date:           10/01/08                                         *
 *                                                                     *
 *    Tools used:     MPLAB IDE -> 8.14                                *
 *                    C30 -> 3.10                                      *
 *    Linker File:    p33FJ32MC204.gld                                 *
 *                                                                     *
 ***********************************************************************
 *      Code Description                                               *
 *                                                                     *
 *  This file demonstrates Vector Control of a 3 phase PMSM using the  *
 *  dsPIC. SVM is used as the modulation strategy. Currents are        *
 *  measured to estimate position and speed of PMSM Motors             *
 *                                                                     *
 **********************************************************************/

/************** GLOBAL DEFINITIONS ***********/

#define INITIALIZE
#include "general.h"
#include "parms.h"
#include "svgen.h"
#include "ReadADC.h"
#include "MeasCurr.h"
#include "control.h"
#include "pi.h"
#include "park.h"
#include "UserParms.h"
#include "smcpos.h"
#include "FdWeak.h"
//~ #include "RTDM.h"

#include "conf_foc.h"
#include "util.h"


/******************************************************************************/
/* Configuration bits                                                         */
/******************************************************************************/

//~ _FOSCSEL(FNOSC_FRC);
/* Start with FRC will switch to Primary (XT, HS, EC) Oscillator with PLL */

//~ _FOSC(FCKSM_CSECMD & POSCMD_XT & IOL1WAY_OFF);
/* Clock Switching Enabled and Fail Safe Clock Monitor is disable
   Primary Oscillator Mode: XT Crystal */

//~ _FBS (BSS_NO_FLASH & BWRP_WRPROTECT_OFF);
/* no Boot sector and
   write protection disabled */

//~ _FWDT (FWDTEN_OFF);
/* Turn off Watchdog Timer */

//~ _FGS (GSS_OFF & GCP_OFF & GWRP_OFF);
/* Set Code Protection Off for the General Segment */

//~ _FPOR (PWMPIN_ON & HPOL_ON & LPOL_ON & FPWRT_PWR128);
/* PWM mode is Port registers
   PWM high & low active high
   alternate I2C mapped to SDA1/SCL1
   FPOR power on reset 128ms */

//~ _FICD (ICS_PGD3 & JTAGEN_OFF);
/* Use PGC3/PGD3 for programming and debugging */

/************** END OF GLOBAL DEFINITIONS ***********/

/********************* Variables to display data using DMCI *********************************/
int count = 0; // delay for ramping the reference velocity
int VelReq = 0;

SMC smc1 = SMC_DEFAULTS;

unsigned long Startup_Ramp = 0;	/* Start up ramp in open loop. This variable
								is incremented in CalculateParkAngle()
								subroutine, and it is assigned to
								ParkParm.qAngle as follows:
								ParkParm.qAngle += (int)(Startup_Ramp >> 16);*/

unsigned int Startup_Lock = 0;	/* This is a counter that is incremented in
								CalculateParkAngle() every time it is called.
								Once this counter has a value of LOCK_TIME,
								then theta will start increasing moving the
								motor in open loop. */

union   {
        struct
            {
            unsigned int OpenLoop;	// Indicates if motor is running in open or closed loop
            unsigned int RunMotor;	// If motor is running, or stopped.
			unsigned int EnTorqueMod;	// This bit enables Torque mode when running closed loop
			unsigned int EnVoltRipCo;	// Bit that enables Voltage Ripple Compensation
            unsigned int Btn1Pressed;	// Button 1 has been pressed.
            unsigned int Btn2Pressed;	// Button 2 has been pressed.
            unsigned int ChangeMode;	// This flag indicates that a transition from open to closed
									// loop, or closed to open loop has happened. This
									// causes DoControl subroutine to initialize some variables
									// before executing open or closed loop for the first time
            unsigned int ChangeSpeed;	// This flag indicates a step command in speed reference.
									// This is mainly used to analyze step response
            //~ unsigned    int nothing;
            }bit;
        unsigned int Word[8];
        } uGF;

tPIParm     PIParmD;	// Structure definition for Flux component of current, or Id
tPIParm     PIParmQ;	// Structure definition for Torque component of current, or Iq
tPIParm     PIParmW;	// Structure definition for Speed, or Omega

tReadADCParm ReadADCParm;	// Struct used to read ADC values.

// Speed Calculation Variables

WORD iADCisrCnt = 0;	// This Counter is used as a timeout for polling the push buttons
						// in main() subroutine. It will be reset to zero when it matches
						// dButPolLoopCnt defined in UserParms.h
SFRAC16 PrevTheta = 0;	// Previous theta which is then substracted from Theta to get
						// delta theta. This delta will be accumulated in AccumTheta, and
						// after a number of accumulations Omega is calculated.
SFRAC16 AccumTheta = 0;	// Accumulates delta theta over a number of times
WORD AccumThetaCnt = 0;	// Counter used to calculate motor speed. Is incremented
						// in SMC_Position_Estimation() subroutine, and accumulates
						// delta Theta. After N number of accumulations, Omega is
						// calculated. This N is diIrpPerCalc which is defined in
						// UserParms.h.

// Vd and Vq vector limitation variables

SFRAC16 qVdSquared = 0;	// This variable is used to know what is left from the VqVd vector
						// in order to have maximum output PWM without saturation. This is
						// done before executing Iq control loop at the end of DoControl()

SFRAC16 DCbus = 0;		// DC Bus measured continuously and stored in this variable
						// while motor is running. Will be compared with TargetDCbus
						// and Vd and Vq will be compensated depending on difference
						// between DCbus and TargetDCbus

SFRAC16 TargetDCbus = 0;// DC Bus is measured before running motor and stored in this
						// variable. Any variation on DC bus will be compared to this value
						// and compensated linearly.

SFRAC16 Theta_error = 0;// This value is used to transition from open loop to closed looop.
						// At the end of open loop ramp, there is a difference between
						// forced angle and estimated angle. This difference is stored in
						// Theta_error, and added to estimated theta (smc1.Theta) so the
						// effective angle used for commutating the motor is the same at
						// the end of open loop, and at the begining of closed loop.
						// This Theta_error is then substracted from estimated theta
						// gradually in increments of 0.05 degrees until the error is less
						// than 0.05 degrees.

/************* START OF MAIN FUNCTION ***************/
int pic_main ( void )
{
    //The settings below set up the oscillator and PLL for 40 MIPS as
    //follows:
    //            Crystal Frequency  * (DIVISOR+2)
    // Fcy =     ---------------------------------
    //              PLLPOST * (PRESCLR+2) * 4
	// Crystal  = Defined in UserParms.h
	// Fosc		= Crystal * dPLL defined in UserParms.h
	// Fcy		= DesiredMIPs

	//~ PLLFBD = (int)(DPLL * 4 - 2); // dPLL derived in UserParms.h
	//~ CLKDIVbits.PLLPOST = 0;		// N1=2
	//~ CLKDIVbits.PLLPRE = 0;		// N2=2
	//~ __builtin_write_OSCCONH(0x03);
	//~ __builtin_write_OSCCONL(0x01);

	//~ while(OSCCONbits.COSC != 0b011);
	//~ // Wait for PLL to lock
	//~ while(OSCCONbits.LOCK != 1);

	//~ SMCInit(&smc1);
    //~ SetupPorts();
   	SetupControlParameters();
	//~ FWInit();
    //~ uGF.Word = 0;                   // clear flags

    while(1)
    {
        uGF.bit.ChangeSpeed = 0;
        // init Mode
        uGF.bit.OpenLoop = 1;           // start in openloop

        //~ IEC0bits.AD1IE = 0;				// Make sure ADC does not generate
										// interrupts while parameters
										// are being initialized

        // init user specified parms and stop on error
        if( SetupParm() )
        {
            // Error
            uGF.bit.RunMotor=0;
            while(1);	//Jam here!
        }

        // zero out i sums
        PIParmD.qdSum = 0;
        PIParmQ.qdSum = 0;
        PIParmW.qdSum = 0;

        // Enable ADC interrupt and begin main loop timing
        //~ IFS0bits.AD1IF = 0;
        //~ IEC0bits.AD1IE = 1;

        //~ if(!uGF.bit.RunMotor)
        //~ {
            // Initialize current offset compensation
            //~ while(!pinButton1);                  //wait here until button 1 is pressed
            //~ while(pinButton1);                  //when button 1 is released

			SetupParm();
            //~ uGF.bit.RunMotor = (int)1;               //then start motor
        //~ }

        // Run the motor
        uGF.bit.ChangeMode = 1;	// Ensure variable initialization when open loop is
								// executed for the first time

		//Run Motor loop
        while(1)
        {
            // The code that polls the buttons executes every 100 msec.
            if(iADCisrCnt >= BUTPOLLOOPCNT)
            {
				//~ if (uGF.bit.RunMotor == 0)
					//~ break;

      			// Button 1 starts or stops the motor
				//~ if(pinButton1)
	            //~ {
					//~ DebounceDelay();
                    //~ if(pinButton1)
					//~ {
						//~ if( !uGF.bit.Btn1Pressed )
                        	//~ uGF.bit.Btn1Pressed  = 1;
                    //~ }
                	//~ else
                    //~ {
                    	//~ if( uGF.bit.Btn1Pressed )
                        //~ {
	                        //~ // Button just released
	                        //~ uGF.bit.Btn1Pressed  = 0;
	                        //~ // begin stop sequence
	                        //~ uGF.bit.RunMotor = 0;
	                        //~ break;
                        //~ }
                    //~ }
				}

				//while running button 2 will double/half the speed
                //~ if(pinButton2)
                //~ {
					//~ DebounceDelay();
					//~ if(pinButton2)
					//~ {
	                    //~ if( !uGF.bit.Btn2Pressed )
	                        //~ uGF.bit.Btn2Pressed  = 1;
	                //~ }
                	//~ else
                    //~ {
                    	//~ if( uGF.bit.Btn2Pressed )
                        //~ {
                        	//~ // Button just released
                        	//~ uGF.bit.Btn2Pressed  = 0;
							//~ uGF.bit.ChangeSpeed = !uGF.bit.ChangeSpeed;
                        //~ }
                    //~ }
				//~ }
            //~ }  // end of button polling code
        }   // End of Run Motor loop
    } // End of Main loop
    // should never get here
    while(1){}
}

//---------------------------------------------------------------------
// Executes one PI itteration for each of the three loops Id,Iq,Speed,

void DoControl( void )
{
	ReadSignedADC0( &ReadADCParm );

    if( uGF.bit.OpenLoop )
        {
        // OPENLOOP:	force rotating angle, and control Iq and Id
		//				Also limits Vs vector to ensure maximum PWM duty
		//				cycle and no saturation

		// This If statement is executed only the first time we enter open loop,
		// everytime we run the motor
        if( uGF.bit.ChangeMode )
        {
            // just changed to openloop
            uGF.bit.ChangeMode = 0;
            // synchronize angles

            // VqRef & VdRef not used
            CtrlParm.qVqRef = 0;
            CtrlParm.qVdRef = 0;
			CtrlParm.qVelRef = 0;
			Startup_Lock = 0;
			Startup_Ramp = 0;
			// Initialize SMC
			smc1.Valpha = 0;
			smc1.Ealpha = 0;
			smc1.EalphaFinal = 0;
			smc1.Zalpha = 0;
			smc1.EstIalpha = 0;
			smc1.Vbeta = 0;
			smc1.Ebeta = 0;
			smc1.EbetaFinal = 0;
			smc1.Zbeta = 0;
			smc1.EstIbeta = 0;
			smc1.Ialpha = 0;
			smc1.IalphaError = 0;
			smc1.Ibeta = 0;
			smc1.IbetaError = 0;
			smc1.Theta = 0;
			smc1.Omega = 0;
        }

		// Enter initial torque demand in Amps using REFINAMPS() macro.
		// Maximum Value for reference is defined by shunt resistor value and
		// differential amplifier gain. Use this equation to calculate
		// maximum torque in Amperes:
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
		// If motor requires more torque than Maximum torque to startup, user
		// needs to change either shunt resistors installed on the board,
		// or differential amplifier gain.

		CtrlParm.qVqRef = REFINAMPS(INITIALTORQUE);

        if(AccumThetaCnt == 0)
	    {
            PIParmW.qInMeas = smc1.Omega;
		}

        // PI control for D
        PIParmD.qInMeas = ParkParm.qId;
        PIParmD.qInRef  = CtrlParm.qVdRef;
        CalcPI(&PIParmD);
        ParkParm.qVd    = PIParmD.qOut;

		// Vector limitation
		// Vd is not limited
		// Vq is limited so the vector Vs is less than a maximum of 95%.
		// The 5% left is needed to be able to measure current through
		// shunt resistors.
		// Vs = SQRT(Vd^2 + Vq^2) < 0.95
		// Vq = SQRT(0.95^2 - Vd^2)
		//~ qVdSquared = FracMpy(PIParmD.qOut, PIParmD.qOut);
       	//~ PIParmQ.qOutMax = _Q15sqrt(Q15(0.95*0.95) - qVdSquared);
       	qVdSquared = PIParmD.qOut * PIParmD.qOut;
       	PIParmQ.qOutMax = sqrtt(Q15(0.95*0.95) - qVdSquared, SQRT_ERROR_MAX);
		PIParmQ.qOutMin = -PIParmQ.qOutMax;

        // PI control for Q
        PIParmQ.qInMeas = ParkParm.qIq;
        PIParmQ.qInRef  = CtrlParm.qVqRef;
        CalcPI(&PIParmQ);
        ParkParm.qVq    = PIParmQ.qOut;

    }

    else
    // Closed Loop Vector Control
    {
		// Pressing one of the push buttons, speed reference (or torque reference
		// if enabled) will be doubled. This is done to test transient response
		// of the controllers
		if( ++count == SPEEDDELAY )
	    {
		   		VelReq = ReadADCParm.qADValue + Q15((OMEGA10 + OMEGA1)/2.0);

			if((uGF.bit.ChangeSpeed) && (CtrlParm.qVelRef <= VelReq)) // 2x speed
			{
			     CtrlParm.qVelRef += SPEEDDELAY;
			}
			else if (CtrlParm.qVelRef <= VelReq/2) //normal speed
			{
			     CtrlParm.qVelRef += SPEEDDELAY;
			}
			else CtrlParm.qVelRef -= SPEEDDELAY ;
	 	count = 0;
		}

		// When it first transition from open to closed loop, this If statement is
		// executed
        if( uGF.bit.ChangeMode )
        {
            // just changed from openloop
            uGF.bit.ChangeMode = 0;
			// An initial value is set for the speed controller accumulation.
			//
			// The first time the speed controller is executed, we want the output
			// to be the same as it was the last time open loop was executed. So,
			// last time open loop was executed, torque refefernce was constant,
			// and set to CtrlParm.qVqRef.
			//
			// First time in closed loop, CtrlParm.qVqRef = PIParmW.qdSum >> 16
			// assuming the error is zero at time zero. This is why we set
			// PIParmW.qdSum = (long)CtrlParm.qVqRef << 16.
			PIParmW.qdSum = (long)CtrlParm.qVqRef << 16;
			Startup_Lock = 0;
			Startup_Ramp = 0;
				//velocity reference ramp begins at minimum speed
			CtrlParm.qVelRef = MINSPEEDINRPM;

	    }

        // Check to see if new velocity information is available by comparing
        // the number of interrupts per velocity calculation against the
        // number of velocity count samples taken.  If new velocity info
        // is available, calculate the new velocity value and execute
        // the speed control loop.

        if(AccumThetaCnt == 0)
        {
        	// Execute the velocity control loop
			PIParmW.qInMeas = smc1.Omega;
        	PIParmW.qInRef  = CtrlParm.qVelRef;
        	CalcPI(&PIParmW);
        	CtrlParm.qVqRef = PIParmW.qOut;
        }

        // If the application is running in torque mode, the velocity
        // control loop is bypassed.  The velocity reference value, read
        // from the potentiometer, is used directly as the torque
        // reference, VqRef. This feature is enabled automatically only if
		// #define TORQUEMODE is defined in UserParms.h. If this is not
		// defined, uGF.bit.EnTorqueMod bit can be set in debug mode to enable
		// torque mode as well.

		if (uGF.bit.EnTorqueMod)
			CtrlParm.qVqRef = CtrlParm.qVelRef;

		// Get Id reference from Field Weakening table. If Field weakening
		// is not needed or user does not want to enable this feature,
		// let NOMINALSPEEDINRPM be equal to FIELDWEAKSPEEDRPM in
		// UserParms.h
		//~ CtrlParm.qVdRef = FieldWeakening(_Q15abs(CtrlParm.qVelRef));
		CtrlParm.qVdRef = FieldWeakening(abs(CtrlParm.qVelRef));

        // PI control for D
        PIParmD.qInMeas = ParkParm.qId;
        PIParmD.qInRef  = CtrlParm.qVdRef;
        CalcPI(&PIParmD);

		// If voltage ripple compensation flag is set, adjust the output
		// of the D controller depending on measured DC Bus voltage. This
		// feature is enabled automatically only if #define ENVOLTRIPPLE is
		// defined in UserParms.h. If this is not defined, uGF.bit.EnVoltRipCo
		// bit can be set in debug mode to enable voltage ripple compensation.
		//
		// NOTE:
		//
		// If Input power supply has switching frequency noise, for example if a
		// switch mode power supply is used, Voltage Ripple Compensation is not
		// recommended, since it will generate spikes on Vd and Vq, which can
		// potentially make the controllers unstable.
		//~ if(uGF.bit.EnVoltRipCo)
        	//~ ParkParm.qVd = VoltRippleComp(PIParmD.qOut);
		//~ else
			ParkParm.qVd = PIParmD.qOut;

		// Vector limitation
		// Vd is not limited
		// Vq is limited so the vector Vs is less than a maximum of 95%.
		// Vs = SQRT(Vd^2 + Vq^2) < 0.95
		// Vq = SQRT(0.95^2 - Vd^2)
		//~ qVdSquared = FracMpy(ParkParm.qVd, ParkParm.qVd);
       	//~ PIParmQ.qOutMax = _Q15sqrt(Q15(0.95*0.95) - qVdSquared);
		qVdSquared = ParkParm.qVd * ParkParm.qVd;
		PIParmQ.qOutMax = sqrtt(Q15(0.95*0.95) - qVdSquared, SQRT_ERROR_MAX);
		PIParmQ.qOutMin = -PIParmQ.qOutMax;

        // PI control for Q
        PIParmQ.qInMeas = ParkParm.qIq;
        PIParmQ.qInRef  = CtrlParm.qVqRef;
        CalcPI(&PIParmQ);

		// If voltage ripple compensation flag is set, adjust the output
		// of the Q controller depending on measured DC Bus voltage
		//~ if(uGF.bit.EnVoltRipCo)
        	//~ ParkParm.qVq = VoltRippleComp(PIParmQ.qOut);
		//~ else
        	ParkParm.qVq = PIParmQ.qOut;

		// Limit, if motor is stalled, stop motor commutation
		if (smc1.OmegaFltred < 0)
		{
			uGF.bit.RunMotor = 0;
        }
	}
}

//---------------------------------------------------------------------
// The ADC ISR does speed calculation and executes the vector update loop.
// The ADC sample and conversion is triggered by the PWM period.
// The speed calculation assumes a fixed time interval between calculations.
//---------------------------------------------------------------------

//~ void __attribute__((interrupt, no_auto_psv)) _ADC1Interrupt(void)
void __attribute__((interrupt)) pwm_interrupt_pic(void)
{
    //~ IFS0bits.AD1IF = 0;

    // Increment count variable that controls execution
    // of display and button functions.
    iADCisrCnt++;

    if( uGF.bit.RunMotor )
    {
            // Calculate qIa,qIb
            MeasCompCurr();

            // Calculate commutation angle using estimator
            CalculateParkAngle();

            // Calculate qId,qIq from qSin,qCos,qIa,qIb
            ClarkePark();

            // Calculate control values
            DoControl();

            // Calculate qSin,qCos from qAngle
            SinCos();

            // Calculate qValpha, qVbeta from qSin,qCos,qVd,qVq
            InvPark();

            // Calculate Vr1,Vr2,Vr3 from qValpha, qVbeta
            CalcRefVec();

            // Calculate and set PWM duty cycles from Vr1,Vr2,Vr3
            CalcSVGen();
    }
	return;
}

//---------------------------------------------------------------------
bool SetupParm(void)
{
    // Turn saturation on to insure that overflows will be handled smoothly.
    //~ CORCONbits.SATA  = 0;

    // Setup required parameters

// ============= Open Loop ======================
	// Motor End Speed Calculation
	// MotorParm.EndSpeed = ENDSPEEDOPENLOOP * POLEPAIRS * LOOPTIMEINSEC * 65536 * 65536 / 60.0;
	// Then, * 65536 which is a right shift done in "void CalculateParkAngle(void)"
	// ParkParm.qAngle += (int)(Startup_Ramp >> 16);
	MotorParm.EndSpeed = ENDSPEEDOPENLOOP * POLEPAIRS * LOOPTIMEINSEC * 65536 * 65536 / 60.0;
	MotorParm.LockTime = LOCKTIME;

// ============= ADC - Measure Current & Pot ======================

    // Scaling constants: Determined by calibration or hardware design.
    ReadADCParm.qK      = DQK;

    MeasCurrParm.qKa    = DQKA;
    MeasCurrParm.qKb    = DQKB;

    // Initial Current offsets
	//~ InitMeasCompCurr( ADC1BUF1, ADC1BUF2 );

	// Target DC Bus, without sign.
	//~ TargetDCbus = ((SFRAC16)ADC1BUF3 >> 1) + Q15(0.5);

// ============= SVGen ===============
    // Set PWM period to Loop Time
    SVGenParm.iPWMPeriod = LOOPINTCY;

/*
// ============= Motor PWM ======================

    PDC1 = 0;
    PDC2 = 0;
    PDC3 = 0;

    // Center aligned PWM.
    // Note: The PWM period is set to dLoopInTcy/2 but since it counts up and
    // and then down => the interrupt flag is set to 1 at zero => actual
    // interrupt period is dLoopInTcy

    PTPER = LOOPINTCY/2;   // Setup PWM period to Loop Time defined in parms.h

    PWMCON1 = 0x0077;       // Enable PWM 1,2,3 pairs for complementary mode
    DTCON1 = (0x40 | (DDEADTIME/2));     // Dead time
    DTCON2 = 0;
//    FLTACON = 0;            // PWM fault pins not used
	__builtin_write_OSCCONL(OSCCON & ~(1<<6)); // UNLOCK
	RPINR12bits.FLTA1R = 8; // Select pin-44 (RP8) as input for fault
	__builtin_write_OSCCONL(OSCCON | (1<<6)); // LOCK

	FLTACON = 0x0087; // All PWM into inactive state, cycle-by-cycle, PWM1, 2, 3 controlled by fault-A

	IFS3bits.PWM1IF = 0;
	IEC3bits.PWM1IE = 0;

    PTCON = 0x8002;         // Enable PWM for center aligned operation

    // SEVTCMP: Special Event Compare Count Register
    // Phase of ADC capture set relative to PWM cycle: 80 offset and counting down
	// to avoid ripple on the current measurement
	SEVTCMP = PTPER - 80;
    SEVTCMPbits.SEVTDIR = 1;
*/

/*
// ============= ADC - Measure Current & Pot ======================
// ADC setup for simultanous sampling on
//      CH0=AN1, CH1=AN3, CH2=AN4, CH3=AN5.
// Sampling triggered by PWM and stored in signed fractional form.

    AD1CON1 = 0;

    // Signed fractional (DOUT = sddd dddd dd00 0000)
    AD1CON1bits.FORM = 3;
    // Motor Control PWM interval ends sampling and starts conversion
    AD1CON1bits.SSRC = 3;
    // Simultaneous Sample Select bit (only applicable when CHPS = 01 or 1x)
    // Samples CH0, CH1, CH2, CH3 simultaneously (when CHPS = 1x)
    // Samples CH0 and CH1 simultaneously (when CHPS = 01)
    AD1CON1bits.SIMSAM = 1;
    // Sampling begins immediately after last conversion completes.
    // SAMP bit is auto set.
    AD1CON1bits.ASAM = 1;


    AD1CON2 = 0;
    // Samples CH0, CH1, CH2, CH3 simultaneously (when CHPS = 1x)
    AD1CON2bits.CHPS = 2;


    AD1CON3 = 0;
    // A/D Conversion Clock Select bits = 8 * Tcy
    AD1CON3bits.ADCS = 15;


    / * ADCHS: ADC Input Channel Select Register * /
    AD1CHS0 = 0;
    // CH0 is AN8 for POT
    AD1CHS0bits.CH0SA = 8;
    // CH1 positive input is AN0, CH2 positive input is AN1, CH3 positive input is AN2
    AD1CHS123bits.CH123SA = 0;


    / * ADPCFG: ADC Port Configuration Register * /
    // Set all ports digital
	AD1PCFGL = 0xFFFF;
    AD1PCFGLbits.PCFG0 = 0;   // AN0 analog - IA
    AD1PCFGLbits.PCFG1 = 0;   // AN1 analog - IB
    AD1PCFGLbits.PCFG2 = 0;   // AN2 analog - VBUS
    AD1PCFGLbits.PCFG8 = 0;   // AN8 analog - POT

    / * ADCSSL: ADC Input Scan Select Register * /
    AD1CSSL = 0;
	AD1CON2bits.SMPI = 0;		//Interrupt after every conversion (DMA takes all four conversions)

    // Turn on A/D module
    AD1CON1bits.ADON = 1;
*/
    return 0;
}

void CalculateParkAngle(void)
{
 	smc1.Ialpha = ParkParm.qIalpha;
  	smc1.Ibeta = ParkParm.qIbeta;
    smc1.Valpha = ParkParm.qValpha;
    smc1.Vbeta = ParkParm.qVbeta;

	SMC_Position_Estimation(&smc1);

	if(uGF.bit.OpenLoop)
	{
		if (Startup_Lock < MotorParm.LockTime)
			Startup_Lock += 1;	// This variable is incremented until
								// lock time expires, them the open loop
								// ramp begins
		else if (Startup_Ramp < MotorParm.EndSpeed)
			// Ramp starts, and increases linearly until EndSpeed is reached.
			// After ramp, estimated theta is used to commutate motor.
			Startup_Ramp += DELTA_STARTUP_RAMP; //89.5
		else
		{
			// This section enables closed loop, right after open loop ramp.
            uGF.bit.ChangeMode = 1;
            uGF.bit.OpenLoop = 0;
			// Difference between force angle and estimated theta is saved,
			// so a soft transition is made when entering closed loop.
			Theta_error = ParkParm.qAngle - smc1.Theta;
		}
		ParkParm.qAngle += (int)(Startup_Ramp >> 16); //1.37e-3
	}
	else
	{
		// This value is used to transition from open loop to closed looop.
		// At the end of open loop ramp, there is a difference between
		// forced angle and estimated angle. This difference is stored in
		// Theta_error, and added to estimated theta (smc1.Theta) so the
		// effective angle used for commutating the motor is the same at
		// the end of open loop, and at the begining of closed loop.
		// This Theta_error is then substracted from estimated theta
		// gradually in increments of 0.05 degrees until the error is less
		// than 0.05 degrees.
		ParkParm.qAngle = smc1.Theta + Theta_error;
		//~ if (_Q15abs(Theta_error) > _0_05DEG)
		if(abs(Theta_error) > _0_05DEG)
		{
			if (Theta_error < 0)
				Theta_error += _0_05DEG;
			else
				Theta_error -= _0_05DEG;
		}
	}
	return;
}

void SetupControlParameters(void)
{

// ============= PI D Term ===============
    PIParmD.qKp = DKP;
    PIParmD.qKi = DKI;
    PIParmD.qKc = DKC;
    PIParmD.qOutMax = DOUTMAX;
    PIParmD.qOutMin = -PIParmD.qOutMax;

    InitPI(&PIParmD);

// ============= PI Q Term ===============
    PIParmQ.qKp = QKP;
    PIParmQ.qKi = QKI;
    PIParmQ.qKc = QKC;
    PIParmQ.qOutMax = QOUTMAX;
    PIParmQ.qOutMin = -PIParmQ.qOutMax;

    InitPI(&PIParmQ);

// ============= PI W Term ===============
    PIParmW.qKp = WKP;
    PIParmW.qKi = WKI;
    PIParmW.qKc = WKC;
    PIParmW.qOutMax = WOUTMAX;
    PIParmW.qOutMin = -PIParmW.qOutMax;

    InitPI(&PIParmW);
	return;
}

void DebounceDelay(void)
{
	long i;
	for (i = 0;i < 100000;i++);
	return;
}

// NOTE:
//
// If Input power supply has switching frequency noise, for example if a
// switch mode power supply is used, Voltage Ripple Compensation is not
// recommended, since it will generate spikes on Vd and Vq, which can
// potentially make the controllers unstable.
/*
SFRAC16 VoltRippleComp(SFRAC16 Vdq)
{
	SFRAC16 CompVdq;
	// DCbus is already updated with new DC Bus measurement
	// in ReadSignedADC0 subroutine.
	//
	// If target DC Bus is greater than what we measured last sample, adjust
	// output as follows:
	//
	//                  TargetDCbus - DCbus
	// CompVdq = Vdq + --------------------- * Vdq
	//                         DCbus
	//
	// If Measured DCbus is greater than target, then the following compensation
	// is implemented:
	//
	//            TargetDCbus
	// CompVdq = ------------- * Vdq
	//               DCbus
	//
	// If target and measured are equal, no operation is made.
	//
	if (TargetDCbus > DCbus)
		CompVdq = Vdq + FracMpy(FracDiv(TargetDCbus - DCbus, DCbus), Vdq);
	else if (DCbus > TargetDCbus)
		CompVdq = FracMpy(FracDiv(TargetDCbus, DCbus), Vdq);
	else
		CompVdq = Vdq;

	return CompVdq;
}
*/
