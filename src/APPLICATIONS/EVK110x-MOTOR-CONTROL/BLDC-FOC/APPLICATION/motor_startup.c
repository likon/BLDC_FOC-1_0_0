#include "motor_startup.h"

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
            unsigned OpenLoop:1;	// Indicates if motor is running in open or closed loop
            unsigned RunMotor:1;	// If motor is running, or stopped.
			unsigned EnTorqueMod:1;	// This bit enables Torque mode when running closed loop
			unsigned EnVoltRipCo:1;	// Bit that enables Voltage Ripple Compensation
            unsigned Btn1Pressed:1;	// Button 1 has been pressed.
            unsigned Btn2Pressed:1;	// Button 2 has been pressed.
            unsigned ChangeMode:1;	// This flag indicates that a transition from open to closed
									// loop, or closed to open loop has happened. This
									// causes DoControl subroutine to initialize some variables
									// before executing open or closed loop for the first time
            unsigned ChangeSpeed:1;	// This flag indicates a step command in speed reference.
									// This is mainly used to analyze step response
            unsigned    :8;
            }bit;
        WORD Word;
        } uGF;

tPIParm     PIParmD;	// Structure definition for Flux component of current, or Id
tPIParm     PIParmQ;	// Structure definition for Torque component of current, or Iq
tPIParm     PIParmW;	// Structure definition for Speed, or Omega

void SMCInit(SMC *s)
{
    //                R * Ts
    // Fsmopos = 1 - --------
    //                  L
    //            Ts
    // Gsmopos = ----
    //            L
    // Ts = Sampling Period. If sampling at PWM, Ts = 50 us
    // R = Phase Resistance. If not provided by motor datasheet,
    //     measure phase to phase resistance with multimeter, and
    //     divide over two to get phase resistance. If 4 Ohms are
    //     measured from phase to phase, then R = 2 Ohms
    // L = Phase inductance. If not provided by motor datasheet,
    //     measure phase to phase inductance with multimeter, and
    //     divide over two to get phase inductance. If 2 mH are
    //     measured from phase to phase, then L = 1 mH

	if (Q15(PHASERES * LOOPTIMEINSEC) > Q15(PHASEIND))
		s->Fsmopos = Q15(0.0);
	else
		s->Fsmopos = Q15(1 - PHASERES * LOOPTIMEINSEC / PHASEIND);

	if (Q15(LOOPTIMEINSEC) > Q15(PHASEIND))
		s->Gsmopos = Q15(0.99999);
	else
		s->Gsmopos = Q15(LOOPTIMEINSEC / PHASEIND);

	s->Kslide = Q15(SMCGAIN);
	s->MaxSMCError = Q15(MAXLINEARSMC);
	s->FiltOmCoef = Q15(OMEGA0 * _PI / IRP_PERCALC); // Cutoff frequency for omega filter
													 // is minimum omega, or OMEGA0

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

bool SetupParm(void)
{
    // Turn saturation on to insure that overflows will be handled smoothly.
    CORCONbits.SATA  = 0;

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
	InitMeasCompCurr( ADC1BUF1, ADC1BUF2 );

	// Target DC Bus, without sign.
	TargetDCbus = ((SFRAC16)ADC1BUF3 >> 1) + Q15(0.5);

// ============= SVGen ===============
    // Set PWM period to Loop Time
    SVGenParm.iPWMPeriod = LOOPINTCY;

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


    /* ADCHS: ADC Input Channel Select Register */
    AD1CHS0 = 0;
    // CH0 is AN8 for POT
    AD1CHS0bits.CH0SA = 8;
    // CH1 positive input is AN0, CH2 positive input is AN1, CH3 positive input is AN2
    AD1CHS123bits.CH123SA = 0;


    /* ADPCFG: ADC Port Configuration Register */
    // Set all ports digital
	AD1PCFGL = 0xFFFF;
    AD1PCFGLbits.PCFG0 = 0;   // AN0 analog - IA
    AD1PCFGLbits.PCFG1 = 0;   // AN1 analog - IB
    AD1PCFGLbits.PCFG2 = 0;   // AN2 analog - VBUS
    AD1PCFGLbits.PCFG8 = 0;   // AN8 analog - POT

    /* ADCSSL: ADC Input Scan Select Register */
    AD1CSSL = 0;
	AD1CON2bits.SMPI = 0;		//Interrupt after every conversion (DMA takes all four conversions)

    // Turn on A/D module
    AD1CON1bits.ADON = 1;

    return False;
}

void motor_startup(void)
{
	SMCInit(&smc1);
	SetupControlParameters();
	SetupParm();
	uGF.bit.RunMotor = 1;	//Start motor
	uGF.bit.ChangeMode = 1;	// Ensure variable initialization when open loop is
							// executed for the first time
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
			Startup_Ramp += DELTA_STARTUP_RAMP;
		else
		{
			// This section enables closed loop, right after open loop ramp.
            uGF.bit.ChangeMode = 1;
            uGF.bit.OpenLoop = 0;
			// Difference between force angle and estimated theta is saved,
			// so a soft transition is made when entering closed loop.
			Theta_error = ParkParm.qAngle - smc1.Theta;
		}
		ParkParm.qAngle += (int)(Startup_Ramp >> 16);
	}
}
