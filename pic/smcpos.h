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
 *    Filename:       smcpos.h                                         *
 *    Date:           10/01/08                                         *
 *                                                                     *
 *    Tools used:     MPLAB IDE -> 8.14                                *
 *                    C30 -> 3.10                                      *
 *    Linker File:    p33FJ32MC204.gld                                 *
 *                                                                     *
 **********************************************************************/
#ifndef smcpos_H
#define smcpos_H

#include "UserParms.h"

typedef struct	{  
				SFRAC16  Valpha;   		// Input: Stationary alfa-axis stator voltage 
                SFRAC16  Ealpha;   		// Variable: Stationary alfa-axis back EMF 
				SFRAC16  EalphaFinal;	// Variable: Filtered EMF for Angle calculation
                SFRAC16  Zalpha;      	// Output: Stationary alfa-axis sliding control 
                SFRAC16  Gsmopos;    	// Parameter: Motor dependent control gain 
                SFRAC16  EstIalpha;   	// Variable: Estimated stationary alfa-axis stator current 
                SFRAC16  Fsmopos;    	// Parameter: Motor dependent plant matrix 
                SFRAC16  Vbeta;   		// Input: Stationary beta-axis stator voltage 
                SFRAC16  Ebeta;  		// Variable: Stationary beta-axis back EMF 
				SFRAC16  EbetaFinal;	// Variable: Filtered EMF for Angle calculation
                SFRAC16  Zbeta;      	// Output: Stationary beta-axis sliding control 
                SFRAC16  EstIbeta;    	// Variable: Estimated stationary beta-axis stator current 
                SFRAC16  Ialpha;  		// Input: Stationary alfa-axis stator current 
                SFRAC16  IalphaError; 	// Variable: Stationary alfa-axis current error                 
                SFRAC16  Kslide;     	// Parameter: Sliding control gain 
                SFRAC16  MaxSMCError;  	// Parameter: Maximum current error for linear SMC 
                SFRAC16  Ibeta;  		// Input: Stationary beta-axis stator current 
                SFRAC16  IbetaError;  	// Variable: Stationary beta-axis current error                 
                SFRAC16  Kslf;       	// Parameter: Sliding control filter gain 
                SFRAC16  KslfFinal;    	// Parameter: BEMF Filter for angle calculation
                SFRAC16  FiltOmCoef;   	// Parameter: Filter Coef for Omega filtered calc
				SFRAC16  ThetaOffset;	// Output: Offset used to compensate rotor angle
                SFRAC16  Theta;			// Output: Compensated rotor angle 
				SFRAC16  Omega;     	// Output: Rotor speed
				SFRAC16  OmegaFltred;  	// Output: Filtered Rotor speed for speed PI
				} SMC;	            

typedef SMC *SMC_handle;

#define SMC_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}

// Define this in RPMs

#define SPEED0 MINSPEEDINRPM
#define SPEED1 (SPEED0 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED2 (SPEED1 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED3 (SPEED2 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED4 (SPEED3 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED5 (SPEED4 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED6 (SPEED5 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED7 (SPEED6 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED8 (SPEED7 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED9 (SPEED8 + (int)((FIELDWEAKSPEEDRPM - MINSPEEDINRPM) / 10.0))
#define SPEED10 (FIELDWEAKSPEEDRPM)

// Define this in Degrees, from 0 to 360

#define THETA_AT_SPEED0 50  // 30 for 2 pole pair motors with F=1.0 and G=0.1
#define THETA_AT_SPEED1 50
#define THETA_AT_SPEED2 70
#define THETA_AT_SPEED3 80
#define THETA_AT_SPEED4 95
#define THETA_AT_SPEED5 110
#define THETA_AT_SPEED6 110
#define THETA_AT_SPEED7 110
#define THETA_AT_SPEED8 110
#define THETA_AT_SPEED9 110
#define THETA_AT_SPEED10 110
#define THETA_AT_ALL_SPEED 90

#define OMEGA0 (float)(SPEED0 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA1 (float)(SPEED1 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA2 (float)(SPEED2 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA3 (float)(SPEED3 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA4 (float)(SPEED4 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA5 (float)(SPEED5 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA6 (float)(SPEED6 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA7 (float)(SPEED7 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA8 (float)(SPEED8 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA9 (float)(SPEED9 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGA10 (float)(SPEED10 * LOOPTIMEINSEC * \
                IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)


#define OMEGANOMINAL	(float)(NOMINALSPEEDINRPM * LOOPTIMEINSEC * \
                		IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)
#define OMEGAFIELDWK	(float)(FIELDWEAKSPEEDRPM * LOOPTIMEINSEC * \
                		IRP_PERCALC * POLEPAIRS * 2.0 / 60.0)

#define THETA0 (float)(THETA_AT_SPEED0 * 180.0 / 32768.0)
#define THETA1 (float)(THETA_AT_SPEED1 * 180.0 / 32768.0)
#define THETA2 (float)(THETA_AT_SPEED2 * 180.0 / 32768.0)
#define THETA3 (float)(THETA_AT_SPEED3 * 180.0 / 32768.0)
#define THETA4 (float)(THETA_AT_SPEED4 * 180.0 / 32768.0)
#define THETA5 (float)(THETA_AT_SPEED5 * 180.0 / 32768.0)
#define THETA6 (float)(THETA_AT_SPEED6 * 180.0 / 32768.0)
#define THETA7 (float)(THETA_AT_SPEED7 * 180.0 / 32768.0)
#define THETA8 (float)(THETA_AT_SPEED8 * 180.0 / 32768.0)
#define THETA9 (float)(THETA_AT_SPEED9 * 180.0 / 32768.0)
#define THETA10 (float)(THETA_AT_SPEED10 * 180.0 / 32768.0)
#define THETA_ALL (float)(THETA_AT_ALL_SPEED * 180.0 / 32768.0)

#define SLOPECALC0 (float)(THETA0 / OMEGA0)
#define SLOPECALC1 (float)((THETA1 - THETA0) / (OMEGA1 -OMEGA0))
#define SLOPECALC2 (float)((THETA2 - THETA1) / (OMEGA2 -OMEGA1))
#define SLOPECALC3 (float)((THETA3 - THETA2) / (OMEGA3 -OMEGA2))
#define SLOPECALC4 (float)((THETA4 - THETA3) / (OMEGA4 -OMEGA3))
#define SLOPECALC5 (float)((THETA5 - THETA4) / (OMEGA5 -OMEGA4))
#define SLOPECALC6 (float)((THETA6 - THETA5) / (OMEGA6 -OMEGA5))
#define SLOPECALC7 (float)((THETA7 - THETA6) / (OMEGA7 -OMEGA6))
#define SLOPECALC8 (float)((THETA8 - THETA7) / (OMEGA8 -OMEGA7))
#define SLOPECALC9 (float)((THETA9 - THETA8) / (OMEGA9 -OMEGA8))
#define SLOPECALC10 (float)((THETA10 - THETA9) / (OMEGA10 -OMEGA9))

#define LINEOFFSET0 (float)(THETA0 - OMEGA0 * SLOPECALC0)
#define LINEOFFSET1 (float)(THETA1 - OMEGA1 * SLOPECALC1)
#define LINEOFFSET2 (float)(THETA2 - OMEGA2 * SLOPECALC2)
#define LINEOFFSET3 (float)(THETA3 - OMEGA3 * SLOPECALC3)
#define LINEOFFSET4 (float)(THETA4 - OMEGA4 * SLOPECALC4)
#define LINEOFFSET5 (float)(THETA5 - OMEGA5 * SLOPECALC5)
#define LINEOFFSET6 (float)(THETA6 - OMEGA6 * SLOPECALC6)
#define LINEOFFSET7 (float)(THETA7 - OMEGA7 * SLOPECALC7)
#define LINEOFFSET8 (float)(THETA8 - OMEGA8 * SLOPECALC8)
#define LINEOFFSET9 (float)(THETA9 - OMEGA9 * SLOPECALC9)
#define LINEOFFSET10 (float)(THETA10 - OMEGA10 * SLOPECALC10)

#define SLOPEINT0 (int)(SLOPECALC0)
#define SLOPEINT1 (int)(SLOPECALC1)
#define SLOPEINT2 (int)(SLOPECALC2)
#define SLOPEINT3 (int)(SLOPECALC3)
#define SLOPEINT4 (int)(SLOPECALC4)
#define SLOPEINT5 (int)(SLOPECALC5)
#define SLOPEINT6 (int)(SLOPECALC6)
#define SLOPEINT7 (int)(SLOPECALC7)
#define SLOPEINT8 (int)(SLOPECALC8)
#define SLOPEINT9 (int)(SLOPECALC9)
#define SLOPEINT10 (int)(SLOPECALC10)

#define SLOPEFRAC0 Q15(SLOPECALC0 - SLOPEINT0)
#define SLOPEFRAC1 Q15(SLOPECALC1 - SLOPEINT1)
#define SLOPEFRAC2 Q15(SLOPECALC2 - SLOPEINT2)
#define SLOPEFRAC3 Q15(SLOPECALC3 - SLOPEINT3)
#define SLOPEFRAC4 Q15(SLOPECALC4 - SLOPEINT4)
#define SLOPEFRAC5 Q15(SLOPECALC5 - SLOPEINT5)
#define SLOPEFRAC6 Q15(SLOPECALC6 - SLOPEINT6)
#define SLOPEFRAC7 Q15(SLOPECALC7 - SLOPEINT7)
#define SLOPEFRAC8 Q15(SLOPECALC8 - SLOPEINT8)
#define SLOPEFRAC9 Q15(SLOPECALC9 - SLOPEINT9)
#define SLOPEFRAC10 Q15(SLOPECALC10 - SLOPEINT10)

#define CONSTANT0 Q15(LINEOFFSET0)
#define CONSTANT1 Q15(LINEOFFSET1)
#define CONSTANT2 Q15(LINEOFFSET2)
#define CONSTANT3 Q15(LINEOFFSET3)
#define CONSTANT4 Q15(LINEOFFSET4)
#define CONSTANT5 Q15(LINEOFFSET5)
#define CONSTANT6 Q15(LINEOFFSET6)
#define CONSTANT7 Q15(LINEOFFSET7)
#define CONSTANT8 Q15(LINEOFFSET8)
#define CONSTANT9 Q15(LINEOFFSET9)
#define CONSTANT10 Q15(LINEOFFSET10)

#define DEFAULTCONSTANT Q15(THETA6)
#define CONSTANT_PHASE_SHIFT Q15(THETA_ALL)

#define PUSHCORCON()  {__asm__ volatile ("push CORCON");}
#define POPCORCON()   {__asm__ volatile ("pop CORCON");}
#define _PI 3.1416

void SMC_Position_Estimation(SMC_handle);
void SMCInit(SMC_handle);
void CalcEstI(void);
void CalcIError(void);
void CalcZalpha(void);
void CalcZbeta(void);
void CalcBEMF(void);
void CalcOmegaFltred(void);
SFRAC16 FracMpy(SFRAC16 mul_1, SFRAC16 mul_2);
SFRAC16 FracDiv(SFRAC16 num_1, SFRAC16 den_1);

extern SFRAC16 PrevTheta;
extern SFRAC16 AccumTheta;
extern WORD AccumThetaCnt;

#endif
