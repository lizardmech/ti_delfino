//----------------------------------------------------------------------------------
//	FILE:			IDDK_PM_Servo_F2837x-Settings.h
//
//	Description:	User settings
//
//	Version: 		1.0
//
//  Target:  		TMS320F28377D,
//
//----------------------------------------------------------------------------------
//  Copyright Texas Instruments © 2004-2015
//----------------------------------------------------------------------------------
//  Revision History:
//----------------------------------------------------------------------------------
//  Date	  | Description / Status
//----------------------------------------------------------------------------------
// 4 Nov 2015  - User settings
//----------------------------------------------------------------------------------

#ifndef PROJ_INCLUDE_H
#define PROJ_INCLUDE_H

/*-------------------------------------------------------------------------------
Include project specific include files.
-------------------------------------------------------------------------------*/
// define math type as float(1)
#define   MATH_TYPE      1
#include "IQmathLib.h"
#include "F28x_Project.h"

#include "smopos.h"       		// Include header for the SMOPOS object 
#include "smopos_const.h"       // Include header for the SMOPOS object
#include "park.h"       		// Include header for the PARK object 
#include "ipark.h"       		// Include header for the IPARK object 
#include "pi.h"       			// Include header for the PIDREG3 object 
#include "clarke.h"         	// Include header for the CLARKE object 
#include "svgen.h"		       	// Include header for the SVGENDQ object 
#include "rampgen.h"        	// Include header for the RAMPGEN object 
#include "rmp_cntl.h"       	// Include header for the RMPCNTL object 
#include "volt_calc.h"      	// Include header for the PHASEVOLTAGE object 
#include "speed_est.h"          // Include header for the SPEED_ESTIMATION object 
#include "speed_fr.h"			// Include header for the SPEED_MEAS_QEP object
#include "resolver.h"
#include "pid_grando.h"
#include "pid_reg3.h"
#include <math.h>

#include "DLOG_4CH_F.h"

#include "F2837xD_sdfm_drivers.h"
#include "Resolver_Float.h"

#include "F2837x_QEP_Module.h"
#include "encoder-integration.h"

/*------------------------------------------------------------------------------
List of control GND configurations - COLD or HOT
------------------------------------------------------------------------------*/
#define COLD  1      	  // control GND is COLD
#define HOT   2           // control GND is HOT

/*------------------------------------------------------------------------------
This line sets the CGND configuration to one of the available choices.
------------------------------------------------------------------------------*/
#define  CGND  COLD

/*------------------------------------------------------------------------------
Following is the list of the Build Level choices.
------------------------------------------------------------------------------*/
#define LEVEL1  1      		// Module check out (do not connect the motors)
#define LEVEL2  2           // Verify ADC, park/clarke, calibrate the offset and speed measurement
#define LEVEL3  3           // Verify closed current(torque) loop and its PIs
#define LEVEL4  4           // Verify speed loop and speed PID
#define LEVEL5  5           // Verify position loop

/*------------------------------------------------------------------------------
This line sets the BUILDLEVEL to one of the available choices.
------------------------------------------------------------------------------*/
#define   BUILDLEVEL LEVEL1

/*------------------------------------------------------------------------------
Following is the list of Current sense options
------------------------------------------------------------------------------*/
#define SHUNT_CURRENT_SENSE 1
#define LEM_CURRENT_SENSE   2
#define SD_CURRENT_SENSE    3

/*------------------------------------------------------------------------------
This line sets the CURRENT_SENSE to user's choice from the above.
------------------------------------------------------------------------------*/
#define CURRENT_SENSE LEM_CURRENT_SENSE

/*------------------------------------------------------------------------------
Following is the list of Position Encoder options
------------------------------------------------------------------------------*/
// Select Position Feedback Option
#define QEP_POS_ENCODER       1
#define RESOLVER_POS_ENCODER  2
#define BISS_POS_ENCODER      3
#define ENDAT_POS_ENCODER     4
#define SINCOS_POS_ENCODER    5

/*------------------------------------------------------------------------------
This line sets the POSITION_ENCODER to user's choice from the above.
------------------------------------------------------------------------------*/
#define POSITION_ENCODER QEP_POS_ENCODER

#ifndef BUILDLEVEL
#error  Critical: BUILDLEVEL must be defined !!
#endif  // BUILDLEVEL
//------------------------------------------------------------------------------


#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

#define PI 3.14159265358979

// Define the system frequency (MHz)
#if (DSP2803x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 60
#elif (DSP280x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 100
#elif (F28_2837xD==1)
#define SYSTEM_FREQUENCY 200
#endif

// Timer definitions based on System Clock
// 150 MHz devices
#define		mSec0_5		0.5*SYSTEM_FREQUENCY*1000		// 0.5 mS
#define		mSec1		1*SYSTEM_FREQUENCY*1000		// 1.0 mS
#define		mSec2		2.0*SYSTEM_FREQUENCY*1000		// 2.0 mS
#define		mSec5		5*SYSTEM_FREQUENCY*1000		// 5.0 mS
#define		mSec7_5		7.5*SYSTEM_FREQUENCY*1000		// 7.5 mS
#define		mSec10		10*SYSTEM_FREQUENCY*1000		// 10 mS
#define		mSec20		20*SYSTEM_FREQUENCY*1000		// 20 mS
#define		mSec50		50*SYSTEM_FREQUENCY*1000		// 50 mS
#define		mSec100		100*SYSTEM_FREQUENCY*1000		// 100 mS
#define		mSec500		500*SYSTEM_FREQUENCY*1000	// 500 mS
#define		mSec1000	1000*SYSTEM_FREQUENCY*1000	// 1000 mS

// Define the ISR frequency (kHz)
#define ISR_FREQUENCY 10
#define INV_PWM_TICKS  ((SYSTEM_FREQUENCY/2.0)/ISR_FREQUENCY)*1000
#define INV_PWM_TBPRD INV_PWM_TICKS/2
#define INV_PWM_HALF_TBPRD INV_PWM_TICKS/4

// Resolver Related defines
#define RESOLVER_EXC_SAMPLE_FREQUENCY   160   // in KHz
#define RESOLVER_EXC_FREQUENCY           10   // in KHz
#define RESOLVER_PWM_TICKS  ((SYSTEM_FREQUENCY/2.0)/RESOLVER_EXC_SAMPLE_FREQUENCY)*1000

#define  TWO_PI                  (2*PI)
#define  DELAY_LENGTH            16

#define  FIR32_SEL    0   /* 1 - Implement 33 tap FIR, samples  over 2 cycles
                             0 - Implement 17 tap FIR, samples  over 1 cycle   */

#define  TUNING_SEL   0   /* 1 - bypass atan value - to tune PI coefficients
                             0 - use atan value*/

// RESOLVER specs
#define RESOLVER_STEPS_PER_TURN         4096       // Resolver's discrete steps/turn
#define RESOLVER_STEPS_PER_POLEPAIR    (RESOLVER_STEPS_PER_TURN/(POLES/2))

#define  POS_KI_LOW_SPD   0.8// _IQ(0.0001)
#define  POS_KI_MED_SPD   1.0//_IQ(0.001)
#define  POS_KI_HI_SPD    1.2//_IQ(0.01)


/*------------------------------------------------------------------------------
set the motor parameters to the one available
------------------------------------------------------------------------------*/

// Define the electrical motor parametes (Estun Servomotor)
#define RS 		2.35		    	    // Stator resistance (ohm)
#define RR   			               	// Rotor resistance (ohm)
#define LS   	0.0065					// Stator inductance (H)
#define LR   			  				// Rotor inductance (H)
#define LM   			   				// Magnatizing inductance (H)
#define POLES  	8						// Number of poles
										// Number of poles

// Define the base quantites
#define BASE_VOLTAGE        236.14    // Base peak phase voltage (volt), Vdc/sqrt(3)
#define BASE_SHUNT_CURRENT    9.95    // Base peak phase current (amp), Max. measurable peak curr.
#define BASE_LEM_CURRENT     12.0     //  ----- do -----
#define BASE_TORQUE     		      // Base torque (N.m)
#define BASE_FLUX       			  // Base flux linkage (volt.sec/rad)
#define BASE_FREQ      	200           // Base electrical frequency (Hz)


/*-----------------------------------------------------------------------------
 * Sigma Delta Filter Module - frame setting
 * ----------------------------------------------------------------------------
 */
#define  SDFM_TICKS    5
#define  OSR_RATE      OSR_128

/*------------------------------------------------------------------------------
Current sensors scaling
------------------------------------------------------------------------------*/
// LEM    1.0pu current ==> 12.0A -> 2048 counts ==> 8A -> 1365
// SHUNT  1.0pu current ==> 9.95A -> 2048 counts ==> 8A -> 1647
#define LEM(A)     2048*A/BASE_LEM_CURRENT
#define SHUNT(A)   2048*A/BASE_SHUNT_CURRENT

// ADC Configuration
//definitions for selecting ADC resolution
#define RESOLUTION_12BIT   0 //12-bit resolution
#define RESOLUTION_16BIT   1 //16-bit resolution (not supported for all variants)

//definitions for selecting ADC signal mode
#define SIGNAL_SINGLE          0 //single-ended channel conversions (12-bit mode only)
#define SIGNAL_DIFFERENTIAL    1 //differential pair channel conversions

#define ADC_PU_SCALE_FACTOR        0.000244140625     //1/2^12
#define ADC_PU_PPB_SCALE_FACTOR    0.000488281250     //1/2^11
#define SD_PU_SCALE_FACTOR         0.000030517578125  // 1/2^15

#define REFERENCE_VDAC     0
#define REFERENCE_VREF     1

// ADC Related defines
#define IFB_SV AdcaResultRegs.ADCRESULT0
#define IFB_SW AdcbResultRegs.ADCRESULT0
#define IFB_SV_PPB ((signed int)AdcaResultRegs.ADCPPB1RESULT.all)
#define IFB_SW_PPB ((signed int)AdcbResultRegs.ADCPPB1RESULT.all)

#define R_SIN  AdcdResultRegs.ADCRESULT0
#define R_COS  AdccResultRegs.ADCRESULT0
#define R_SIN_PPB ((signed int)AdcdResultRegs.ADCPPB1RESULT.all)
#define R_COS_PPB ((signed int)AdccResultRegs.ADCPPB1RESULT.all)

#define IFB_LEMV AdcaResultRegs.ADCRESULT1
#define IFB_LEMW AdcbResultRegs.ADCRESULT1
#define IFB_LEMV_PPB ((signed int)AdcaResultRegs.ADCPPB2RESULT.all)
#define IFB_LEMW_PPB ((signed int)AdcbResultRegs.ADCPPB2RESULT.all)


// ************************************************************************
// Scaling factors to bring all current feedbacks to normal scale
//   matching with shunt based measurement
//  With shunt, 1.0pu current  == 9.945A
//       LEM,   1.0pu current  == 12A
//       SDFM,  0.8906pu current  == 12.5A
// ************************************************************************
#define  LEM_TO_SHUNT    1.206637   // (12.0/9.945)
#define  SDFM_TO_SHUNT   1.41131    // (12.5/0.8906)/9.945

/*****************************************************************************
 * ***************************************************************************
 */
#endif
