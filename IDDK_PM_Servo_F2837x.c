/* ============================================================================
System Name:  	IDDK PMSM Servo Control

File Name:	  	IDDK_PM_Servo_F2837x.C

Target:			Rev 1.3 of the F2837x control card

Mother board :  Industrial Drive Development Kit - IDDK - R2.2.1

Author:			C2000 Systems Lab, 06th Novemeber 2015

Description:	Project uses the following ISRs
				1. Resolver ISR triggered by ADC INT, this is defined as the
				     high priority interrupt and runs at 160Khz
				2. Inverter ISR rate is 10Khz and is triggered by EPM11,
				     reason to choose EPWM11 is because of the SD reading
				     EPWM11 provides periodic sync resets to the SDFM module,
				     after each reset the SDFM data is ready after 3 OSRs and
				     needs to be read immediately.
				3. SPI ISR --> for EnDAT and BiSS position encoder interface

===========================================================================  */
//----------------------------------------------------------------------------------
//  Copyright Texas Instruments © 2004-2015
//----------------------------------------------------------------------------------
//  Revision History:
//  v1.0 - Initial Release
//  v2.0 - Addition of EnDAT absolute encoder interface library and code
//       - Addition of BiSS-C encoder interfce library and code
//       - Bug fix in over current protection algorithm
//         - new variable 'curLimit' added, which the user can set up at actuals
//           set to 10.0 if protection needed at 10A
//       - Default code set for COLD control GND configuration
//           (SHUNT currents and voltages can be sensed in HOT config only)
//       - device support library changed from v150 to v170
//         - TrigRegs changed to SyncSocRegs
//
//----------------------------------------------------------------------------------
//  Date	  | Description / Status
//----------------------------------------------------------------------------------
// 4 Nov 2015  - Example project for PM BiSS-C Library Usage
//----------------------------------------------------------------------------------


/**********************************************************************************
 * Peripheral functions:
   EPWMs
		- EPWM1, EPWM2, EPWM3 ---> Inverter PWMs for phases A, B, C
		- EPWM5  ---> clk for Sigma Delta
		- EPWM6  ---> Resolver feedback sampling @ 160KHz
		- EPWM11 ---> sync SD filter windows with motor control PWMs
		- EPWM4  ---> Not available for users if EnDAT / BiSS interface is active

	SPIs
		- SPIB  ---> Not available for users if EnDAT / BiSS interface is active

	Analog to Digital Conversion channels
	  ADC A4/C+  --->  Ifb-SV
	  ADC B4/C+  ---> Ifb-SW
	  ADC A2/C+  ---> LEM V
	  ADC B2/C+  ---> LEM W
	  ADC D1     ---> R_SIN
	  ADC C1     ---> R_COS
	  ADC C3     ---> Vfb-U
	  ADC A3     ---> Vfb-V
	  ADC B3     ---> Vfb-W
	  ADC B0     ---> Vfb-Bus

	 Analog Signals brought in but not sampled
	  ADC C2/C+  ---> Ifb-SU
	  ADC A5     --->
	  ADC C0/C+  ---> SC-A2
	  ADC D0/C+  ---> SC-B2
	  ADC D2/C+  ---> SC-R

	  DAC-A  ---> Resolver carrier excitation
	  DAC-B  ---> General purpose display
	  DAC-C  ---> General purpose display

 ********************************************************************************
 */
// Include header files used in the main function
// define float maths and then include IQmath library

#include "IDDK_PM_Servo_F2837x-Settings.h"

#ifdef _FLASH
#pragma CODE_SECTION(MotorControlISR,"ramfuncs");
#pragma CODE_SECTION(ResolverISR,"ramfuncs");
#endif

#pragma INTERRUPT (ResolverISR, HPI)
#pragma INTERRUPT (MotorControlISR, LPI)

// Prototype statements for functions found within this file.
interrupt void MotorControlISR(void);
interrupt void ResolverISR(void);
void DeviceInit();
void MemCopy();
void InitFlash();
void HVDMC_Protection(void);
void PWM_1ch_UpDwnCnt_CNF(int16 n, Uint16 period, int16 db);
void PWM_1ch_UpCnt_CNF(int16 n, Uint16 period);
void ConfigureADC(void);

// **********************************************************
// ********************* Functions **************************
// **********************************************************
_iq refPosGen(_iq out);
_iq ramper(_iq in, _iq out, _iq rampDelta);

// State Machine function prototypes
//------------------------------------
// Alpha states
void A0(void);	//state A0
void B0(void);	//state B0
void C0(void);	//state C0

// A branch states
void A1(void);	//state A1
void A2(void);	//state A2
void A3(void);	//state A3

// B branch states
void B1(void);	//state B1
void B2(void);	//state B2
void B3(void);	//state B3

// C branch states
void C1(void);	//state C1
void C2(void);	//state C2
void C3(void);	//state C3

// Variable declarations
void (*Alpha_State_Ptr)(void);	// Base States pointer
void (*A_Task_Ptr)(void);		// State pointer A branch
void (*B_Task_Ptr)(void);		// State pointer B branch
void (*C_Task_Ptr)(void);		// State pointer C branch

// ****************************************************************************
// Variables for CPU control
// ****************************************************************************
// adc static cal
int *adc_cal;

// Used to indirectly access all EPWM modules
volatile struct EPWM_REGS *ePWM[] = {
		&EPwm1Regs,			//intentional: (ePWM[0] not used)
		&EPwm1Regs, &EPwm2Regs, &EPwm3Regs, &EPwm4Regs, &EPwm5Regs, &EPwm6Regs,
		&EPwm7Regs, &EPwm8Regs, &EPwm9Regs, &EPwm10Regs, &EPwm11Regs, &EPwm12Regs};

volatile struct SDFM_REGS *SDFM[] =
{	0, &Sdfm1Regs, &Sdfm2Regs};

// Used to indirectly access eQEP module
volatile struct EQEP_REGS *eQEP[] =
 				  { &EQep1Regs,
 				  	&EQep1Regs,
					&EQep2Regs,
				  };

int16	VTimer0[4];			// Virtual Timers slaved off CPU Timer 0 (A events)
int16	VTimer1[4]; 		// Virtual Timers slaved off CPU Timer 1 (B events)
int16	VTimer2[4]; 		// Virtual Timers slaved off CPU Timer 2 (C events)
int16	SerialCommsTimer;

// Default ADC initialization
int ChSel[16]   = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int	TrigSel[16] = {5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5};
int ACQPS[16]   = {8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};

//*********************** USER Variables *************************************
// Global variables used in this system
//****************************************************************************

// ****************************************************************************
// Variables for current measurement
// ****************************************************************************
// Offset calibration routine is run to calibrate for any offsets on the opamps
_iq offset_Rsin,         // offset in resolver sine fbk channel @ no excitation
	offset_Rcos,         // offset in resolver cos  fbk channel @ no excitation
#if (CNGD == HOT)
	offset_shntV,        // offset in shunt current V fbk channel @ 0A
    offset_shntW,        // offset in shunt current W fbk channel @ 0A
	offset_shntU,        // offset in shunt current U fbk channel @ 0A
#endif
    offset_lemV,         // offset in LEM current V fbk channel @ 0A
	offset_lemW;         // offset in LEM current W fbk channel @ 0A

int16 OffsetCalCounter;

//SD Trip Level - scope for additonal work
Uint16  HLT = 0x7FFF,
        LLT = 0x0;

volatile float offset_SDFM1;  // offset in SD current V fbk channel @ 0A
volatile float offset_SDFM2;  // offset in SD current W fbk channel @ 0A

_iq K1 = _IQ(0.998),		  // Offset filter coefficient K1: 0.05/(T+0.05);
    K2 = _IQ(0.001999);	      // Offset filter coefficient K2: T/(T+0.05);

typedef struct {
	float As;      // phase A
	float Bs;      // phase B
	float Cs;      // phase C
} CURRENT_SENSOR;

CURRENT_SENSOR current_sensor[3];

float curLimit = 8.0;

// CMPSS parameters for Over Current Protection
Uint16  clkPrescale = 20,
		sampwin     = 30,
		thresh      = 18,
		LEM_curHi   = LEM(8.0),
		LEM_curLo   = LEM(8.0),
		SHUNT_curHi = SHUNT(8.0),
		SHUNT_curLo = SHUNT(8.0);

// ****************************************************************************
// Flag variables
// ****************************************************************************
volatile Uint16 EnableFlag = FALSE;

Uint32 IsrTicker = 0;

Uint16 BackTicker = 0,
       lsw = 0,
       TripFlagDMC = 0,				//PWM trip status
       clearTripFlagDMC = 0,
       RunMotor = 0;

int    EnableResolverISR,  // not used
       Run_Delay,          // not used
       LedCnt1=0;

Uint16 SpeedLoopPrescaler = 10,      // Speed loop pre scalar
       SpeedLoopCount = 1;           // Speed loop counter

// ****************************************************************************
// Variables for Field Oriented Control
// ****************************************************************************
float32 T = 0.001/ISR_FREQUENCY;    // Samping period (sec), see parameter.h
_iq VdTesting = _IQ(0.0),			// Vd reference (pu)
    VqTesting = _IQ(0.10),			// Vq reference (pu)
    IdRef     = _IQ(0.0),			// Id reference (pu)
    IqRef     = _IQ(0.0),			// Iq reference (pu)
    SpeedRef  = _IQ(0.0);           // For Closed Loop tests

// Instance a few transform objects
CLARKE clarke1 = CLARKE_DEFAULTS;
PARK   park1   = PARK_DEFAULTS;
IPARK  ipark1  = IPARK_DEFAULTS;

// Instance PI(D) regulators to regulate the d and q  axis currents, speed and position
PIDREG3         pid_pos = PIDREG3_DEFAULTS;          // (optional - for eval)
PI_CONTROLLER   pi_pos  = PI_CONTROLLER_DEFAULTS;
PID_CONTROLLER	pid_spd = {PID_TERM_DEFAULTS, PID_PARAM_DEFAULTS, PID_DATA_DEFAULTS};
PI_CONTROLLER   pi_id   = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER   pi_iq   = PI_CONTROLLER_DEFAULTS;

//PID_CONTROLLER	pid_pos = {PID_TERM_DEFAULTS, PID_PARAM_DEFAULTS, PID_DATA_DEFAULTS};
//PI_CONTROLLER pi_spd = PI_CONTROLLER_DEFAULTS;

// Instance a PWM driver instance
//PWMGEN pwm1 = PWMGEN_DEFAULTS;

// Instance a Space Vector PWM modulator. This modulator generates a, b and c
// phases based on the d and q stationery reference frame inputs
SVGEN svgen1 = SVGEN_DEFAULTS;

// Instance a ramp controller to smoothly ramp the frequency
RMPCNTL rc1 = RMPCNTL_DEFAULTS;
RMPCNTL rc2 = RMPCNTL_DEFAULTS; // for speed

//	Instance a ramp generator to simulate an Anglele
RAMPGEN rg1 = RAMPGEN_DEFAULTS;

//	Instance a phase voltage calculation
PHASEVOLTAGE volt1 = PHASEVOLTAGE_DEFAULTS;

// Instance a speed calculator based on Encoder position
SPEED_MEAS_QEP speed1 = SPEED_MEAS_QEP_DEFAULTS;

// Instance a QEP interface driver
QEP qep1 = QEP_DEFAULTS;

// Instance a RESOLVER interface driver
volatile RESOLVER resolver1 = RESOLVER_DEFAULTS;
volatile ABS_ENCODER endat1 = ABSENC_DEFAULTS;
volatile ABS_ENCODER biss1 = ABSENC_DEFAULTS;

// Variables for Position Sensor Suite
_iq posEncElecTheta[6],
    posEncMechTheta[6];

_iq  cntr=0,
	 alignCnt = 20000;
_iq  IdRef_start = _IQ(0.1),
	 IdRef_run   = _IQ(0.0);

// Variables for position reference generation and control
// =========================================================
_iq   posArray[8] = { _IQ(1.5), _IQ(-1.5), _IQ(2.5), _IQ(-2.5) },
	  cntr1=0 ,
	  posSlewRate = _IQ(0.001);

int16 ptrMax = 2,
      ptr1=0;

// ****************************************************************************
// Extern functions and variables referred from resolver.c
// ****************************************************************************
extern void baseParamsInit(void);
extern void derivParamsCal(void);

extern RESOLVER_OUTPUT rslvrOut;

// ****************************************************************************
// Variables for Datalog module
// ****************************************************************************
float DBUFF_4CH1[200],
      DBUFF_4CH2[200],
      DBUFF_4CH3[200],
      DBUFF_4CH4[200],
      DlogCh1,
      DlogCh2,
      DlogCh3,
      DlogCh4;

// Create an instance of DATALOG Module
DLOG_4CH_F dlog_4ch1;

//*******************************************************************************

// ******************************************************************************
// CURRENT SENSOR SUITE
// - Reads motor current from inverter bottom leg SHUNT
// - Reads motor current from LEM flux gate current sensors connected to motor
//     phases V and W
// - Reads motor current from series shunt connected to motor using isolated
//     Sigma Delta Filter Module
// ******************************************************************************
#if BUILDLEVEL != LEVEL1
inline void currentSensorSuite()
{
	volatile int16 temp;  // temp variable used to avoid warning msgs

#if (CNGD == HOT)
	current_sensor[SHUNT_CURRENT_SENSE-1].As = (float)IFB_SV_PPB* ADC_PU_PPB_SCALE_FACTOR;
	current_sensor[SHUNT_CURRENT_SENSE-1].Bs = (float)IFB_SW_PPB* ADC_PU_PPB_SCALE_FACTOR;
	current_sensor[SHUNT_CURRENT_SENSE-1].Cs = -current_sensor[SHUNT_CURRENT_SENSE-1].As
			                                   -current_sensor[SHUNT_CURRENT_SENSE-1].Bs;
#endif

	current_sensor[LEM_CURRENT_SENSE-1].As   = (float)IFB_LEMV_PPB* ADC_PU_PPB_SCALE_FACTOR * LEM_TO_SHUNT;
	current_sensor[LEM_CURRENT_SENSE-1].Bs   = (float)IFB_LEMW_PPB* ADC_PU_PPB_SCALE_FACTOR * LEM_TO_SHUNT;
	current_sensor[LEM_CURRENT_SENSE-1].Cs   = -current_sensor[LEM_CURRENT_SENSE-1].As
			                                   -current_sensor[LEM_CURRENT_SENSE-1].Bs;

	current_sensor[SD_CURRENT_SENSE-1].As    = ((temp=SDFM1_READ_FILTER1_DATA_16BIT)*SD_PU_SCALE_FACTOR -
			                                    offset_SDFM1) * SDFM_TO_SHUNT;
	current_sensor[SD_CURRENT_SENSE-1].Bs    = ((temp=SDFM1_READ_FILTER2_DATA_16BIT)*SD_PU_SCALE_FACTOR -
			                                    offset_SDFM2) * SDFM_TO_SHUNT;
	current_sensor[SD_CURRENT_SENSE-1].Cs    = -current_sensor[SD_CURRENT_SENSE-1].As
			                                   -current_sensor[SD_CURRENT_SENSE-1].Bs;

	return;
}
#endif

// ******************************************************************************
// POSITION ENCODER SUITE
// - Reads QEP
// - Decodes RESOLVER (core algo is available in resolver.lib)
// - Angles are normalised to the the range 0 to 0.99999 (1.0)
// ******************************************************************************
void posEncoderSuite(void)
{
// ----------------------------------
// lsw = 0 ---> Alignment Routine
// ----------------------------------
#if (POSITION_ENCODER == QEP_POS_ENCODER)
	if (lsw == 0)
	{
		// during alignment, assign the current shaft position as initial position
		EQep1Regs.QPOSCNT = 0;
		EQep1Regs.QCLR.bit.IEL = 1;  // Reset position cnt for QEP
	} // end if (lsw=0)

// ******************************************************************************
//    Detect calibration angle and call the QEP module
// ******************************************************************************
	// for once the QEP index pulse is found, go to lsw=2
	if(lsw==1)
	{
		if (EQep1Regs.QFLG.bit.IEL == 1)			// Check the index occurrence
		{
			qep1.CalibratedAngle=EQep1Regs.QPOSILAT;
//			EQep1Regs.QPOSINIT = EQep1Regs.QPOSILAT; //new
//			EQep1Regs.QEPCTL.bit.IEI = IEI_RISING;   // new
			lsw=2;
		}   // Keep the latched pos. at the first index
	}

	if (lsw!=0){
		QEP_MACRO(1,qep1);
	}

	// Reverse the sense of position if needed - comment / uncomment accordingly
	// Position Sense as is
	//posEncElecTheta[QEP_POS_ENCODER] = qep1.ElecTheta;
	//posEncMechTheta[QEP_POS_ENCODER] = qep1.MechTheta;

	// Position Sense Reversal
	posEncElecTheta[QEP_POS_ENCODER] = 1.0 - qep1.ElecTheta;
	posEncMechTheta[QEP_POS_ENCODER] = 1.0 - qep1.MechTheta;

// ******************************************************************************
//  Read resolver data, get position and speed feedback
// ******************************************************************************
#elif (POSITION_ENCODER == RESOLVER_POS_ENCODER)
	// during alignment, assign the current shaft position as initial position
	if (lsw == 0)
	{
		resolver1.InitTheta = resolver1.RawTheta;
	}

	resolver1.Speed    = rslvrOut.angleObs;
	resolver1.RawTheta = (rslvrOut.angleObs*0.5)+0.5;
	//RESOLVER_MACRO_F(resolver1)
	resolver1.MechTheta   = resolver1.RawTheta - resolver1.InitTheta;      /* MechTheta in step counts */
	if(resolver1.MechTheta < 0)
		resolver1.MechTheta = resolver1.MechTheta + 1.0;
	else if (resolver1.MechTheta > 1.0)
		resolver1.MechTheta = resolver1.MechTheta - 1.0;

    // Compute the electrical angle in Q24
	resolver1.ElecTheta  = _IQfrac(resolver1.PolePairs * resolver1.MechTheta);

	// Reverse the sense of position if needed - comment / uncomment accordingly
	// Position Sense as is
	posEncElecTheta[RESOLVER_POS_ENCODER] = resolver1.ElecTheta;
	posEncMechTheta[RESOLVER_POS_ENCODER] = resolver1.MechTheta;

	// Position Sense Reversal
	//posEncElecTheta[RESOLVER_POS_ENCODER] = 1.0 - resolver1.ElecTheta;
	//posEncMechTheta[RESOLVER_POS_ENCODER] = 1.0 - resolver1.MechTheta;

#endif

#if POSITION_ENCODER==ENDAT_POS_ENCODER
//Read position data in EnDat21 mode. Function defined in endat.c
	if(endat22Data.dataReady == 1)
	{
				endat21_readPosition();
	}

	endat1.RawTheta = (1.0-((float)endat22Data.position_lo)/33554432.0);; // Dividing by 2^25 : EnDat Encoder 25 bits

	// during alignment, assign the current shaft position as initial position
	if (lsw == 0)
	{
		endat1.InitTheta = endat1.RawTheta;
	}

	endat1.MechTheta   = endat1.RawTheta - endat1.InitTheta;      /* MechTheta in step counts */
	if(endat1.MechTheta < 0)
		endat1.MechTheta = endat1.MechTheta + 1.0;
	else if (endat1.MechTheta > 1.0)
		endat1.MechTheta = endat1.MechTheta - 1.0;

    // Compute the electrical angle in Q24
	endat1.ElecTheta  = _IQfrac(endat1.PolePairs * endat1.MechTheta);

	// Reverse the sense of position if needed - comment / uncomment accordingly
	// Position Sense as is
//	posEncElecTheta[ENDAT_POS_ENCODER] = endat1.ElecTheta;
//	posEncMechTheta[ENDAT_POS_ENCODER] = endat1.MechTheta;

	// Position Sense Reversal
	posEncElecTheta[ENDAT_POS_ENCODER] = 1.0 - endat1.ElecTheta;
	posEncMechTheta[ENDAT_POS_ENCODER] = 1.0 - endat1.MechTheta;

#endif
#if POSITION_ENCODER==BISS_POS_ENCODER
//Read position data in EnDat21 mode. Function defined in endat.c
	if(bissc_data_struct.dataReady == 1)
	{
				bissc_readPosition();
	}

	biss1.RawTheta = (1.0-((float)bissc_data_struct.position)/262144.0);; // Dividing by 2^25 : EnDat Encoder 25 bits

	// during alignment, assign the current shaft position as initial position
	if (lsw == 0)
	{
		biss1.InitTheta = biss1.RawTheta;
	}

	biss1.MechTheta   = biss1.RawTheta - biss1.InitTheta;      /* MechTheta in step counts */
	if(biss1.MechTheta < 0)
		biss1.MechTheta = biss1.MechTheta + 1.0;
	else if (biss1.MechTheta > 1.0)
		biss1.MechTheta = biss1.MechTheta - 1.0;

    // Compute the electrical angle in Q24
	biss1.ElecTheta  = _IQfrac(biss1.PolePairs * biss1.MechTheta);

	// Reverse the sense of position if needed - comment / uncomment accordingly
	// Position Sense as is
	posEncElecTheta[BISS_POS_ENCODER] = biss1.ElecTheta;
	posEncMechTheta[BISS_POS_ENCODER] = biss1.MechTheta;

	// Position Sense Reversal
//	posEncElecTheta[BISS_POS_ENCODER] = 1.0 - biss1.ElecTheta;
//	posEncMechTheta[BISS_POS_ENCODER] = 1.0 - biss1.MechTheta;

#endif

	return;
}

//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************

void main(void)
{

	volatile int16 temp;

#ifdef _FLASH
// Copy time critical code and Flash setup code to RAM
// The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
// symbols are created by the linker. Refer to the linker files.
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);
#endif

	//  Initialize System Control:
	// PLL, WatchDog, enable Peripheral Clocks
	// This example function is found in the F28M3Xx_SysCtrl.c file.
	InitSysCtrl();

	// Only used if running from FLASH
	// Note that the variable FLASH is defined by the compiler

#ifdef _FLASH
// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
	InitFlash();	// Call the flash wrapper init function
#endif //(FLASH)

	// Waiting for enable flag set
	while (EnableFlag == FALSE)
	{
	  BackTicker++;
	}

	// Clear all interrupts and initialize PIE vector table:

	// Disable CPU interrupts
	DINT;

	// Initialize the PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.
	// This function is found in the F28M3Xx_PieCtrl.c file.
	InitPieCtrl();

	// Disable CPU interrupts and clear all CPU interrupt flags:
	IER = 0x0000;
	IFR = 0x0000;
	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in F28M3Xx_DefaultIsr.c.
	// This function is found in F28M3Xx_PieVect.c.
	InitPieVectTable();

// Timing sync for background loops
// Timer period definitions found in device specific PeripheralHeaderIncludes.h
	CpuTimer0Regs.PRD.all =  10000;		// A tasks
	CpuTimer1Regs.PRD.all =  20000;		// B tasks
	CpuTimer2Regs.PRD.all =  30000;	// C tasks

// Tasks State-machine init
	Alpha_State_Ptr = &A0;
	A_Task_Ptr = &A1;
	B_Task_Ptr = &B1;
	C_Task_Ptr = &C1;


// ****************************************************************************
// ****************************************************************************
//TODO PWM Configuration
// ****************************************************************************
// ****************************************************************************

    // Initialize PWM module
	EALLOW;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;

	// *****************************************
	// Inverter PWM configuration
	// ****************************************
	/* By default on soprano the PWM clock is divided by 2
	 * ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV=1
	 * Deadband needs to be 2.0us => 10ns*200=2us
	 */
	PWM_1ch_UpDwnCnt_CNF(1,INV_PWM_TICKS,200);
	PWM_1ch_UpDwnCnt_CNF(2,INV_PWM_TICKS,200);
	PWM_1ch_UpDwnCnt_CNF(3,INV_PWM_TICKS,200);

	// **********************************************
	// Sigma Delta clock set up - pwm5
	// *********************************************
	/* Configure PWM5A for SD Clock i.e. 20Mhz
	 * 20 Mhz => 50ns => 50ns/10
	 */
	PWM_1ch_UpCnt_CNF(5,SDFM_TICKS);
	EPwm5Regs.CMPA.bit.CMPA=EPwm5Regs.TBPRD>>1;

	// **********************************************
	// Carrier clock for resolver - pwm6
	// *********************************************
	// Resolver PWM6 time base for injection
	PWM_1ch_UpDwnCnt_CNF(6,RESOLVER_PWM_TICKS,160);

	// ********************************************************************
	//PWM 11 for syncing up the SD filter windows with motor control PWMs
	// ********************************************************************
	PWM_1ch_UpCnt_CNF(11,INV_PWM_TICKS);

	// configure 2 and 3 as slaves
	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm2Regs.TBCTL.bit.PHSEN    = TB_ENABLE;
	EPwm2Regs.TBPHS.bit.TBPHS    = 2;
	EPwm2Regs.TBCTL.bit.PHSDIR   = TB_UP;

	EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm3Regs.TBCTL.bit.PHSEN    = TB_ENABLE;
	EPwm3Regs.TBPHS.bit.TBPHS    = 2;
	EPwm3Regs.TBCTL.bit.PHSDIR   = TB_UP;

	SyncSocRegs.SYNCSELECT.bit.EPWM4SYNCIN = 0; 	 //EPwm1SyncOut

	EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;

	EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm6Regs.TBCTL.bit.PHSEN    = TB_ENABLE;
	EPwm6Regs.TBPHS.bit.TBPHS    = 2;
	EPwm6Regs.TBCTL.bit.PHSDIR   = TB_UP;

	SyncSocRegs.SYNCSELECT.bit.EPWM10SYNCIN = 0;  //EPwm1Sync Out
	EPwm10Regs.TBCTL.bit.SYNCOSEL        = TB_SYNC_IN;

	EPwm11Regs.TBCTL.bit.PHSEN  = TB_ENABLE;
	EPwm11Regs.TBPHS.bit.TBPHS  = 2;
	EPwm11Regs.TBCTL.bit.PHSDIR = TB_UP;

	EPwm11Regs.CMPC = EPwm11Regs.TBPRD - SDFM_TICKS*(OSR_RATE+1)*3/2;
	EPwm11Regs.CMPA.bit.CMPA = (SDFM_TICKS*(OSR_RATE+1)*3/2) + 500; // 500 is arbitrary
	EPwm11Regs.CMPD = 0;

	// ***********************************
	// Set up GPIOs for PWM functions
	// **************************************
	InitEPwm1Gpio();
	InitEPwm2Gpio();
	InitEPwm3Gpio();
	InitEPwm4Gpio();
	InitEPwm5Gpio();

	EDIS;

// ****************************************************
// Initialize DATALOG module
// ****************************************************
	DLOG_4CH_F_init(&dlog_4ch1);
	dlog_4ch1.input_ptr1 = &DlogCh1;	//data value
	dlog_4ch1.input_ptr2 = &DlogCh2;
	dlog_4ch1.input_ptr3 = &DlogCh3;
	dlog_4ch1.input_ptr4 = &DlogCh4;
	dlog_4ch1.output_ptr1 = &DBUFF_4CH1[0];
	dlog_4ch1.output_ptr2 = &DBUFF_4CH2[0];
	dlog_4ch1.output_ptr3 = &DBUFF_4CH3[0];
	dlog_4ch1.output_ptr4 = &DBUFF_4CH4[0];
	dlog_4ch1.size = 200;
	dlog_4ch1.pre_scalar = 5;
	dlog_4ch1.trig_value = 0.01;
	dlog_4ch1.status = 2;



// ****************************************************************************
// ****************************************************************************
//TODO ADC Configuration
// ****************************************************************************
// ****************************************************************************
    //Configure the ADC and power it up
	ConfigureADC();

	//Select the channels to convert and end of conversion flag
	//Current Measurements has multiple options

	EALLOW;

	// Analog signals that are sampled
	// Ifb-SV  ADC A4/C+
	// Ifb-SW  ADC B4/C+
	// LEM V   ADC A2/C+
	// LEM W   ADC B2/C+
	// R_SIN   ADC D1
	// R_COS   ADC C1
	// Vfb-U   ADC C3
	// Vfb-V   ADC A3
	// Vfb-W   ADC B3
	// Vfb-Bus ADC B0

	// Analog Signals brought in but not sampled
	// Ifb-SU  ADC C2/C+ (& A5 not used)
	// SC-A2   ADC C0/C+
	// SC-B2   ADC D0/C+
	// SC-R    ADC D2/C+
	// Bus Volt ADC B0

	// On piccolo 133ns for ACQPS
	// hencce ACQPS on soprano is 133/5~30

	// Configure the SOC0 on ADC a-d
#if (CGND == HOT)
	// Shunt Motor Currents (SV) @ A4
	// ********************************
	AdcaRegs.ADCSOC0CTL.bit.CHSEL     = 4;    // SOC0 will convert pin A4
	AdcaRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcaRegs.ADCSOC0CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
	// Configure the post processing block (PPB) to eliminate subtraction related calculation
	AdcaRegs.ADCPPB1CONFIG.bit.CONFIG = 0;    // PPB is associated with SOC0
	AdcaRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

	// Shunt Motor Currents (SW) @ B4
	// ********************************
	AdcbRegs.ADCSOC0CTL.bit.CHSEL     = 4;    // SOC0 will convert pin B4
	AdcbRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcbRegs.ADCSOC0CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdcbRegs.ADCPPB1CONFIG.bit.CONFIG = 0;    // PPB is associated with SOC0
	AdcbRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run
#endif
	// Resolver Fbk - Cosine @ C1
	// ********************************
	AdccRegs.ADCSOC0CTL.bit.CHSEL     = 15;   // SOC0 will convert pin C15
	AdccRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdccRegs.ADCSOC0CTL.bit.TRIGSEL   = 15;   // trigger on ePWM6 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdccRegs.ADCPPB1CONFIG.bit.CONFIG = 0;    // PPB is associated with SOC0
	AdccRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

	// Resolver Fbk - sine @ D1
	// ********************************
	AdcdRegs.ADCSOC0CTL.bit.CHSEL     = 1;    // SOC0 will convert pin D1
	AdcdRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcdRegs.ADCSOC0CTL.bit.TRIGSEL   = 15;   // trigger on ePWM6 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdcdRegs.ADCPPB1CONFIG.bit.CONFIG = 0;    // PPB is associated with SOC0
	AdcdRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

	// LEM motor current LEM-V @ at A2
	// ********************************
	AdcaRegs.ADCSOC1CTL.bit.CHSEL     = 2;    // SOC1 will convert pin A2
	AdcaRegs.ADCSOC1CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcaRegs.ADCSOC1CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdcaRegs.ADCPPB2CONFIG.bit.CONFIG = 1;    // PPB is associated with SOC1
	AdcaRegs.ADCPPB2OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

	// LEM motor current LEM-W @ at B2
	// ********************************
	AdcbRegs.ADCSOC1CTL.bit.CHSEL     = 2;    // SOC0 will convert pin B2
	AdcbRegs.ADCSOC1CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcbRegs.ADCSOC1CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdcbRegs.ADCPPB2CONFIG.bit.CONFIG = 1;    // PPB is associated with SOC1
	AdcbRegs.ADCPPB2OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

	// Phase Voltage Vfb-V @ A3
	// ***************************
	AdcaRegs.ADCSOC2CTL.bit.CHSEL     = 3;    // SOC2 will convert pin A3,
	AdcaRegs.ADCSOC2CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcaRegs.ADCSOC2CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdcaRegs.ADCPPB3CONFIG.bit.CONFIG = 1;    // PPB is associated with SOC2
	AdcaRegs.ADCPPB3OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

	// Phase Voltage Vfb-W @ B3
	// ***************************
	AdcbRegs.ADCSOC2CTL.bit.CHSEL     = 3;    // SOC2 will convert pin B3
	AdcbRegs.ADCSOC2CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcbRegs.ADCSOC2CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdcbRegs.ADCPPB3CONFIG.bit.CONFIG = 1;    // PPB is associated with SOC2
	AdcbRegs.ADCPPB3OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

	// Phase Voltage Vfb-U @ C3
	// ****************************
	AdccRegs.ADCSOC2CTL.bit.CHSEL     = 3;    // SOC2 will convert pin C3
	AdccRegs.ADCSOC2CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdccRegs.ADCSOC2CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdccRegs.ADCPPB3CONFIG.bit.CONFIG = 1;    // PPB is associated with SOC2
	AdccRegs.ADCPPB3OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

	// Bus Voltage Feedback at B0 (not used)
	// **************************************
	AdcbRegs.ADCSOC3CTL.bit.CHSEL    = 0;     // SOC3 will convert pin B0
	AdcbRegs.ADCSOC3CTL.bit.ACQPS    = 30;    // sample window in SYSCLK cycles
	AdcbRegs.ADCSOC3CTL.bit.TRIGSEL  = 5;     // trigger on ePWM1 SOCA/C

	// ******************************************************
	// static analog trim for all ADCs (A, B, C and D)
	// *******************************************************
	adc_cal=(int*)0x0000743F;
	*adc_cal=0x7000;
	adc_cal=(int*)0x000074BF;
	*adc_cal=0x7000;
	adc_cal=(int*)0x0000753F;
	*adc_cal=0x7000;
	adc_cal=(int*)0x000075BF;
	*adc_cal=0x7000;

	// Setting up link from EPWM to ADC
	EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD; // Select SOC from counter at ctr = 0
	EPwm1Regs.ETPS.bit.SOCAPRD  = ET_1ST;     // Generate pulse on 1st even
	EPwm1Regs.ETSEL.bit.SOCAEN  = 1;          // Enable SOC on A group

	EPwm6Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD; // Select SOC from counter at ctr = 0
	EPwm6Regs.ETPS.bit.SOCAPRD  = ET_1ST;     // Generate pulse on 1st even
	EPwm6Regs.ETSEL.bit.SOCAEN  = 1;          // Enable SOC on A group

	EPwm11Regs.ETSEL.bit.INTSEL = ET_CTRU_CMPA;   // INT on PRD event
	EPwm11Regs.ETSEL.bit.INTEN  = 1;              // Enable INT
	EPwm11Regs.ETPS.bit.INTPRD  = ET_1ST;         // Generate INT on every event

	// SETUP DACS
	DacaRegs.DACCTL.bit.DACREFSEL = REFERENCE_VREF;
	DacaRegs.DACCTL.bit.LOADMODE  = 1;      // enable value change only on sync signal
	//Enable DAC output
	DacaRegs.DACOUTEN.bit.DACOUTEN = 1;
	DacaRegs.DACCTL.bit.SYNCSEL    = 5;     // sync sel 5 meanse sync from pwm 6
	DacaRegs.DACVALS.bit.DACVALS   = 1024;

	DacbRegs.DACCTL.bit.DACREFSEL  = REFERENCE_VREF;
	//Enable DAC output
	DacbRegs.DACOUTEN.bit.DACOUTEN = 1;
	DacbRegs.DACVALS.bit.DACVALS   = 1024;

	DaccRegs.DACCTL.bit.DACREFSEL  = REFERENCE_VREF;
	//Enable DAC output
	DaccRegs.DACOUTEN.bit.DACOUTEN = 1;
	DaccRegs.DACVALS.bit.DACVALS   = 1024;
	EDIS;

// ****************************************************************************
// ****************************************************************************
//TODO 	Sigma Delta Initialization
// ****************************************************************************
// ****************************************************************************
    // Setup GPIO for SD current measurement
	GPIO_SetupPinOptions(48, GPIO_INPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(48,0,7);

	GPIO_SetupPinOptions(49, GPIO_INPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(49,0,7);

	GPIO_SetupPinOptions(50, GPIO_INPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(50,0,7);

	GPIO_SetupPinOptions(51, GPIO_INPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(51,0,7);

	// Setup GPIO for SD voltage measurement
	GPIO_SetupPinOptions(52, GPIO_INPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(52,0,7);

	GPIO_SetupPinOptions(53, GPIO_INPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(53,0,7);

	/*******************************************************/
	/* Input Control Module */
	/*******************************************************/
	//Configure Input Control Mode: Modulator Clock rate = Modulator data rate
	Sdfm_configureInputCtrl(1,FILTER1,MODE_0);
	Sdfm_configureInputCtrl(1,FILTER2,MODE_0);
	Sdfm_configureInputCtrl(1,FILTER3,MODE_0);

	/*******************************************************/
	/* Comparator Module */
	/*******************************************************/
	//Comparator HLT and LLT
	//Configure Comparator module's comparator filter type and comparator's OSR value,
	// high level threshold, low level threshold
	Sdfm_configureComparator(1, FILTER1, SINC3, OSR_32, HLT, LLT);
	Sdfm_configureComparator(1, FILTER2, SINC3, OSR_32, HLT, LLT);
	Sdfm_configureComparator(1, FILTER3, SINC3, OSR_32, HLT, LLT);

	/*******************************************************/
	/* Sinc filter Module */
	/*******************************************************/
	//Configure Data filter modules filter type, OSR value and enable / disable data filter
	// 16 bit data representation is chosen for OSR 128 using Sinc3, from the table in the TRM
	// the max value represented for OSR 128 using sinc 3 is +/-2097152 i.e. 2^21
	// to represent this in 16 bit format where the first bit is sign shift by 6 bits
	Sdfm_configureData_filter(1, FILTER1, FILTER_ENABLE, SINC3, OSR_RATE, DATA_16_BIT, SHIFT_6_BITS);
	Sdfm_configureData_filter(1, FILTER2, FILTER_ENABLE, SINC3, OSR_RATE, DATA_16_BIT, SHIFT_6_BITS);
	Sdfm_configureData_filter(1, FILTER3, FILTER_ENABLE, SINC3, OSR_RATE, DATA_16_BIT, SHIFT_6_BITS);

	//PWM11.CMPC, PWM11.CMPD, PWM12.CMPC and PWM12.CMPD signals cannot synchronize the filters. This option is not being used in this example.
    Sdfm_configureExternalreset(1,FILTER_1_EXT_RESET_ENABLE, FILTER_2_EXT_RESET_ENABLE, FILTER_3_EXT_RESET_ENABLE, FILTER_4_EXT_RESET_ENABLE);

    // Enable master filter bit of the SDFM module 1
    Sdfm_enableMFE(1);

// ****************************************************************************
// ****************************************************************************
// Initialize QEP module
// ****************************************************************************
// ****************************************************************************
    // Setup GPIO for QEP operation
	GPIO_SetupPinOptions(20, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(20,0,1);

	GPIO_SetupPinOptions(21, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(21,0,1);

	GPIO_SetupPinOptions(22, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(22,0,1);

	GPIO_SetupPinOptions(23, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(23,0,1);

// ****************************************************************************
// ****************************************************************************
// To profile use GPIO 42 and GPIO43 (gen purpose)
// ****************************************************************************
// ****************************************************************************
	//configure LED
	GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(34,0,0);

	GPIO_SetupPinOptions(42, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(42,0,0);

	GPIO_SetupPinOptions(43, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(43,0,0);

// ****************************************************************************
// ****************************************************************************
// Paramaeter Initialisation
// ****************************************************************************
// ****************************************************************************

	// Init QEP parameters
    qep1.LineEncoder = 2500; // these are the number of slots in the QEP encoder
    qep1.MechScaler  = _IQ30(0.25/qep1.LineEncoder);
    qep1.PolePairs   = POLES/2;
    qep1.CalibratedAngle = 0;
    QEP_INIT_MACRO(1,qep1)
    EQep1Regs.QEPCTL.bit.IEI = 0;        // disable POSCNT=POSINIT @ Index

    // Init RESOLVER parameters
	resolver1.StepsPerTurn = RESOLVER_STEPS_PER_TURN;
	resolver1.MechScaler   =  1.0;       //_IQ30(1.0/resolver1.StepsPerTurn);
	resolver1.PolePairs    = POLES/2;

	// Init BiSS-C parameters
	biss1.PolePairs    = POLES/2;
	// Init EnDat22 parameters
	endat1.PolePairs    = POLES/2;

	baseParamsInit();                    // initialise all parameters
	derivParamsCal();                    // set up derivative loop parameters
	init_resolver_Float();

    // Initialize the Speed module for speed calculation from QEP/RESOLVER
    speed1.K1 = _IQ21(1/(BASE_FREQ*T));
    speed1.K2 = _IQ(1/(1+T*2*PI*5));      // Low-pass cut-off frequency
    speed1.K3 = _IQ(1)-speed1.K2;
    speed1.BaseRpm = 120*(BASE_FREQ/POLES);

    // Initialize the RAMPGEN module
    rg1.StepAngleMax = _IQ(BASE_FREQ*T);

    // Initialize the PI module for position
	pi_pos.Kp = _IQ(1.0);            //_IQ(10.0);
	pi_pos.Ki = _IQ(0.001);          //_IQ(T*SpeedLoopPrescaler/0.3);
	pi_pos.Umax = _IQ(1.0);
	pi_pos.Umin = _IQ(-1.0);

    // Initialize the PID module for position (alternative option for eval)
    pid_pos.Ref = 0;
    pid_pos.Fdb = 0;
    pid_pos.OutMin = _IQ(-0.5);
    pid_pos.OutMax = _IQ(0.5);
    pid_pos.Out = 0;

    pid_pos.Kp = 1.0;
    pid_pos.Ki = 0;
    pid_pos.Kd = 0;
    pid_pos.Kc = 0.9;

    pid_pos.Up1 = 0;
	pid_pos.Up  = 0;
	pid_pos.Ui  = 0;
	pid_pos.Ud  = 0;
	pid_pos.SatErr    = 0;
	pid_pos.OutPreSat = 0;

	//Initialization routine for endat operation - defined in endat.c
	//Configures the peripherals and enables clocks for required modules
	//Configures GPIO and XBar as needed for EnDat operation
	//Sets up the SPI peripheral in endat data structure and enables interrupt
#if POSITION_ENCODER==ENDAT_POS_ENCODER
	EnDat_Init();
	//Peforms cable propagation delay calculation.
	//This is required for long cable lengths and higher EnDat Clock frequencies
	//Function defined in endat.c

	EnDat_initDelayComp();

	//Switch to high frequency - 8.3MHz	(=200/4*ENDAT_RUNTIME_FREQ_DIVIDER)
	PM_endat22_setFreq(ENDAT_RUNTIME_FREQ_DIVIDER);
	DELAY_US(800L); 	//Delay 800us
	endat22Data.dataReady = 1;

#endif

#if POSITION_ENCODER==BISS_POS_ENCODER
	bissc_init();
	DELAY_US(800L); 	//Delay 800us
	bissc_data_struct.dataReady = 1;

#endif

	// Initialize the PID module for speed
#if (BUILDLEVEL==LEVEL5) || (BUILDLEVEL == LEVEL6)
//	pid_spd.param.Kp=_IQ(2.5);
//	pid_spd.param.Ki=_IQ(0.0001);
//	pid_spd.param.Kd=_IQ(0.0);
//	pid_spd.param.Kr=_IQ(1.0);
//	pid_spd.param.Umax=_IQ(0.9);
//	pid_spd.param.Umin=_IQ(-0.9);
	pid_spd.param.Kp=_IQ(1.0);
	pid_spd.param.Ki=_IQ(0.001);
	pid_spd.param.Kd=_IQ(0.0);
	pid_spd.param.Kr=_IQ(1.0);
	pid_spd.param.Umax=_IQ(0.95);
	pid_spd.param.Umin=_IQ(-0.95);
#else
	pid_spd.param.Kp   = _IQ(1.0);
	pid_spd.param.Ki   = _IQ(0.001);
	pid_spd.param.Kd   = _IQ(0.0);
	pid_spd.param.Kr   = _IQ(1.0);
	pid_spd.param.Umax = _IQ(0.95);
	pid_spd.param.Umin = _IQ(-0.95);
#endif

	// Init PI module for ID loop
	pi_id.Kp   = _IQ(1.0);//_IQ(3.0);
	pi_id.Ki   = _IQ(T/0.04);//0.0075);
	pi_id.Umax = _IQ(0.5);
	pi_id.Umin = _IQ(-0.5);

	// Init PI module for IQ loop
	pi_iq.Kp   = _IQ(1.0);//_IQ(4.0);
	pi_iq.Ki   = _IQ(T/0.04);//_IQ(0.015);
	pi_iq.Umax = _IQ(0.8);
	pi_iq.Umin = _IQ(-0.8);

	// Set mock REFERENCES for Speed and Iq loops
	SpeedRef = 0.05;

#if BUILDLEVEL == LEVEL3
	IqRef    = 0.0;
#else
	IqRef = _IQ(0.05);
#endif

	// Init FLAGS
	RunMotor = 0;
	LedCnt1  = 0;
	EnableResolverISR = 1;

#if (BUILDLEVEL==LEVEL2)
	Run_Delay = 10;
#else
	Run_Delay = 100;
#endif

//  Note that the vectorial sum of d-q PI outputs should be less than 1.0 which
//  refers to maximum duty cycle for SVGEN. Another duty cycle limiting factor
//	is current sense through shunt resistors which depends on hardware/software
//  implementation. Depending on the application requirements 3,2 or a single
//	shunt resistor can be used for current waveform reconstruction. The higher
//  number of shunt resistors allow the higher duty cycle operation and better
//	dc bus utilization. The users should adjust the PI saturation levels
//  carefully during open loop tests (i.e pi_id.Umax, pi_iq.Umax and Umins) as
//	in project manuals. Violation of this procedure yields distorted  current
// waveforms and unstable closed loop operations which may damage the inverter.

// ****************************************************************************
// ****************************************************************************
// Call HVDMC Protection function
// ****************************************************************************
// ****************************************************************************
	HVDMC_Protection();

// TODO
// ****************************************************************************
// ****************************************************************************
// Feedbacks OFFSET Calibration Routine
// ****************************************************************************
// ****************************************************************************
	EALLOW;
	  CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;

	offset_Rsin  = 0;
	offset_Rcos  = 0;
#if (CNGD == HOT)
	offset_shntV = 0;
	offset_shntW = 0;
	offset_shntU = 0;
#endif
	offset_lemW  = 0;
	offset_lemV  = 0;
	offset_SDFM1 = 0;
	offset_SDFM2 = 0;

	for (OffsetCalCounter=0; OffsetCalCounter<20000; )
	{
		if(EPwm11Regs.ETFLG.bit.INT==1)
		{
			if(OffsetCalCounter>1000)
			{
				offset_SDFM1 = K1*offset_SDFM1 + K2*(temp=SDFM1_READ_FILTER1_DATA_16BIT)*SD_PU_SCALE_FACTOR;
				offset_SDFM2 = K1*offset_SDFM2 + K2*(temp=SDFM1_READ_FILTER2_DATA_16BIT)*SD_PU_SCALE_FACTOR;
#if (CNGD == HOT)
				offset_shntV = K1*offset_shntV + K2*(IFB_SV)*ADC_PU_SCALE_FACTOR; 			//Phase A offset
				offset_shntW = K1*offset_shntW + K2*(IFB_SW)*ADC_PU_SCALE_FACTOR; 			//Phase B offset
#endif
				offset_lemV  = K1*offset_lemV + K2*(IFB_LEMV)*ADC_PU_SCALE_FACTOR;
				offset_lemW  = K1*offset_lemW + K2*(IFB_LEMW)*ADC_PU_SCALE_FACTOR;

				offset_Rsin  = K1*offset_Rsin + K2*R_SIN*ADC_PU_SCALE_FACTOR;
				offset_Rcos  = K1*offset_Rcos + K2*R_COS*ADC_PU_SCALE_FACTOR;
			}
			EPwm11Regs.ETCLR.bit.INT=1;
			OffsetCalCounter++;
		}
	}

	// ********************************************
	// Init OFFSET regs with identified values
	// ********************************************
	EALLOW;
#if (CNGD == HOT)
	AdcaRegs.ADCPPB1OFFREF = (offset_shntV*4096.0);     // setting shunt Iu offset
	AdcbRegs.ADCPPB1OFFREF = (offset_shntW*4096.0);     // setting shunt Iv offset
#endif
	AdccRegs.ADCPPB1OFFREF = (offset_Rcos*4096.0);      // setting resolver cos in offset
	AdcdRegs.ADCPPB1OFFREF = (offset_Rsin*4096.0);      // setting resolver sin in offset

	AdcaRegs.ADCPPB2OFFREF = (offset_lemV*4096.0);      // setting LEM Iv offset
	AdcbRegs.ADCPPB2OFFREF = (offset_lemW*4096.0);      // setting LEM Iw offset

	EDIS;

// ****************************************************************************
// ****************************************************************************
//TODO ISR Mapping
// ****************************************************************************
// ****************************************************************************
	EALLOW;
	// ADC C EOC of SOC0 is used to trigger Resolver Interrupt
	AdccRegs.ADCINTSEL1N2.bit.INT1SEL  = 0;
	AdccRegs.ADCINTSEL1N2.bit.INT1CONT = 1;
	AdccRegs.ADCINTSEL1N2.bit.INT1E    = 1;

	//PWM11 INT is used to trigger Motor Control ISR
	EPwm11Regs.ETSEL.bit.INTSEL = ET_CTRU_CMPA;   // INT on PRD event
	EPwm11Regs.ETSEL.bit.INTEN  = 1;              // Enable INT
	EPwm11Regs.ETPS.bit.INTPRD  = ET_1ST;         // Generate INT on every event

	PieVectTable.ADCC1_INT = &ResolverISR;
	PieVectTable.EPWM11_INT = &MotorControlISR;

	PieCtrlRegs.PIEIER3.bit.INTx11 = 1;  // Enable PWM11INT in PIE group 3

#if POSITION_ENCODER == RESOLVER_POS_ENCODER
	PieCtrlRegs.PIEIER1.bit.INTx3  = 1;  // Enable ADCC1INT in PIE group 1
#endif

	EPwm11Regs.ETCLR.bit.INT=1;

	IER |= M_INT3; // Enable group 3 interrupts
	IER |= M_INT1; // Enable group 1 interrupts
	EINT;          // Enable Global interrupt INTM
	ERTM;          // Enable Global realtime interrupt DBGM
	EDIS;

// ***************************************************************************
//  Initialisations COMPLETE
//  - IDLE loop. Just loop forever
// ***************************************************************************
	for(;;)  //infinite loop
	{
		// State machine entry & exit point
		//===========================================================
		(*Alpha_State_Ptr)();	// jump to an Alpha state (A0,B0,...)
		//===========================================================
	}
} //END MAIN CODE

/******************************************************************************
 * ****************************************************************************
 * ****************************************************************************
 * ****************************************************************************
 */

//=================================================================================
//	STATE-MACHINE SEQUENCING AND SYNCRONIZATION FOR SLOW BACKGROUND TASKS
//=================================================================================

//--------------------------------- FRAMEWORK -------------------------------------
void A0(void)
{
	// loop rate synchronizer for A-tasks
	if(CpuTimer0Regs.TCR.bit.TIF == 1)
	{
		CpuTimer0Regs.TCR.bit.TIF = 1;	// clear flag

		//-----------------------------------------------------------
		(*A_Task_Ptr)();		// jump to an A Task (A1,A2,A3,...)
		//-----------------------------------------------------------

		VTimer0[0]++;			// virtual timer 0, instance 0 (spare)
		SerialCommsTimer++;
	}

	Alpha_State_Ptr = &B0;		// Comment out to allow only A tasks
}

void B0(void)
{
	// loop rate synchronizer for B-tasks
	if(CpuTimer1Regs.TCR.bit.TIF == 1)
	{
		CpuTimer1Regs.TCR.bit.TIF = 1;				// clear flag

		//-----------------------------------------------------------
		(*B_Task_Ptr)();		// jump to a B Task (B1,B2,B3,...)
		//-----------------------------------------------------------
		VTimer1[0]++;			// virtual timer 1, instance 0 (spare)
	}

	Alpha_State_Ptr = &C0;		// Allow C state tasks
}

void C0(void)
{
	// loop rate synchronizer for C-tasks
	if(CpuTimer2Regs.TCR.bit.TIF == 1)
	{
		CpuTimer2Regs.TCR.bit.TIF = 1;				// clear flag

		//-----------------------------------------------------------
		(*C_Task_Ptr)();		// jump to a C Task (C1,C2,C3,...)
		//-----------------------------------------------------------
		VTimer2[0]++;			//virtual timer 2, instance 0 (spare)
	}

	Alpha_State_Ptr = &A0;	// Back to State A0
}


//=================================================================================
//	A - TASKS (executed in every 50 usec)
//=================================================================================

// Setup OCP limits and digital filter parameters of CMPSS
void  CMPSS_DIG_FILTER(volatile struct CMPSS_REGS *v, Uint16 curHi, Uint16 curLo)
{
	// comparator references
	v->DACHVALS.bit.DACVAL = curHi;   // positive max current limit
	v->DACLVALS.bit.DACVAL = curLo;   // negative max current limit

	// digital filter settings - HIGH side
	v->CTRIPHFILCLKCTL.bit.CLKPRESCALE = clkPrescale;    // set time between samples, max : 1023
	v->CTRIPHFILCTL.bit.SAMPWIN        = sampwin;        // # of samples in window, max : 31
	v->CTRIPHFILCTL.bit.THRESH         = thresh;         // recommended : thresh > sampwin/2

	// digital filter settings - LOW side
	v->CTRIPLFILCLKCTL.bit.CLKPRESCALE = clkPrescale;    // Max count of 1023 */
	v->CTRIPLFILCTL.bit.SAMPWIN        = sampwin;        // # of samples in window, max : 31
	v->CTRIPLFILCTL.bit.THRESH         = thresh;         // recommended : thresh > sampwin/2

	return;
}

//--------------------------------------------------------
void A1(void) // SPARE (not used)
//--------------------------------------------------------
{
	// *******************************************************
	// Current limit setting / tuning in Debug environment
	// *******************************************************
	EALLOW;
	  LEM_curHi = 2048 + LEM(curLimit);
	  LEM_curLo = 2048 - LEM(curLimit);
	  SHUNT_curHi = 2048 + SHUNT(curLimit);
	  SHUNT_curLo = 2048 - SHUNT(curLimit);

	  CMPSS_DIG_FILTER(&Cmpss1Regs, LEM_curHi, LEM_curLo);      // LEM - V
	  CMPSS_DIG_FILTER(&Cmpss3Regs, LEM_curHi, LEM_curLo);      // LEM - W
#if (CGND == HOT)
	  CMPSS_DIG_FILTER(&Cmpss2Regs, SHUNT_curHi, SHUNT_curLo);  // SHUNT - V
	  CMPSS_DIG_FILTER(&Cmpss6Regs, SHUNT_curHi, SHUNT_curLo);  // SHUNT - U
#endif
	EDIS;

	// Check for PWM trip due to over current
	if (EPwm1Regs.TZFLG.bit.OST ||
		EPwm2Regs.TZFLG.bit.OST ||
		EPwm3Regs.TZFLG.bit.OST
	   )
	{
		// if any EPwm's OST is set, force OST on all three to DISABLE inverter
		EALLOW;
		  EPwm1Regs.TZFRC.bit.OST = 1;
		  EPwm2Regs.TZFRC.bit.OST = 1;
		  EPwm3Regs.TZFRC.bit.OST = 1;
		EDIS;
	    TripFlagDMC = 1;      // Trip on DMC (halt and IPM fault trip )
	    RunMotor = 0;
	}

	// If clear cmd received, reset PWM trip
	if (clearTripFlagDMC)
	{
		GpioDataRegs.GPBDAT.bit.GPIO41 = 0;  // clear the ocp latch in macro M6
		TripFlagDMC = 0;
		clearTripFlagDMC = 0;
		GpioDataRegs.GPBDAT.bit.GPIO41 = 1;

		// clear EPWM trip flags
		DELAY_US(1L);
		EALLOW;
		  // clear OST flags
		  EPwm1Regs.TZCLR.bit.OST = 1;
		  EPwm2Regs.TZCLR.bit.OST = 1;
		  EPwm3Regs.TZCLR.bit.OST = 1;

		  // clear DCAEVT1 flags
		  EPwm1Regs.TZCLR.bit.DCAEVT1 = 1;
		  EPwm2Regs.TZCLR.bit.DCAEVT1 = 1;
		  EPwm3Regs.TZCLR.bit.DCAEVT1 = 1;

		  // clear HLATCH - (not in TRIP gen path)
		  Cmpss1Regs.COMPSTSCLR.bit.HLATCHCLR = 1;
		  Cmpss3Regs.COMPSTSCLR.bit.HLATCHCLR = 1;
		  Cmpss2Regs.COMPSTSCLR.bit.HLATCHCLR = 1;
		  Cmpss6Regs.COMPSTSCLR.bit.HLATCHCLR = 1;

		  // clear LLATCH - (not in TRIP gen path)
		  Cmpss1Regs.COMPSTSCLR.bit.LLATCHCLR = 1;
		  Cmpss3Regs.COMPSTSCLR.bit.LLATCHCLR = 1;
		  Cmpss2Regs.COMPSTSCLR.bit.LLATCHCLR = 1;
		  Cmpss6Regs.COMPSTSCLR.bit.LLATCHCLR = 1;
		EDIS;
	}

	//-------------------
	//the next time CpuTimer0 'counter' reaches Period value go to A2
	A_Task_Ptr = &A2;
	//-------------------
}

//-----------------------------------------------------------------
void A2(void) // SPARE (not used)
//-----------------------------------------------------------------
{

	//-------------------
	//the next time CpuTimer0 'counter' reaches Period value go to A3
	A_Task_Ptr = &A3;
	//-------------------
}

//-----------------------------------------
void A3(void) // SPARE (not used)
//-----------------------------------------
{

	//-----------------
	//the next time CpuTimer0 'counter' reaches Period value go to A1
	A_Task_Ptr = &A1;
	//-----------------
}



//=================================================================================
//	B - TASKS (executed in every 100 usec)
//=================================================================================

//----------------------------------- USER ----------------------------------------

//----------------------------------------
void B1(void) // Toggle GPIO-00
//----------------------------------------
{

	//-----------------
	//the next time CpuTimer1 'counter' reaches Period value go to B2
	B_Task_Ptr = &B2;
	//-----------------
}

//----------------------------------------
void B2(void) //  SPARE
//----------------------------------------
{

	//-----------------
	//the next time CpuTimer1 'counter' reaches Period value go to B3
	B_Task_Ptr = &B3;
	//-----------------
}

//----------------------------------------
void B3(void) //  SPARE
//----------------------------------------
{

	//-----------------
	//the next time CpuTimer1 'counter' reaches Period value go to B1
	B_Task_Ptr = &B1;
	//-----------------
}


//=================================================================================
//	C - TASKS (executed in every 150 usec)
//=================================================================================

//--------------------------------- USER ------------------------------------------

//----------------------------------------
void C1(void) 	// Toggle GPIO-34
//----------------------------------------
{

	if(LedCnt1==0)
	{
//		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;  // LED blinking code
		LedCnt1 = 200;
	}
	else
		LedCnt1--;

	//-----------------
	//the next time CpuTimer2 'counter' reaches Period value go to C2
	C_Task_Ptr = &C2;
	//-----------------

}

//----------------------------------------
void C2(void) //  SPARE
//----------------------------------------
{

	//-----------------
	//the next time CpuTimer2 'counter' reaches Period value go to C3
	C_Task_Ptr = &C3;
	//-----------------
}


//-----------------------------------------
void C3(void) //  SPARE
//-----------------------------------------
{

	//-----------------
	//the next time CpuTimer2 'counter' reaches Period value go to C1
	C_Task_Ptr = &C1;
	//-----------------
}

// ****************************************************************************
// ****************************************************************************
//TODO Motor Control ISR
// ****************************************************************************
// ****************************************************************************
interrupt void MotorControlISR(void)
{

	EINT;

	// Verifying the ISR
    IsrTicker++;


// =============================== LEVEL 1 ======================================
//	  Checks target independent modules, duty cycle waveforms and PWM update
//	  Keep the motors disconnected at this level
// ==============================================================================

//TODO BUILD 1
#if (BUILDLEVEL == LEVEL1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    rc1.TargetValue = SpeedRef;
	RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
	RG_MACRO(rg1)

// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
//	There are two option for trigonometric functions:
//  IQ sin/cos look-up table provides 512 discrete sin and cos points in Q30 format
//  IQsin/cos PU functions interpolate the data in the lookup table yielding higher resolution.
// ------------------------------------------------------------------------------
    ipark1.Ds = VdTesting;
    ipark1.Qs = VqTesting;

    park1.Angle  = rg1.Out;
	park1.Sine   = __sinpuf32(park1.Angle);
	park1.Cosine = __cospuf32(park1.Angle);

	ipark1.Sine=park1.Sine;
    ipark1.Cosine=park1.Cosine;
	IPARK_MACRO(ipark1)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
  	svgen1.Ualpha = ipark1.Alpha;
 	svgen1.Ubeta  = ipark1.Beta;
	SVGENDQ_MACRO(svgen1)

// ------------------------------------------------------------------------------
//  Computed Duty and Write to CMPA register
// ------------------------------------------------------------------------------
 	EPwm1Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Ta)+INV_PWM_HALF_TBPRD;
	EPwm2Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tb)+INV_PWM_HALF_TBPRD;
	EPwm3Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tc)+INV_PWM_HALF_TBPRD;

// ------------------------------------------------------------------------------
//  Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
	DlogCh1 = rg1.Out;
	DlogCh2 = svgen1.Ta;
	DlogCh3 = svgen1.Tb;
	DlogCh4 = svgen1.Tc;

//------------------------------------------------------------------------------
// Variable display on DACs B and C
//------------------------------------------------------------------------------
	if (rslvrIn.TUNING)
	{
		DacbRegs.DACVALS.bit.DACVALS = rslvrOut.angleRaw*4096;
		DaccRegs.DACVALS.bit.DACVALS = rslvrOut.angleOut*4096;
	}
	else
	{
		DacbRegs.DACVALS.bit.DACVALS = (svgen1.Tb*0.5+0.5)*4096;
		DaccRegs.DACVALS.bit.DACVALS = (svgen1.Tc*0.5+0.5)*4096;
	}

#endif // (BUILDLEVEL==LEVEL1)

// =============================== LEVEL 2 ======================================
//	  Level 2 verifies
//	     - all current sense schems
//         - analog-to-digital conversion (shunt and LEM)
//         - SDFM function
//       - Current Limit Settings for over current protection
//       - clarke/park transformations (CLARKE/PARK)
//       - Position sensor interface
//         - speed estimation
// ==============================================================================

//TODO INCRBUILD 2
#if (BUILDLEVEL==LEVEL2)

	// ------------------------------------------------------------------------------
	// Alignment Routine: this routine aligns the motor to zero electrical angle
	// and in case of QEP also finds the index location and initializes the angle
	// w.r.t. the index location
	// ------------------------------------------------------------------------------
	if(!RunMotor)
		lsw = 0;
	else if (lsw == 0)
	{
		// for restarting from (RunMotor = 0)
		rc1.TargetValue =  rc1.SetpointValue = 0;

#if POSITION_ENCODER==QEP_POS_ENCODER
		lsw = 1;   // for QEP, spin the motor to find the index pulse
#else
		lsw  = 2;  // for absolute encoders no need for lsw=1
#endif
	} // end else if (lsw=0)

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
	if(lsw==0)rc1.TargetValue = 0;
	else rc1.TargetValue = SpeedRef;
	RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
	RG_MACRO(rg1)

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//	Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
	currentSensorSuite();
	clarke1.As = current_sensor[CURRENT_SENSE-1].As; // Phase A curr.
	clarke1.Bs = current_sensor[CURRENT_SENSE-1].Bs; // Phase B curr.
	CLARKE_MACRO(clarke1)
// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
	park1.Alpha  = clarke1.Alpha;
	park1.Beta   = clarke1.Beta;
	park1.Angle  = rg1.Out;
	park1.Sine   = __sinpuf32(park1.Angle);
	park1.Cosine = __cospuf32(park1.Angle);
	PARK_MACRO(park1)

// ------------------------------------------------------------------------------
//	Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
	ipark1.Ds = VdTesting;
    ipark1.Qs = VqTesting;

	ipark1.Sine=park1.Sine;
    ipark1.Cosine=park1.Cosine;
	IPARK_MACRO(ipark1)

// ------------------------------------------------------------------------------
//   Position encoder suite module
// ------------------------------------------------------------------------------
	posEncoderSuite();  // if needed reverse the sense of position in this module

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------
	speed1.ElecTheta = posEncElecTheta[POSITION_ENCODER];
	SPEED_FR_MACRO(speed1)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
  	svgen1.Ualpha = ipark1.Alpha;
 	svgen1.Ubeta  = ipark1.Beta;
	SVGENDQ_MACRO(svgen1)

// ------------------------------------------------------------------------------
//  Computed Duty and Write to CMPA register
// ------------------------------------------------------------------------------
	EPwm1Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tc)+INV_PWM_HALF_TBPRD;
	EPwm2Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Ta)+INV_PWM_HALF_TBPRD;
	EPwm3Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tb)+INV_PWM_HALF_TBPRD;

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
	DlogCh1 = rg1.Out;
	DlogCh2 = svgen1.Ta;
	DlogCh3 = clarke1.As;
	DlogCh4 = clarke1.Bs;

//------------------------------------------------------------------------------
// Variable display on DACs B and C
//------------------------------------------------------------------------------
	DacbRegs.DACVALS.bit.DACVALS = rg1.Out*4096;
	DaccRegs.DACVALS.bit.DACVALS = posEncElecTheta[POSITION_ENCODER]*4096;


#endif // (BUILDLEVEL==LEVEL2)


// =============================== LEVEL 3 ======================================
//	Level 3 verifies the dq-axis current regulation performed by PID and speed
//	measurement modules
//  lsw=0: lock the rotor of the motor
//  lsw=1: close the current loop
//  NOTE:- Iq loop is closed using internal ramp angle as position feedback.
//	       Therefore, motor speed does not race high with lighter load. User's
//         wanting to use actual rotor angle should ensure that the test value
//         for Iq reference will not race the motor to high speeds. In other
//         words, to use the actual angle, a loaded motor is needed.
// ==============================================================================

//TODO INCRBUILD 3
#if (BUILDLEVEL==LEVEL3)

	// ------------------------------------------------------------------------------
	// Alignment Routine: this routine aligns the motor to zero electrical angle
	// and in case of QEP also finds the index location and initializes the angle
	// w.r.t. the index location
	// ------------------------------------------------------------------------------
	if(!RunMotor)
	{
		lsw = 0;
		pi_id.ui = 0;
		pi_iq.ui = 0;
	}
	else if (lsw == 0)
	{
		// alignment current
		IdRef = IdRef_start;  //IQ(0.1);

		// for restarting from (RunMotor = 0)
		rc1.TargetValue =  rc1.SetpointValue = 0;

		// set up an alignment and hold time for shaft to settle down
		if (pi_id.Ref >= IdRef)
		{
			if (cntr < alignCnt)
				cntr++;
			else
			{
#if POSITION_ENCODER==QEP_POS_ENCODER
				lsw = 1;      // for QEP, spin the motor to find the index pulse
				IqRef = _IQ(0.05);
#else
				lsw = 2;   // for absolute encoders no need for lsw=1
#endif
				cntr  = 0;
				IdRef = IdRef_run;
			}
		}
	} // end else if (lsw=0)

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    if(lsw==0)rc1.TargetValue = 0;
    else rc1.TargetValue = SpeedRef;
	RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
	RG_MACRO(rg1)

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//	Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
	currentSensorSuite();
	clarke1.As = current_sensor[CURRENT_SENSE-1].As; // Phase A curr.
	clarke1.Bs = current_sensor[CURRENT_SENSE-1].Bs; // Phase B curr.
	CLARKE_MACRO(clarke1)

// ------------------------------------------------------------------------------
//   Position encoder suite module
// ------------------------------------------------------------------------------
	posEncoderSuite();  // if needed reverse the sense of position in this module

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------
	speed1.ElecTheta = posEncElecTheta[POSITION_ENCODER];
	SPEED_FR_MACRO(speed1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
	park1.Alpha = clarke1.Alpha;
	park1.Beta  = clarke1.Beta;

	if(lsw==0) park1.Angle = 0;
	else if (lsw==1)  park1.Angle = rg1.Out; // this state exists only for QEP
	else park1.Angle = rg1.Out; // posEncElecTheta[POSITION_ENCODER];

	park1.Sine   = __sinpuf32(park1.Angle);
	park1.Cosine = __cospuf32(park1.Angle);
	PARK_MACRO(park1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PID_REG3 module and call the PID IQ controller macro
// ------------------------------------------------------------------------------
//	if(lsw==0) 	pi_iq.Ref = 0;
//	else		pi_iq.Ref= IqRef;
	pi_iq.Ref =  IqRef * (lsw==0 ? 0 : 1);
	pi_iq.Fbk = park1.Qs;
	PI_MACRO(pi_iq)

// ------------------------------------------------------------------------------
//  Connect inputs of the PI module and call the PID ID controller macro
// ------------------------------------------------------------------------------
//	if(lsw==0)	pi_id.Ref = IdRef;
//	else		pi_id.Ref = IdRef;
	pi_id.Ref = ramper(IdRef, pi_id.Ref, _IQ(0.00001));
	pi_id.Fbk = park1.Ds;
	PI_MACRO(pi_id)

// ------------------------------------------------------------------------------
//	Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
    ipark1.Ds = pi_id.Out;
    ipark1.Qs = pi_iq.Out ;
	ipark1.Sine   = park1.Sine;
    ipark1.Cosine = park1.Cosine;
	IPARK_MACRO(ipark1)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
  	svgen1.Ualpha = ipark1.Alpha;
 	svgen1.Ubeta  = ipark1.Beta;
	SVGENDQ_MACRO(svgen1)

// ------------------------------------------------------------------------------
//  Computed Duty and Write to CMPA register
// ------------------------------------------------------------------------------
	EPwm1Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tc)+INV_PWM_HALF_TBPRD;
	EPwm2Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Ta)+INV_PWM_HALF_TBPRD;
	EPwm3Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tb)+INV_PWM_HALF_TBPRD;


// ------------------------------------------------------------------------------
//  Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
 	DlogCh1 = posEncElecTheta[POSITION_ENCODER];
 	DlogCh2 = rg1.Out;     // current_sensor[SHUNT_CURRENT_SENSE-1].As;
 	DlogCh3 = clarke1.As;  // current_sensor[LEM_CURRENT_SENSE-1].As;
 	DlogCh4 = clarke1.Bs;  // current_sensor[SD_CURRENT_SENSE-1].As;

//------------------------------------------------------------------------------
// Variable display on DACs B and C
//------------------------------------------------------------------------------
 	DacbRegs.DACVALS.bit.DACVALS = rg1.Out*4096;
 	DaccRegs.DACVALS.bit.DACVALS = posEncElecTheta[POSITION_ENCODER]*4096;

#endif // (BUILDLEVEL==LEVEL3)


// =============================== LEVEL 4 ======================================
//	  Level 4 verifies the speed regulator performed by PID module.
//	  The system speed loop is closed by using the measured speed as feedback
//  lsw=0: lock the rotor of the motor
//  lsw=1: - needed only with QEP encoders until first index pulse
//         - Loops shown for lsw=2 are closed in this stage
//  lsw=2: close speed loop and current loops Id, Iq
// ==============================================================================

//TODO INCRBUILD 4
#if (BUILDLEVEL==LEVEL4)

// ------------------------------------------------------------------------------
// Alignment Routine: this routine aligns the motor to zero electrical angle
// and in case of QEP also finds the index location and initializes the angle
// w.r.t. the index location
// ------------------------------------------------------------------------------
	if(!RunMotor)
		lsw = 0;
	else if (lsw == 0)
	{
		// alignment current
		IdRef = IdRef_start;  //IQ(0.1);

		// for restarting from (RunMotor = 0)
		rc1.TargetValue =  rc1.SetpointValue = 0;

		// set up an alignment and hold time for shaft to settle down
		if (pi_id.Ref >= IdRef)
		{
			if (cntr < alignCnt)
				cntr++;
			else
			{
				cntr  = 0;
				IdRef = IdRef_run;
#if POSITION_ENCODER==QEP_POS_ENCODER
				lsw = 1;       // for QEP, spin the motor to find the index pulse
				IqRef = _IQ(0.05);
#else
				lsw = 2;    // for absolute encoders no need for lsw=1
#endif
			}
		}
	} // end else if (lsw=0)

// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
    if(lsw==0)rc1.TargetValue = 0;
    else rc1.TargetValue = SpeedRef;
	RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;
	RG_MACRO(rg1)

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//	Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
	currentSensorSuite();
	clarke1.As = current_sensor[CURRENT_SENSE-1].As; // Phase A curr.
	clarke1.Bs = current_sensor[CURRENT_SENSE-1].Bs; // Phase B curr.
	CLARKE_MACRO(clarke1)

// ------------------------------------------------------------------------------
//   Position encoder suite module
// ------------------------------------------------------------------------------
	posEncoderSuite();  // if needed reverse the sense of position in this module

// ------------------------------------------------------------------------------
//    Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------
	speed1.ElecTheta = posEncElecTheta[POSITION_ENCODER];
	SPEED_FR_MACRO(speed1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
	park1.Alpha = clarke1.Alpha;
	park1.Beta  = clarke1.Beta;

	if(lsw==0) park1.Angle = 0;
//	else if (lsw==1)  park1.Angle =  rg1.Out; // this state exists only for QEP
	else park1.Angle = posEncElecTheta[POSITION_ENCODER];

	park1.Sine   = __sinpuf32(park1.Angle);
	park1.Cosine = __cospuf32(park1.Angle);

	PARK_MACRO(park1)

// ------------------------------------------------------------------------------
//    Connect inputs of the PI module and call the PID speed controller macro
// ------------------------------------------------------------------------------
   if (SpeedLoopCount==SpeedLoopPrescaler)
     {
		pid_spd.term.Ref = rc1.SetpointValue;  //SpeedRef;
		pid_spd.term.Fbk = speed1.Speed;
		PID_MACRO(pid_spd);

		SpeedLoopCount = 1;
     }
	else SpeedLoopCount++;

	if(lsw==0 || lsw==1)
	{
		pid_spd.data.d1 = 0; pid_spd.data.d2 = 0; pid_spd.data.i1 = 0;
		pid_spd.data.ud = 0; pid_spd.data.ui = 0; pid_spd.data.up = 0;
	}

// ------------------------------------------------------------------------------
//    Connect inputs of the PI module and call the PID IQ controller macro
// ------------------------------------------------------------------------------
	if(lsw==0)
	{
		pi_iq.Ref = 0;
		pi_iq.ui  = 0;
	}
    else if(lsw==1) pi_iq.Ref = IqRef;
    else pi_iq.Ref = pid_spd.term.Out;
	pi_iq.Fbk = park1.Qs;
	PI_MACRO(pi_iq)

// ------------------------------------------------------------------------------
//    Connect inputs of the PI module and call the PID ID controller macro
// ------------------------------------------------------------------------------
	pi_id.Ref=ramper(IdRef, pi_id.Ref, _IQ(0.00001));
	pi_id.Fbk = park1.Ds;
	PI_MACRO(pi_id)

// ------------------------------------------------------------------------------
//	Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
	ipark1.Ds = pi_id.Out;
	ipark1.Qs = pi_iq.Out ;
	ipark1.Sine   = park1.Sine;
	ipark1.Cosine = park1.Cosine;
	IPARK_MACRO(ipark1)

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
	svgen1.Ualpha = ipark1.Alpha;
	svgen1.Ubeta  = ipark1.Beta;
	SVGENDQ_MACRO(svgen1)

// ------------------------------------------------------------------------------
//  Computed Duty and Write to CMPA register
// ------------------------------------------------------------------------------
	EPwm1Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tc)+INV_PWM_HALF_TBPRD;
	EPwm2Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Ta)+INV_PWM_HALF_TBPRD;
	EPwm3Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tb)+INV_PWM_HALF_TBPRD;

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
	DlogCh1 = posEncElecTheta[POSITION_ENCODER];
	DlogCh2 = svgen1.Ta;
	DlogCh3 = clarke1.As;
	DlogCh4 = clarke1.Bs;

//------------------------------------------------------------------------------
// Variable display on DACs B and C
//------------------------------------------------------------------------------
 	DacbRegs.DACVALS.bit.DACVALS = rg1.Out*4096;
 	DaccRegs.DACVALS.bit.DACVALS = posEncElecTheta[POSITION_ENCODER]*4096;

#endif // (BUILDLEVEL==LEVEL4)


// =============================== LEVEL 5 ======================================
//  Level 5 verifies the position control
//  Position references generated locally
//  lsw=0: lock the rotor of the motor
//  lsw=1: - needed only with QEP encoders until first index pulse
//         - Loops shown for lsw=2 are closed in this stage
//  lsw=2: close all loops, position, speed and currents Id, Iq
// ==============================================================================

//TODO INCRBUILD 5
#if (BUILDLEVEL==LEVEL5)

	// ------------------------------------------------------------------------------
	//  Connect inputs of the RMP module and call the ramp control macro
	// ------------------------------------------------------------------------------
	if(!RunMotor)
		lsw = 0;
	else if (lsw == 0)
	{
		// alignment curretnt
		IdRef = IdRef_start;  //IQ(0.1);

		// for restarting from (RunMotor = 0)
		rc1.TargetValue = rc1.SetpointValue = 0;

		// set up an alignment and hold time for shaft to settle down
		if (pi_id.Ref >= IdRef)
		{
			if (cntr < alignCnt)
				cntr++;
			else
			{
				cntr  = 0;
				IdRef = IdRef_run;
#if POSITION_ENCODER==QEP_POS_ENCODER
				// for QEP because it is an incremental encoder spin the motor to find the index pulse
				lsw=1;
				IqRef= _IQ(0.05); // make the motor spin to get an index pulse
#else
				// for absolute encoders no need for lsw=1
				lsw = 2;
#endif
			}
		}
	} // end else if (lsw=0)

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//	Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
	currentSensorSuite();
	clarke1.As = current_sensor[CURRENT_SENSE-1].As; // Phase A curr.
	clarke1.Bs = current_sensor[CURRENT_SENSE-1].Bs; // Phase B curr.
	CLARKE_MACRO(clarke1)

// ------------------------------------------------------------------------------
//   Position encoder suite module
// ------------------------------------------------------------------------------
	posEncoderSuite();  // if needed reverse the sense of position in this module

// ------------------------------------------------------------------------------
//  Connect inputs of the SPEED_FR module and call the speed calculation macro
// ------------------------------------------------------------------------------
	speed1.ElecTheta = posEncElecTheta[POSITION_ENCODER];
	SPEED_FR_MACRO(speed1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
	park1.Alpha = clarke1.Alpha;
	park1.Beta  = clarke1.Beta;

	if(lsw==0) park1.Angle = 0;
//	else if (lsw==1)  park1.Angle =  rg1.Out; // this state exists only for QEP
	else park1.Angle = posEncElecTheta[POSITION_ENCODER];

	park1.Sine   = __sinpuf32(park1.Angle);
	park1.Cosine = __cospuf32(park1.Angle);
	PARK_MACRO(park1)

// ------------------------------------------------------------------------------
//    Connect inputs of the PID module and call the PID speed controller macro
// ------------------------------------------------------------------------------
	if (++SpeedLoopCount >= SpeedLoopPrescaler)
	{
		SpeedLoopCount=0;
		if (lsw == 0)
		{
			rc1.SetpointValue = 0;  // position = 0 deg
			pid_spd.data.d1 = 0; pid_spd.data.d2 = 0; pid_spd.data.i1 = 0;
			pid_spd.data.ud = 0; pid_spd.data.ui = 0; pid_spd.data.up = 0;
			pi_pos.ui = 0;
		}
		else
		{

		// ========== reference position setting =========
		// --------------------------------------------------------------------
		//  Connect inputs of RAMP GEN module and call ramp generator macro
		// --------------------------------------------------------------------
		rg1.Freq = SpeedRef;
		RG_MACRO(rg1)

		// choose between one of  two position commands - watch Ref and Fbk
		// Position command read from a table
		   rc1.TargetValue = refPosGen(rc1.TargetValue);
		// Position command generated as integral of SpeedRef
//		   rc1.TargetValue = rg1.Out;

		rc1.SetpointValue = _IQfrac(rc1.TargetValue);
		// Rolling in angle within 0 to 1pu
		if (rc1.SetpointValue < 0)
			rc1.SetpointValue += _IQ(1.0);

		pi_pos.Ref = rc1.SetpointValue;
		pi_pos.Fbk = posEncMechTheta[POSITION_ENCODER];
		PI_POS_MACRO(pi_pos);

		// speed PI regulator
		pid_spd.term.Ref = pi_pos.Out;     //	pid_spd.term.Ref = rc1.SetpointValue;
		pid_spd.term.Fbk = speed1.Speed;
		PID_MACRO(pid_spd);                //	pi_spd.Out = pi_pos.Out;
		}
	}

// ------------------------------------------------------------------------------
//    Connect inputs of the PID module and call the PID IQ controller macro
// ------------------------------------------------------------------------------
	if(lsw==0)
	{
		pi_iq.Ref = 0;
		pi_iq.ui  = 0;
	}
//	else if(lsw==1) pi_iq.Ref = IqRef;
	else pi_iq.Ref =  pid_spd.term.Out;
	pi_iq.Fbk = park1.Qs;
	PI_MACRO(pi_iq);

// ------------------------------------------------------------------------------
//    Connect inputs of the PID module and call the PID ID controller macro
// ------------------------------------------------------------------------------
	pi_id.Ref = ramper(IdRef, pi_id.Ref, _IQ(0.00001));  //ramprate = 1pu/s
	pi_id.Fbk = park1.Ds;
	PI_MACRO(pi_id);

// ------------------------------------------------------------------------------
//  Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
	ipark1.Qs = pi_iq.Out;
	ipark1.Ds = pi_id.Out;
	ipark1.Sine   = park1.Sine;
	ipark1.Cosine = park1.Cosine;
	IPARK_MACRO(ipark1);

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
	svgen1.Ualpha = ipark1.Alpha;
	svgen1.Ubeta  = ipark1.Beta;
	SVGENDQ_MACRO(svgen1)

// ------------------------------------------------------------------------------
//  Computed Duty and Write to CMPA register
// ------------------------------------------------------------------------------
	EPwm1Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tc)+INV_PWM_HALF_TBPRD;
	EPwm2Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Ta)+INV_PWM_HALF_TBPRD;
	EPwm3Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tb)+INV_PWM_HALF_TBPRD;

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
	DlogCh1 = rg1.Out;
	DlogCh2 = park1.Ds;
	DlogCh3 = park1.Qs;
	DlogCh4 = park1.Beta;

//------------------------------------------------------------------------------
// Variable display on DACs B and C
//------------------------------------------------------------------------------
	DacbRegs.DACVALS.bit.DACVALS = pi_pos.Ref*4096; //
	DaccRegs.DACVALS.bit.DACVALS = pi_pos.Fbk*4096; //

#endif // (BUILDLEVEL==LEVEL5)


// ------------------------------------------------------------------------------
//    Call the DATALOG update function.
// ------------------------------------------------------------------------------
	DLOG_4CH_F_FUNC(&dlog_4ch1);

    EPwm11Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all   = PIEACK_GROUP3;

}// MainISR Ends Here


// ****************************************************************************
// ****************************************************************************
//TODO  DMC Protection Against Over Current Protection
// ****************************************************************************
// ****************************************************************************

//definitions for selecting DACH reference
#define REFERENCE_VDDA     0

//definitions for COMPH input selection
#define NEGIN_DAC          0
#define NEGIN_PIN          1

//definitions for CTRIPH/CTRIPOUTH output selection
#define CTRIP_ASYNCH       0
#define CTRIP_SYNCH        1
#define CTRIP_FILTER       2
#define CTRIP_LATCH        3

void cmpssConfig(volatile struct CMPSS_REGS *v, int16 Hi, int16 Lo)
{
	// Set up COMPCTL register
	v->COMPCTL.bit.COMPDACE    = 1;             // Enable CMPSS
	v->COMPCTL.bit.COMPLSOURCE = NEGIN_DAC;     // NEG signal from DAC for COMP-L
	v->COMPCTL.bit.COMPHSOURCE = NEGIN_DAC;     // NEG signal from DAC for COMP-H
	v->COMPCTL.bit.COMPHINV    = 0;             // COMP-H output is NOT inverted
	v->COMPCTL.bit.COMPLINV    = 1;             // COMP-L output is inverted
	v->COMPCTL.bit.ASYNCHEN    = 0;             // Disable aynch COMP-H ouput
	v->COMPCTL.bit.ASYNCLEN    = 0;             // Disable aynch COMP-L ouput
	v->COMPCTL.bit.CTRIPHSEL    = CTRIP_FILTER; // Dig filter output ==> CTRIPH
	v->COMPCTL.bit.CTRIPOUTHSEL = CTRIP_FILTER; // Dig filter output ==> CTRIPOUTH
	v->COMPCTL.bit.CTRIPLSEL    = CTRIP_FILTER; // Dig filter output ==> CTRIPL
	v->COMPCTL.bit.CTRIPOUTLSEL = CTRIP_FILTER; // Dig filter output ==> CTRIPOUTL

	// Set up COMPHYSCTL register
	v->COMPHYSCTL.bit.COMPHYS   = 2; // COMP hysteresis set to 2x typical value

	// set up COMPDACCTL register
	v->COMPDACCTL.bit.SELREF    = 0; // VDDA is REF for CMPSS DACs
	v->COMPDACCTL.bit.SWLOADSEL = 0; // DAC updated on sysclock
	v->COMPDACCTL.bit.DACSOURCE = 0; // Ramp bypassed

	// Load DACs - High and Low
	v->DACHVALS.bit.DACVAL = Hi;     // Set DAC-H to allowed MAX +ve current
	v->DACLVALS.bit.DACVAL = Lo;     // Set DAC-L to allowed MAX -ve current

	// digital filter settings - HIGH side
	v->CTRIPHFILCLKCTL.bit.CLKPRESCALE = clkPrescale; // set time between samples, max : 1023
	v->CTRIPHFILCTL.bit.SAMPWIN        = sampwin;     // # of samples in window, max : 31
	v->CTRIPHFILCTL.bit.THRESH         = thresh;      // recommended : thresh > sampwin/2
	v->CTRIPHFILCTL.bit.FILINIT        = 1;           // Init samples to filter input value

	// digital filter settings - LOW side
	v->CTRIPLFILCLKCTL.bit.CLKPRESCALE = clkPrescale; // set time between samples, max : 1023
	v->CTRIPLFILCTL.bit.SAMPWIN        = sampwin;     // # of samples in window, max : 31
	v->CTRIPLFILCTL.bit.THRESH         = thresh;      // recommended : thresh > sampwin/2
	v->CTRIPLFILCTL.bit.FILINIT        = 1;           // Init samples to filter input value

	// Clear the status register for latched comparator events
	v->COMPSTSCLR.bit.HLATCHCLR = 1;
	v->COMPSTSCLR.bit.LLATCHCLR = 1;

	return;
}


void HVDMC_Protection(void)
{
	EALLOW;

	// Configure GPIO used for Trip Mechanism

	//GPIO input for reading the status of the LEM-overcurrent macro block (active low), GPIO40
	//could trip PWM based on this, if desired
	// Configure as Input
	GpioCtrlRegs.GPBPUD.bit.GPIO40  = 1; // disable pull ups
	GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0; // choose GPIO for mux option
	GpioCtrlRegs.GPBDIR.bit.GPIO40  = 0; // set as input
	GpioCtrlRegs.GPBINV.bit.GPIO40  = 1; //invert the input such that '0' is good and '1' is bad after inversion

	InputXbarRegs.INPUT2SELECT = 40;     //Select GPIO40 as INPUTXBAR2

	//Clearing the Fault(active low), GPIO41,
	// Configure as Output
	GpioCtrlRegs.GPBPUD.bit.GPIO41  = 1; // disable pull ups
	GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 0; // choose GPIO for mux option
	GpioCtrlRegs.GPBDIR.bit.GPIO41  = 1; // set as output
	GpioDataRegs.GPBSET.bit.GPIO41  = 1;

	//Forcing IPM Shutdown (Trip) using GPIO58 (Active high)
	// Configure as Output
	GpioCtrlRegs.GPBPUD.bit.GPIO58   = 1; // disable pull ups
	GpioCtrlRegs.GPBMUX2.bit.GPIO58  = 0; // choose GPIO for mux option
	GpioCtrlRegs.GPBDIR.bit.GPIO58   = 1; // set as output
	GpioDataRegs.GPBCLEAR.bit.GPIO58 = 1;

	// LEM Current phase V(ADC A2, COMP1) and W(ADC B2, COMP3), High Low Compare event trips
	LEM_curHi = 2048 + LEM(curLimit);
	LEM_curLo = 2048 - LEM(curLimit);
	cmpssConfig(&Cmpss1Regs, LEM_curHi, LEM_curLo);  //Enable CMPSS1 - LEM V
	cmpssConfig(&Cmpss3Regs, LEM_curHi, LEM_curLo);  //Enable CMPSS3 - LEM W

#if (CNGD == HOT)
	// Shunt Current phase V(ADC A4, COMP2) and W(ADC C2, COMP6), High Low Compare event trips
	SHUNT_curHi = 2048 + SHUNT(curLimit);
	SHUNT_curLo = 2048 - SHUNT(curLimit);
	cmpssConfig(&Cmpss2Regs, SHUNT_curHi, SHUNT_curLo);  //Enable CMPSS2 - Shunt V
	cmpssConfig(&Cmpss6Regs, SHUNT_curHi, SHUNT_curLo);  //Enable CMPSS6 - Shunt U
#endif

	// Configure TRIP 4 to OR the High and Low trips from both comparator 1 & 3
	// Clear everything first
	EPwmXbarRegs.TRIP4MUX0TO15CFG.all  = 0x0000;
	EPwmXbarRegs.TRIP4MUX16TO31CFG.all = 0x0000;
	// Enable Muxes for ored input of CMPSS1H and 1L, i.e. .1 mux for Mux0
	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX0  = 1;  //cmpss1 - tripH or tripL
	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX4  = 1;  //cmpss3 - tripH or tripL
	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX2  = 1;  //cmpss2 - tripH or tripL
	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX10 = 1;  //cmpss6 - tripH or tripL
	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX3  = 1;  //inputxbar2 trip

	// Disable all the muxes first
	EPwmXbarRegs.TRIP4MUXENABLE.all = 0x0000;
	// Enable Mux 0  OR Mux 4 to generate TRIP4
	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX0  = 1;
	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX4  = 1;
	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX2  = 1;
	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX10 = 1;
	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX1  = 1;

	EPwm1Regs.DCTRIPSEL.bit.DCAHCOMPSEL = 3; //Trip 4 is the input to the DCAHCOMPSEL
	EPwm1Regs.TZDCSEL.bit.DCAEVT1       = TZ_DCAH_HI;
	EPwm1Regs.DCACTL.bit.EVT1SRCSEL     = DC_EVT1;
	EPwm1Regs.DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;
	EPwm1Regs.TZSEL.bit.DCAEVT1         = 1;           // 1/0 - Enable/Disable One Shot Mode

	EPwm2Regs.DCTRIPSEL.bit.DCAHCOMPSEL = 3; //Trip 4 is the input to the DCAHCOMPSEL
	EPwm2Regs.TZDCSEL.bit.DCAEVT1       = TZ_DCAH_HI;
	EPwm2Regs.DCACTL.bit.EVT1SRCSEL     = DC_EVT1;
	EPwm2Regs.DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;
	EPwm2Regs.TZSEL.bit.DCAEVT1         = 1;

	EPwm3Regs.DCTRIPSEL.bit.DCAHCOMPSEL = 3; //Trip 4 is the input to the DCAHCOMPSEL
	EPwm3Regs.TZDCSEL.bit.DCAEVT1       = TZ_DCAH_HI;
	EPwm3Regs.DCACTL.bit.EVT1SRCSEL     = DC_EVT1;
	EPwm3Regs.DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;
	EPwm3Regs.TZSEL.bit.DCAEVT1         = 1;

	EPwm1Regs.TZSEL.bit.CBC6 = 0x1; // Emulator Stop
	EPwm2Regs.TZSEL.bit.CBC6 = 0x1; // Emulator Stop
	EPwm3Regs.TZSEL.bit.CBC6 = 0x1; // Emulator Stop

	// What do we want the OST/CBC events to do?
	// TZA events can force EPWMxA
	// TZB events can force EPWMxB

	EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low
	EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low

	EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low
	EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low

	EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low
	EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low

	// Clear any spurious OV trip
	EPwm1Regs.TZCLR.bit.DCAEVT1 = 1;
	EPwm2Regs.TZCLR.bit.DCAEVT1 = 1;
	EPwm3Regs.TZCLR.bit.DCAEVT1 = 1;

	EPwm1Regs.TZCLR.bit.OST = 1;
	EPwm2Regs.TZCLR.bit.OST = 1;
	EPwm3Regs.TZCLR.bit.OST = 1;

	EDIS;

//************************** End of Prot. Conf. ***************************//
}

// ****************************************************************************
// ****************************************************************************
//TODO PWM Configuration
// ****************************************************************************
// ****************************************************************************
void PWM_1ch_UpDwnCnt_CNF(int16 n, Uint16 period, int16 db) {
	EALLOW;
	// Time Base SubModule Registers
	(*ePWM[n]).TBCTL.bit.PRDLD = TB_IMMEDIATE; // set Immediate load
	(*ePWM[n]).TBPRD = period / 2; // PWM frequency = 1 / period
	(*ePWM[n]).TBPHS.bit.TBPHS = 0;
	(*ePWM[n]).TBCTR = 0;
	(*ePWM[n]).TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;
	(*ePWM[n]).TBCTL.bit.HSPCLKDIV = TB_DIV1;
	(*ePWM[n]).TBCTL.bit.CLKDIV    = TB_DIV1;

	(*ePWM[n]).TBCTL.bit.PHSEN    = TB_DISABLE;
	(*ePWM[n]).TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // sync "down-stream"

	// Counter Compare Submodule Registers
	(*ePWM[n]).CMPA.bit.CMPA = 0; // set duty 0% initially
	(*ePWM[n]).CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	(*ePWM[n]).CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

	// Action Qualifier SubModule Registers
	(*ePWM[n]).AQCTLA.bit.CAU = AQ_CLEAR;
	(*ePWM[n]).AQCTLA.bit.CAD = AQ_SET;

	// Active high complementary PWMs - Set up the deadband
	(*ePWM[n]).DBCTL.bit.IN_MODE  = DBA_ALL;
	(*ePWM[n]).DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	(*ePWM[n]).DBCTL.bit.POLSEL   = DB_ACTV_HIC;
	(*ePWM[n]).DBRED = db;
	(*ePWM[n]).DBFED = db;
	EDIS;
}

void PWM_1ch_UpCnt_CNF(int16 n, Uint16 period) {
	EALLOW;
	// Time Base SubModule Registers
	(*ePWM[n]).TBCTL.bit.PRDLD = TB_IMMEDIATE; // set Immediate load
	(*ePWM[n]).TBPRD = period-1; // PWM frequency = 1 / period
	(*ePWM[n]).TBPHS.bit.TBPHS = 0;
	(*ePWM[n]).TBCTR = 0;
	(*ePWM[n]).TBCTL.bit.CTRMODE   = TB_COUNT_UP;
	(*ePWM[n]).TBCTL.bit.HSPCLKDIV = TB_DIV1;
	(*ePWM[n]).TBCTL.bit.CLKDIV    = TB_DIV1;

	(*ePWM[n]).TBCTL.bit.PHSEN    = TB_DISABLE;
	(*ePWM[n]).TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // sync "down-stream"

	// Counter Compare Submodule Registers
	(*ePWM[n]).CMPA.bit.CMPA        = 0; // set duty 0% initially
	(*ePWM[n]).CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	(*ePWM[n]).CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

	// Action Qualifier SubModule Registers
	(*ePWM[n]).AQCTLA.bit.CAU = AQ_CLEAR;
	(*ePWM[n]).AQCTLA.bit.ZRO = AQ_SET;

	// Active high complementary PWMs - Set up the deadband
	(*ePWM[n]).DBCTL.bit.IN_MODE  = DBA_ALL;
	(*ePWM[n]).DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	(*ePWM[n]).DBCTL.bit.POLSEL   = DB_ACTV_HIC;
	(*ePWM[n]).DBRED = 0;
	(*ePWM[n]).DBFED = 0;
	EDIS;
}

// ****************************************************************************
// ****************************************************************************
//TODO ADC Configuration
// ****************************************************************************
// ****************************************************************************
void ConfigureADC(void)
{
	//Write ADC configurations and power up the ADC for both ADC A
	Uint16 i;

	EALLOW;

	//write configurations for ADC-A
	// External REFERENCE must be provided
	AdcaRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
	AdcaRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
	AdcaRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;

	//Set pulse positions to late
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

	//power up the ADC
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	//write configurations for ADC-B
	// External REFERENCE must be provided
	AdcbRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
	AdcbRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
	AdcbRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;

	//Set pulse positions to late
	AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

	//power up the ADC
	AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

//	//write configurations for ADC-C
	// External REFERENCE must be provided
	AdccRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
	AdccRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
	AdccRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;

	//Set pulse positions to late
	AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;

	//power up the ADC
	AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
//
//	//write configurations for ADC-D
	// External REFERENCE must be provided
	AdcdRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
	AdcdRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
	AdcdRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;

	//Set pulse positions to late
	AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;

	//power up the ADC
	AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	//delay for > 1ms to allow ADC time to power up
	for(i = 0; i < 1000; i++){
		asm("   RPT#255 || NOP");
	}

	EDIS;
}

// ****************************************************************************
// ****************************************************************************
// POSITION LOOP UTILITY FUNCTIONS
// ****************************************************************************
// ****************************************************************************

// slew programmable ramper
_iq ramper(_iq in, _iq out, _iq rampDelta) {
	_iq err;

	err = in - out;
	if (err > rampDelta)
		return(out + rampDelta);
  	else if (err < -rampDelta)
  		return(out - rampDelta);
    else
    	return(in);
}

// Ramp Controller for speed reference (Not currently used)
_iq ramper_speed(_iq in, _iq out, _iq rampDelta) {
	_iq err;

	err = in - out;
	if (err > rampDelta)
	{
		if((out+rampDelta)>1.0)
			return(1.0);
		else
			return (out+rampDelta);
	}
  	else if (err < -rampDelta)
  	{
  		if(out-rampDelta<=0.0)
  			return(0.0);
  		else
  			return(out - rampDelta);
  	}
    else
    	return(in);
}

// Reference Position Generator for position loop
_iq refPosGen(_iq out)
{
	_iq in = posArray[ptr1];

	out = ramper(in, out, posSlewRate);

	if (in == out)
	if (++cntr1 > 1000)
	{
		cntr1 = 0;
		if (++ptr1 >= ptrMax)
			ptr1 = 0;
	}
	return (out);
}

/****************************************************************************
 * End of Code *
 * ***************************************************************************
 */
