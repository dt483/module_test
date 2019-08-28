/**
    \file LMX249x.h
    \author Dennis, Assad Ali, Thomas Finke
 	\brief This file includes the function declarations for Texas Instrument's PLL named LMX249x
*/

	/*THERE iS SOME CHANGES IN THIS FILE */

/* ===========================================================================
** Copyright (C) 2017 Infineon Technologies AG
** All rights reserved.
** ===========================================================================
**
** ===========================================================================
** This document contains proprietary information of Infineon Technologies AG.
** Passing on and copying of this document, and communication of its contents
** is not permitted without Infineon's prior written authorization.
** ===========================================================================
*/

#ifndef DRIVER_PLL_LMX249x_H_
#define DRIVER_PLL_LMX249x_H_

/**************************************************************************************************/
/* Includes                                                                                       */
/**************************************************************************************************/
#include <stdint.h>
#include <stddef.h>

/**************************************************************************************************/
/* Enumeration types                                                                              */
/**************************************************************************************************/

// #pragma message "Rework documentation of enum types"

/*!
 * Determines what event is necessary to cause the state machine to go to the next ramp.
 * It can be set to when the RAMPx_LEN counter reaches zero or one of the events for Triggers A,B, or C.
 */
typedef enum
{
  LMX249x_RAMP_NEXT_TRIG_RAMPX_LEN  = 0,
  LMX249x_RAMP_NEXT_TRIG_TRIGA      = 1,
  LMX249x_RAMP_NEXT_TRIG_TRIGB      = 2,
  LMX249x_RAMP_NEXT_TRIG_TRIGC      = 3
} LMX249x_RampNextTrig_e;

typedef enum
{
  LMX249x_RAMP_COUNT_INCREMENT_SEGMENT_TRANSITION  = 0,
  LMX249x_RAMP_COUNT_INCREMENT_TRIG_TRIGA      = 1,
  LMX249x_RAMP_COUNT_INCREMENT_TRIG_TRIGB      = 2,
  LMX249x_RAMP_COUNT_INCREMENT_TRIG_TRIGC      = 3
} LMX249x_RampTrigInc_e;

/*!
 * LMX249x Ramp reset
 */
typedef enum
{
  LMX249x_RAMP_RST_DISABLE  = 0,
  LMX249x_RAMP_RST_ENABLE   = 1
} LMX249x_RampReset_e;

/*!
 * Ramp flags
 */
typedef enum
{
  LMX249x_RAMP_FLAG_CLR_BOTH  = 0,
  LMX249x_RAMP_FLAG_SET_FLAG0_CLR_FLAG1 = 1,
  LMX249x_RAMP_FLAG_SET_FLAG1_CLR_FLAG0 = 2,
  LMX249x_RAMP_FLAG_SET_BOTH  = 3
} LMX249x_RampFlag_e;

/*!
 * Ramp fastlock enable
 */
typedef enum
{
  LMX249x_RAMP_FASTLOCK_DISABLED  = 0,  /*!< disable fastlock on on specific ramp */
  LMX249x_RAMP_FASTLOCK_ENABLED   = 1   /*!< enable fastlock on on specific ramp */
} LMX249x_RampFastLock_e;

/*!
 * PLL_LMX249x_RAMP_FastLock_disabled
 */
typedef enum
{
  LMX249x_RAMP_NO_COMPARATOR        = 0x00,
  LMX249x_RAMP_USE_COMPARATOR_0     = 0x01,
  LMX249x_RAMP_USE_COMPARATOR_1     = 0x02,
  LMX249x_RAMP_USE_BOTH_COMPARATORS = 0x03
} LMX249x_RampComparators_e;

/*!
 * Ramp trigger definitions
 */
typedef enum
{
  LMX249x_RAMP_TRIG_NEVER_TRIGGERS              =  0,
  LMX249x_RAMP_TRIG_TRIG1_TERMINAL_RISING_EDGE  =  1,
  LMX249x_RAMP_TRIG_TRIG2_TERMINAL_RISING_EDGE  =  2,
  LMX249x_RAMP_TRIG_MOD_TERMINAL_RISING_EDGE    =  3,
  LMX249x_RAMP_TRIG_DLD_RISING_EDGE             =  4,
  LMX249x_RAMP_TRIG_CMP0_DETECTED_LEVEL         =  5,
  LMX249x_RAMP_TRIG_RAMPX_CPG_RISING_EDGE       =  6,
  LMX249x_RAMP_TRIG_RAMPX_FLAG0_RISING_EDGE     =  7,
  LMX249x_RAMP_TRIG_ALWAYS_TRIGGERED_LEVEL      =  8,
  LMX249x_RAMP_TRIG_TRIG1_TERMINAL_FALLING_EDGE =  9,
  LMX249x_RAMP_TRIG_TRIG2_TERMINAL_FALLING_EDGE = 10,
  LMX249x_RAMP_TRIG_MOD_TERMINAL_FALLING_EDGE   = 11,
  LMX249x_RAMP_TRIG_DLD_FALLING_EDGE            = 12,
  LMX249x_RAMP_TRIG_CMP1_DETECTED_LEVEL         = 13,
  LMX249x_RAMP_TRIG_RAMPX_CPG_FALLING_EDGE      = 14,
  LMX249x_RAMP_TRIG_RAMPX_FLAG0_FALLING_EDGE    = 15
} LMX249x_RampTriggerSource_e;

/*!
 * Clock Source for ramp length control
 */
typedef enum
{
  LMX249x_RAMP_CLOCK_INTERNAL = 0,  /*!< 0..ramp clock derived from the phase detector */
  LMX249x_RAMP_CLOCK_EXTERNAL = 1   /*!< 1 clock derived from MOD-Terminal */
} LMX249x_RampClock_e;

/*!
 * Type of modulation controlled by the ramp
 */
typedef enum
{
  LMX249x_RAMP_MODULATION_FM  = 0,  /*!< Enable Frequency Modulation */
  LMX249x_RAMP_MODULTAION_PM  = 1   /*!< Enable Phase Modulation */
} LMX249x_RampModulation_e;

/*!
 * FSK trigger type
 */
typedef enum
{
  LMX249x_TRIGGER_ALWAYS  = 0,  /*!< FSK Deviation is always applied */
  LMX249x_TRIGGER_A       = 1,  /*!< FSK Deviation is triggered by Trigger A */
  LMX249x_TRIGGER_B       = 2,  /*!< FSK Deviation is triggered by Trigger B */
  LMX249x_TRIGGER_C       = 3   /*!< FSK Deviation is triggered by Trigger C */
} LMX249x_TriggerSelect_e;

typedef enum
{
  LMX249x_RAMP_AUTO_OFF_DISABLE = 0,  /*!< Ramp continues regardless of the ramp counter */
  LMX249x_RAMP_AUTO_OFF_ENABLE  = 1   /*!< Ramp is turned off after a certain number of ramps */
} LMX249x_RampAutoOff_e;

/*!
 * fractional order
 */
typedef enum
{
  LMX249x_FRAC_ORD_INT    = 0,
  LMX249x_FRAC_ORD_FIRST  = 1,
  LMX249x_FRAC_ORD_SECOND = 2,
  LMX249x_FRAC_ORD_THIRD  = 3,
  LMX249x_FRAC_ORD_FOURTH = 4
} LMX249x_FracOrder_e;

/*!
 * fractional dithering
 */
typedef enum
{
  LMX249x_FRAC_DITHER_WEAK      = 0,
  LMX249x_FRAC_DITHER_MEAN      = 1,
  LMX249x_FRAC_DITHER_STRONG    = 2,
  LMX249x_FRAC_DITHER_DISABLED  = 3
} LMX249x_FracDither_e;

/*!
 * #35..#39 - MUX functions for pins
 */
typedef enum
{
  LMX249x_MUX_GND                     =  0,
  LMX249x_MUX_IN_TRIG1                =  1,
  LMX249x_MUX_IN_TRIG2                =  2,
  LMX249x_MUX_IN_MOD                  =  3,
  LMX249x_MUX_OUT_TRIG1_AFTER_SYNC    =  4,
  LMX249x_MUX_OUT_TRIG2_AFTER_SYNC    =  5,
  LMX249x_MUX_OUT_MOD_AFTER_SYNC      =  6,
  LMX249x_MUX_OUT_READ_BACK           =  7,
  LMX249x_MUX_OUT_CMP0                =  8,
  LMX249x_MUX_OUT_CMP1                =  9,
  LMX249x_MUX_OUT_LD                  = 10,
  LMX249x_MUX_OUT_DLD                 = 11,
  LMX249x_MUX_OUT_CPMON_GOOD          = 12,
  LMX249x_MUX_OUT_CPMON_TOO_HIGH      = 13,
  LMX249x_MUX_OUT_CPMON_TOO_LOW       = 14,
  LMX249x_MUX_OUT_RAMP_LIMIT_EXCEEDED = 15,
  LMX249x_MUX_OUT_R_DIVIDE_BY_2       = 16,
  LMX249x_MUX_OUT_R_DIVIDE_BY_4       = 17,
  LMX249x_MUX_OUT_N_DIVIDE_BY_2       = 18,
  LMX249x_MUX_OUT_N_DIVIDE_BY_4       = 19,
  LMX249x_MUX_RESERVED                = 20,
  LMX249x_MUX_OUT_CMP0RAMP            = 22,
  LMX249x_MUX_OUT_CMP1RAMP            = 23,
  LMX249x_MUX_OUT_FASTLOCK            = 28,
  LMX249x_MUX_OUT_CPG_FROM_RAMP       = 29,
  LMX249x_MUX_OUT_FLAG0_FROM_RAMP     = 30,
  LMX249x_MUX_OUT_FLAG1_FROM_RAMP     = 31,
  LMX249x_MUX_OUT_TRIGA               = 32,
  LMX249x_MUX_OUT_TRIGB               = 33,
  LMX249x_MUX_OUT_TRIGC               = 34,
  LMX249x_MUX_OUT_R_DIVIDE            = 35,
  LMX249x_MUX_OUT_CPUP                = 36,
  LMX249x_MUX_OUT_CPDN                = 37,
  LMX249x_MUX_OUT_RAMP_CNT_FINISHED   = 38
} LMX249x_MUX_e;

/*!
 * pin function of pll input/output pins
 */
typedef enum
{
  LMX249x_PIN_FCN_TRISTATE                = 0,
  LMX249x_PIN_FCN_OPEN_DRAIN_OUT          = 1,
  LMX249x_PIN_FCN_PULLUPDN_OUT            = 2,
  LMX249x_PIN_FCN_GND                     = 4,
  LMX249x_PIN_FCN_INVERTED_OPEN_DRAIN_OUT = 5,
  LMX249x_PIN_FCN_INVERTED_PULUPDN_OUT    = 6,
  LMX249x_PIN_FCN_INPUT                   = 7
} LMX249x_PinFunction_e;

/*!
 * charge pump polarity
 */
typedef enum
{
  LMX249x_CPPOL_NEGATIVE  = 0,
  LMX249x_CPPOL_POSITIVE  = 1
} LMX249x_ChargePumpPolarity_e;

/*!
 * digital lock detect tolerance
 */
typedef enum
{
  LMX249x_DLD_TOL_1NS   = 0,
  LMX249x_DLD_TOL_1_7NS = 1,
  LMX249x_DLD_TOL_3NS   = 2,
  LMX249x_DLD_TOL_6NS   = 3,
  LMX249x_DLD_TOL_10NS  = 4,
  LMX249x_DLD_TOL_18NS  = 5
} LMX249x_DLDTolerance_e;

/*!
 * A type to identify the status of an operation.
 */
typedef enum
{
  LMX249x_POWER_DOWN  = 0x00,   /*!< Power down, ignore CE */
  LMX249x_POWER_UP    = 0x01,   /*!< Power up, ignore CE */
  LMX249x_POWER_CE    = 0x02    /*!< Power state controlled by by CE Pin */
} LMX249x_PowerState_e;

/*!
 * A type to identify the status of an operation.
 */
typedef enum
{
  LMX249x_OSCIN_DOUBLER_OFF = 0,    /*!< OSCin frequency is used as is */
  LMX249x_OSCIN_DOUBLER_ON  = 1     /*!< OSCin frequency is doubled */
} LMX249x_OscillatiorInputDoubler_e;

/*!
 * A type to identify the status of an operation.
 */
typedef enum
{
  LMX249x_OSCIN_SINGLE_ENDED  = 0,    /*!< OSCin Pin is connected single ended */
  LMX249x_OSCIN_DIFFERENTIAL  = 1     /*!< OSCIN Pin is connected to a differential signal */
} LMX249x_OscillatiorInputMode_e;

/*!
 * A type to identify the status of an operation.
 */
typedef enum
{
  LMX249x_CP_PULSE_860PS  = 1,    /*!< charge pump pulse width of 860ps (recommended) */
  LMX249x_CP_PULSE_1200PS = 2,    /*!< charge pump pulse width of 1200ps */
  LMX249x_CP_PULSE_1500PS = 3     /*!< charge pump pulse width of 1500ps */
} LMX249x_ChargePumpPulseWidth_e;

/*!
 * A type to identify the status of an operation.
 */
typedef enum
{
  LMX249x_CSR_DISABLED  = 0,    /*!< Disable Cycle Slip Reduction (recommended when ramp generator is used) */
  LMX249x_CSR_X2        = 1,    /*!< Reduce cycle slip by multiplying counters by 2 */
  LMX249x_CSR_X4        = 2     /*!< Reduce cycle slip by multiplying counters by 4 */
} LMX249x_CycleSlipReduce_e;

/*!
 * A type to identify the status of an operation.
 */
typedef enum
{
  LMX249x_ERROR_CODE_OK = 0,
  LMX249x_ERROR_CODE_INVALID_NUMBER_OF_RAMPS
} LMX249x_ErrorCode_e;

/**************************************************************************************************/
/* Data Structures                                                                                */
/**************************************************************************************************/

/*!
 * Typedef for the function pointer responsible for SPI data transmission via SPI protocol
 */

//typedef void (*sendSPIFunction)(const uint8_t* data_ptr, uint8_t num_of_bytes, void* peripheral_ptr);
typedef void (*sendSPIFunction)(uint8_t* TxDataPointer, uint32_t DataLen);

/*!
 * This struct contains parameters that are related to the hardware setup and is passed to the init function
 */
typedef struct
{
  double                            dReferenceFreq;         /*!< The frequency of the reference oscillator */
  uint16_t                          uReferenceDivider;      /*!< The Reference Frequency is divided by this factor */
  LMX249x_OscillatiorInputDoubler_e eReferenceDoubler;      /*!< If this is non zero, the reference frequency is doubled by clocking on falling and rising edge */
  LMX249x_OscillatiorInputMode_e    eOscInMode;             /*!< The way the Oscillator input is connected */
  uint32_t                          uExternalDivider;       /*!< An external divide factor inside the loop circuitry (used to translate target frequencies to the frequencies the PLL sees) */
  LMX249x_MUX_e                     eTrig1PinFunction;      /*!< The function routed to the TRIG1 Pin */
  LMX249x_PinFunction_e             eTrig1PinDriveMode;     /*!< The driver mode for the TRIG1 Pin */
  LMX249x_MUX_e                     eTrig2PinFunction;      /*!< The function routed to the TRIG2 Pin */
  LMX249x_PinFunction_e             eTrig2PinDriveMode;     /*!< The driver mode for the TRIG2 Pin */
  LMX249x_MUX_e                     eModPinFunction;        /*!< The function routed to the MOD Pin */
  LMX249x_PinFunction_e             eModPinDriveMode;       /*!< The driver mode for the MOD Pin */
  LMX249x_MUX_e                     eMUXoutPinFunction;     /*!< The function routed to the MUXout Pin */
  LMX249x_PinFunction_e             eMUXoutPinDriveMode;    /*!< The driver mode for the MUXout Pin */
  LMX249x_ChargePumpPolarity_e      eChargePumpPolarity;    /*!< Positive or Negative charge pump gain */
  uint8_t                           uChargePumpCurrent;     /*!< The charge pump current in 100uA for normal operation mode */
  uint8_t                           uChargePumpCurrentFS;   /*!< The charge pump current in 100uA for fast lock mode */
  uint8_t                           uChargePumpThresholdLo; /*!< The lower threshold for lock detection */
  uint8_t                           uChargePumpThresholdHi; /*!< The upper threshold for lock detection */
  LMX249x_ChargePumpPulseWidth_e    eChargePumpPulseWidth;  /*!< charge pump pulse width in ps */
  LMX249x_CycleSlipReduce_e         eCycleSlipReduction;    /*!< cycle slip related information */
  uint16_t                          uFastLockTimer;         /*!< The fast Lock timer is initialized with this value multiplied by 32. */
  uint8_t                           uLockDetectNumGoodEdge; /*!< In a lock detect time window at least this number of good edges must occur, until a lock is detected. */
  uint8_t                           uLockDetectNumBadEdge;  /*!< In a lock detect time window at most this number of bad edges must occur, until a lock is detected. */
  LMX249x_DLDTolerance_e            eLockDetectWindow;      /*!< Digital Lock detect edge window. */
} LMX249x_HardwareSetup_s;

/*!
 * This structure contains parameters for the Ramp generator that are not specific for a single section.
 */
typedef struct
{
  double                      dBaseFrequency;     /*!< Signed ramp increment */
  double					  dDeviationFrequency;/*!< Signed deviation frequency for FSK operation */
  double                      dMinFrequency;      /*!< Minimum frequency that can't be underrun by a ramp */
  double                      dMaxFrequency;      /*!< Maximum frequency that can't be overrun by a ramp */
  double                      dComp0Freq;         /*!< Frequency that is used by Comparator 0 to compare the ramp with. */
  double                      dComp1Freq;         /*!< Frequency that is used by Comparator 0 to compare the ramp with. */
  LMX249x_RampTriggerSource_e eTriggerA;          /*!< Define the source for Trigger A */
  LMX249x_RampTriggerSource_e eTriggerB;          /*!< Define the source for Trigger B */
  LMX249x_RampTriggerSource_e eTriggerC;          /*!< Define the source for Trigger C */
  LMX249x_RampClock_e         eRampClock;         /*!< determines if internal or external clock for ramp timing is used */
  LMX249x_RampModulation_e    eModulation;        /*!< Kind of modulation */
  LMX249x_RampAutoOff_e       eAutoOff;           /*!< Defines, if the Ramp should be turned off after a certain amount of ramps */
  uint16_t                    uNumRamps;          /*!< Number of ramps to do before ramp is disabled (set to 0 for infinite number of ramps) */
  LMX249x_TriggerSelect_e     eRampCountTrigger;  /*!< Defines the increment trigger for the ramp counter */
  LMX249x_TriggerSelect_e     eDevTrigger;        /*!< Defines the FSK deviation trigger */
  LMX249x_FracOrder_e         eFracOrder;		  /*!< Defines the fractional order */
  LMX249x_FracDither_e        eDitherMode;		  /*!< Defines the fractional dithering */
} LMX249x_RampGlobal_s;

/*!
 * The PLL can define up to 8 ramp sections. This structure defines a single ramp section.
 */
typedef struct
{
  double                    dFreqShift;     /*!< Signed ramp increment */
  LMX249x_RampFastLock_e    eFastlock;      /*!< Fast lock enable*/
  double                    dTramp;         /*!< Ramp time in micro-seconds */
  LMX249x_RampFlag_e        eFlag;          /*!< General purpose FLAGS sent out of RAMP. */
  LMX249x_RampReset_e       eReset;         /*!< Forces a clear of the ramp accumulator. */
  LMX249x_RampNextTrig_e    eNextTrig;      /*!< Determines the event to trigger the next ramp. */
  uint8_t                   uNext;          /*!< The next RAMP to execute when the length counter times out. */
  LMX249x_RampComparators_e eComparators;   /*!< Determines which comparators should be used during ramping */
} LMX249x_RampSection_s;

/*!
 * An instance of this structure is passed to every call of LMX249x functions.
 */
typedef struct
{
  double dExternalDivideFactor;   /*!< This factor is applied to all frequencies coming in through
                                       the API. This is just for convenience, so the user can work
                                       with target frequencies, when the hardware setup has a
                                       built-in frequency divider. */
  double dPFDCycleTime;           /*!< This is the PFD cycle time, given frequencies are scaled
                                       with this factor to get a counter value. */
  uint8_t uReg58;                 /*!< This stores the content of register 58 to enable and disable
                                        ramping without reconfiguring the ramps. */
  sendSPIFunction sendSPI;        /*!< This function is used to send SPI data. */
//  void* pDataForSendSPI;          /*!< This function is passed to the send SPI data function. */
} LMX249x_Object_s;

/**************************************************************************************************/
/* Functions                                                                                      */
/**************************************************************************************************/

/*!
 * \brief initialize the LMX249x PLL
 */
/*void LMX249x_init(LMX249x_Object_s* pThis, const LMX249x_HardwareSetup_s* pSetup,
                  sendSPIFunction sendSPI, void* pDataForSendSPI);*/
void LMX249x_init(LMX249x_Object_s* pThis, const LMX249x_HardwareSetup_s* pSetup,
                  sendSPIFunction sendSPI);

/*!
 * This function sets the power state
 */
void LMX249x_setPowerState(LMX249x_Object_s* pThis, LMX249x_PowerState_e eState);

/*!
 * \brief This function sets the frequency for the PLL to oscillate
 */
LMX249x_ErrorCode_e LMX249x_setFrequency(LMX249x_Object_s* pThis, double dBaseFrequency,
                                         LMX249x_FracOrder_e eFracOrder,
                                         LMX249x_FracDither_e eDitherMode);

/*!
 * \brief This function defined up to 8 ramps.
 */
LMX249x_ErrorCode_e LMX249x_configureRamps(LMX249x_Object_s* pThis,
                                           const LMX249x_RampGlobal_s* pGlobalSettings,
                                           const LMX249x_RampSection_s* pRampSections,
                                           uint8_t uNumSections);

/*!
 * \brief This function enables or disables ramp mode of the PLL.
 */
void LMX249x_enableRamps(LMX249x_Object_s* pThis, uint8_t bEnable);

/*!
 * \brief This function calculates the fractional accurate bandwidth to avoid overflow of accumulator inside PLL.
 *
 * Without this correction, multiple ramps may have long delays.
 *
 */
double LMX249x_getRealFrequencyShift(LMX249x_Object_s* pThis, double FreqShift_MHz, uint32_t FreqShift_time_us);

/*!
 * \brief This function reads out the PLL registers.
 */
void LMX249x_read_register(uint16_t address, uint8_t* out_data_ptr, uint8_t num_of_bytes);

#endif /* DRIVER_PLL_LMX249x_H_ */
