/**
    \file: LMX249x.c
    \author: Dennis, Thomas Finke
 	\brief: This file implements the functions to use Texas Instrument's PLL named LMX249x
*/

/* ===========================================================================
** Copyright (C) 2017-2018 Infineon Technologies AG
** All rights reserved.
** ===========================================================================
**
** ===========================================================================
** This document contains proprietary information of Infineon Technologies AG.
** Passing on and copying of this document, and communication of its contents
** is not permitted without Infineon's prior written authorization.
** ===========================================================================
*/

/**************************************************************************************************/
/* Includes                                                                                       */
/**************************************************************************************************/

#include <external_dev_drivers/PLL/include/LMX249x.h>

/**************************************************************************************************/
/* Macros                                                                                         */
/**************************************************************************************************/

/**
 * This Macro is an internal helper to calculate the correct register index when
 * writing a sequence of registers to the PLL. The register sequence is inverted
 * and shifted by 2 to reserve two bytes for the address at the beginning of the
 * data buffer.
 */
#define REG_IDX(BASE_REG, NUM_REGS, IDX)  ((NUM_REGS) - ((IDX)-(BASE_REG)) - 1 + 2)

/**************************************************************************************************/
/* Functions                                                                                      */
/**************************************************************************************************/

/* ----------------------------------------------------------------------------- LMX249x_init */
/*void LMX249x_init(LMX249x_Object_s* pThis, const LMX249x_HardwareSetup_s* pSetup,
                  sendSPIFunction sendSPI, void* pDataForSendSPI)*/
void LMX249x_init(LMX249x_Object_s* pThis, const LMX249x_HardwareSetup_s* pSetup,
                  sendSPIFunction sendSPI)
{
  uint8_t uSPIData[17];

  /* set Internal Parameters */
  /* ----------------------- */
  pThis->dExternalDivideFactor = 1.0 / pSetup->uExternalDivider; /* inverse RF-divider within the BGT chip */
  pThis->dPFDCycleTime = 1.0f / (pSetup->dReferenceFreq * (pSetup->eReferenceDoubler ? 2.0 : 1.0) / pSetup->uReferenceDivider );
  pThis->uReg58 = 0;
  pThis->sendSPI = sendSPI;
 // pThis->pDataForSendSPI = pDataForSendSPI;

  /* do a Power on Reset and set all PLL registers to the default value */
  /* ------------------------------------------------------------------ */
  uSPIData[0] = 0;
  uSPIData[1] = 2;
  uSPIData[2] = (1 << 2);
  //pThis->sendSPI(uSPIData, 3, pThis->pDataForSendSPI);
  pThis->sendSPI(uSPIData, 3);

  /* setup Routing and Drive for Pins TRIG1, TRIG2, MOD and MUXout */
  /* ------------------------------------------------------------- */
  uSPIData[REG_IDX(25, 15, 35)] = 0x41                                        | /* these bits must be set */
                                  ((pSetup->eTrig1PinFunction   >> 2) & 0x08) | /* TRIG1_MUX[5] shifted to bit 3 */
                                  ((pSetup->eTrig2PinFunction   >> 1) & 0x10) | /* TRIG2_MUX[5] shifted to bit 4 */
                                  ((pSetup->eMUXoutPinFunction  >> 0) & 0x20) | /* MUXout_MUX[5] stays at bit 5 */
                                  ((pSetup->eModPinFunction     << 2) & 0x80);  /* MOD_MUX[5] shifted to at bit 7 */

  uSPIData[REG_IDX(25, 15, 36)] = ((pSetup->eTrig1PinFunction & 0x1F) << 3) |
                                    pSetup->eTrig1PinDriveMode;

  uSPIData[REG_IDX(25, 15, 37)] = ((pSetup->eTrig2PinFunction & 0x1F) << 3) |
                                    pSetup->eTrig2PinDriveMode;

  uSPIData[REG_IDX(25, 15, 38)] = ((pSetup->eModPinFunction & 0x1F) << 3) |
                                    pSetup->eModPinDriveMode;

  uSPIData[REG_IDX(25, 15, 39)] = ((pSetup->eMUXoutPinFunction & 0x1F) << 3) |
                                    pSetup->eMUXoutPinDriveMode;

  /* setup Reference Scaling */
  /* ----------------------- */
  uSPIData[REG_IDX(25, 15, 25)] = (pSetup->uReferenceDivider >> 0) & 0xFF;
  uSPIData[REG_IDX(25, 15, 26)] = (pSetup->uReferenceDivider >> 8) & 0xFF;
  uSPIData[REG_IDX(25, 15, 27)] =  pSetup->eReferenceDoubler |
                                  (pSetup->eOscInMode << 2);

  /* setup Charge Pump configuration */
  /* ------------------------------- */
  uSPIData[REG_IDX(25, 15, 27)] |= (pSetup->eChargePumpPulseWidth << 3);
  uSPIData[REG_IDX(25, 15, 28)] = ((pSetup->eChargePumpPolarity) << 5) |
                                   (pSetup->uChargePumpCurrent & 0x1F);
  uSPIData[REG_IDX(25, 15, 29)] =  (pSetup->uChargePumpCurrentFS & 0x1F);

  /* setup speed up settings */
  /* ----------------------- */
  uSPIData[REG_IDX(25, 15, 27)] |= (pSetup->eCycleSlipReduction << 5);
  uSPIData[REG_IDX(25, 15, 32)]  = (pSetup->uFastLockTimer & 0xFF);
  uSPIData[REG_IDX(25, 15, 29)] |= (pSetup->uFastLockTimer >> 3) & 0xE0;

  /* setup lock detection */
  /* -------------------- */
  uSPIData[REG_IDX(25, 15, 30)] =  pSetup->uChargePumpThresholdLo & 0x3F;
  uSPIData[REG_IDX(25, 15, 31)] =  pSetup->uChargePumpThresholdHi & 0x3F;

  uSPIData[REG_IDX(25, 15, 33)] =  pSetup->uLockDetectNumGoodEdge;
  uSPIData[REG_IDX(25, 15, 34)] = (pSetup->uLockDetectNumBadEdge & 0x1F) |
                                  (pSetup->eLockDetectWindow << 5);

  uSPIData[0] = 0;
  uSPIData[1] = 25 + 15 - 1;
 // pThis->sendSPI(uSPIData, 15 + 2, pThis->pDataForSendSPI);
  pThis->sendSPI(uSPIData, 15 + 2);
}

/* ----------------------------------------------------------------------------- LMX249x_setPowerState */
void LMX249x_setPowerState(LMX249x_Object_s* pThis, LMX249x_PowerState_e eState)
{
  uint8_t uSPIData[3];
  uSPIData[0] = 0;
  uSPIData[1] = 2;
  uSPIData[2] = eState;
  //pThis->sendSPI(uSPIData, 3, pThis->pDataForSendSPI);
  pThis->sendSPI(uSPIData, 3);
}

/* ----------------------------------------------------------------------------- LMX249x_setFrequency */
LMX249x_ErrorCode_e LMX249x_setFrequency(LMX249x_Object_s* pThis, double dBaseFrequency,
                                         LMX249x_FracOrder_e eFracOrder,
                                         LMX249x_FracDither_e eDitherMode)
{
// #pragma message "ToDo: Check parameter ranges."
//#pragma message "ToDo: There should be a way for the user to specify the fractional denominator."

  uint8_t uSPIData[11];     /* a data buffer that will be passed to the SPI interface */

  /* disable ramp (just in case a ramp is currently in progress) */
  /* ----------------------------------------------------------- */
  uSPIData[0] = 0;
  uSPIData[1] = 58;
  uSPIData[2] = 0;
  //pThis->sendSPI(uSPIData, 3, pThis->pDataForSendSPI);
  pThis->sendSPI(uSPIData, 3);

  /* setup frequency */
  /* --------------- */
  double dRelFrequency = dBaseFrequency * pThis->dExternalDivideFactor * pThis->dPFDCycleTime;  /* divider ratio between PLL-RF-in and PFD-frequency */

  uint32_t iFactorN = (int32_t)dRelFrequency;
  dRelFrequency -= iFactorN;
  int32_t iFracDenominator = 1<<24;
  int32_t iFracNumarator = (uint32_t)(dRelFrequency * iFracDenominator + 0.5);

  uSPIData[REG_IDX(16, 9, 16)] = (int8_t) ((iFactorN >>  0) & 0xFF);
  uSPIData[REG_IDX(16, 9, 17)] = (int8_t) ((iFactorN >>  8) & 0xFF);
  uSPIData[REG_IDX(16, 9, 18)] = (int8_t) ((iFactorN >> 16) & 0x03) |
                                           (eFracOrder << 4)        |
                                           (eDitherMode << 2);

  uSPIData[REG_IDX(16, 9, 19)] = (iFracNumarator >>  0) & 0xFF;
  uSPIData[REG_IDX(16, 9, 20)] = (iFracNumarator >>  8) & 0xFF;
  uSPIData[REG_IDX(16, 9, 21)] = (iFracNumarator >> 16) & 0xFF;

  iFracDenominator -= 1;
  uSPIData[REG_IDX(16, 9, 22)] = (iFracDenominator >>  0) & 0xFF;
  uSPIData[REG_IDX(16, 9, 23)] = (iFracDenominator >>  8) & 0xFF;
  uSPIData[REG_IDX(16, 9, 24)] = (iFracDenominator >> 16) & 0xFF;

  /* send register sequence to PLL */
  uSPIData[0] = 0;
  uSPIData[1] = 16 + 9 - 1;
  //pThis->sendSPI(uSPIData, 9 + 2, pThis->pDataForSendSPI);
  pThis->sendSPI(uSPIData, 9 + 2);

  return LMX249x_ERROR_CODE_OK;
}

/* ----------------------------------------------------------------------------- LMX249x_configureRamps */
LMX249x_ErrorCode_e LMX249x_configureRamps(LMX249x_Object_s* pThis,
                                           const LMX249x_RampGlobal_s* pGlobalSettings,
                                           const LMX249x_RampSection_s* pRampSections,
                                           uint8_t uNumSections)
{
// #pragma message "ToDo: Check parameter ranges."

  /* check if the number of range is in a valid range */
  if ((uNumSections < 1) || (uNumSections > 8))
    return LMX249x_ERROR_CODE_INVALID_NUMBER_OF_RAMPS;

  uint8_t uSPIData[29];     /* a data buffer that will be passed to the SPI interface */

  /* setup some constants */
  /* -------------------- */
  const double dFrequencyToNFactor = pThis->dExternalDivideFactor * pThis->dPFDCycleTime;
  const double dFracDenominator = (double)(1<<24);

  /* disable ramp (just in case a ramp is currently in progress) */
  /* ----------------------------------------------------------- */
  uSPIData[0] = 0;
  uSPIData[1] = 58;
  uSPIData[2] = 0;
  //pThis->sendSPI(uSPIData, 3, pThis->pDataForSendSPI);
  pThis->sendSPI(uSPIData, 3);

  /* setup up the ramp sections */
  /* -------------------------- */
  uint8_t uComparator0Enable = 0;
  uint8_t uComparator1Enable = 0;

  int8_t uIdx;
  for (uIdx = 0; uIdx < uNumSections; ++uIdx)
  {
    /* setup register buffer */
    uint16_t uBaseRegister = (86 + uIdx * 7);
    const uint8_t uNumRegs = 7;

    const LMX249x_RampSection_s* pThisSection = &pRampSections[uIdx];

    /* convert given ramp parameters to counter values */
    uint32_t uRampLength =  (int32_t) ((pThisSection->dTramp) / pThis->dPFDCycleTime);
    uint8_t uDelayFlag = 0;
    if (uRampLength > 0xFFFF)
    {
      /* if the ramp is too long, divide ramp length by two and set the delay flag which doubles the ramp time */
      uRampLength >>= 1;
      uDelayFlag = 0x80;
    }
    /* if the transition to the next sections is triggered by the length of this section, the length must be
     * at least 1, otherwise the counter seems to wrap around and the section will be longer than expected.
     */
    if ((uRampLength == 0) && (pThisSection->eNextTrig == LMX249x_RAMP_NEXT_TRIG_RAMPX_LEN))
      uRampLength = 1;

    int32_t uCounterInc =  (int32_t) (pThisSection->dFreqShift * dFrequencyToNFactor * dFracDenominator / (double)uRampLength);

    /* set ramp_increment */
    uSPIData[REG_IDX(0, uNumRegs, 0)] = (uint8_t) ((uCounterInc >>  0) & 0xFF);
    uSPIData[REG_IDX(0, uNumRegs, 1)] = (uint8_t) ((uCounterInc >>  8) & 0xFF);
    uSPIData[REG_IDX(0, uNumRegs, 2)] = (uint8_t) ((uCounterInc >> 16) & 0xFF);

    /* set flags and increment */
    uSPIData[REG_IDX(0, uNumRegs, 3)] = uDelayFlag |
                                        (uint8_t) ((pThisSection->eFastlock) << 6) |
                                        (uint8_t) ((uCounterInc >> 24) & 0x3F);

    /* set ramp_length */
    uSPIData[REG_IDX(0, uNumRegs, 4)] = ((uRampLength >> 0) & 0xFF);
    uSPIData[REG_IDX(0, uNumRegs, 5)] = ((uRampLength >> 8) & 0xFF);

    /* set flags */
    uSPIData[REG_IDX(0, uNumRegs, 6)] = ((int8_t) ((pThisSection->uNext &0x07)  << 5)) |
                      ((int8_t)  (pThisSection->eNextTrig << 3)) |
                      ((int8_t)  (pThisSection->eReset    << 2)) |
                      ((int8_t)  (pThisSection->eFlag   << 0));

    /* set comparator enable bit */
    uint8_t uCompEnMask = 1 << uIdx;
    uComparator0Enable |= (pThisSection->eComparators & LMX249x_RAMP_USE_COMPARATOR_1) ? uCompEnMask : 0;
    uComparator1Enable |= (pThisSection->eComparators & LMX249x_RAMP_USE_COMPARATOR_1) ? uCompEnMask : 0;

    /* write the register data to the chip, write highest address (the one of the register written first) at the
     * beginning of the data buffer.
     */
    uBaseRegister += uNumRegs - 1;
    uSPIData[0] = (uint8_t)((uBaseRegister >> 8) & 0xFF);
    uSPIData[1] = (uint8_t)((uBaseRegister >> 0) & 0xFF);
//    pThis->sendSPI(uSPIData, uNumRegs + 2, pThis->pDataForSendSPI);
    pThis->sendSPI(uSPIData, uNumRegs + 2);
  }

  /* set comparator enable bits */
  uSPIData[REG_IDX(58, 27, 64)] = uComparator0Enable;
  uSPIData[REG_IDX(58, 27, 69)] = uComparator1Enable;

  /* setup base frequency */
  /* -------------------- */
// #pragma message "A value range check would be good here."
  double dRelFrequency = pGlobalSettings->dBaseFrequency * dFrequencyToNFactor;  /* divider ratio between PLL-RF-in and PFD-frequency */

  uint32_t iFactorN = (int32_t)dRelFrequency;
  dRelFrequency -= iFactorN;
  int32_t iFracNumarator = (uint32_t) (dRelFrequency * dFracDenominator);

  uSPIData[REG_IDX(16, 9, 16)] = (int8_t) ((iFactorN >>  0) & 0xFF);
  uSPIData[REG_IDX(16, 9, 17)] = (int8_t) ((iFactorN >>  8) & 0xFF);

  uSPIData[REG_IDX(16, 9, 18)] = (int8_t) (((iFactorN >> 16) & 0x03) | (pGlobalSettings->eFracOrder << 4) | (pGlobalSettings->eDitherMode << 2));

  uSPIData[REG_IDX(16, 9, 19)] = (iFracNumarator >>  0) & 0xFF;
  uSPIData[REG_IDX(16, 9, 20)] = (iFracNumarator >>  8) & 0xFF;
  uSPIData[REG_IDX(16, 9, 21)] = (iFracNumarator >> 16) & 0xFF;

  /* fractional denominator is always 0xFFFFFF when ramp generator is active */
  uSPIData[REG_IDX(16, 9, 22)] = 0xFF;
  uSPIData[REG_IDX(16, 9, 23)] = 0xFF;
  uSPIData[REG_IDX(16, 9, 24)] = 0xFF;

  /* send register sequence to PLL */
  uSPIData[0] = 0;
  uSPIData[1] = 16 + 9 - 1;
//  pThis->sendSPI(uSPIData, 9 + 2, pThis->pDataForSendSPI);
  pThis->sendSPI(uSPIData, 9 + 2);

  /* setup ramp range and comparator values */
  /* -------------------------------------- */
  /* The formula in the LMX249x data sheet for the following settings is very misleading. The used formular was
   * found by debugging. The essence here is to specify the limits as the difference (limFreq - baseFreq), while
   * limFreq is the N factor with fractional part, but basFreq must be the integer N factor (no rounding, just truncationg).
   */
  int64_t iRampComp0     = (int64_t) ((pGlobalSettings->dComp0Freq    * dFrequencyToNFactor - iFactorN) * dFracDenominator);
  int64_t iRampComp1     = (int64_t) ((pGlobalSettings->dComp1Freq    * dFrequencyToNFactor - iFactorN) * dFracDenominator);
  int64_t iRampLimitLow  = (int64_t) ((pGlobalSettings->dMinFrequency * dFrequencyToNFactor - iFactorN) * dFracDenominator);
  int64_t iRampLimitHigh = (int64_t) ((pGlobalSettings->dMaxFrequency * dFrequencyToNFactor - iFactorN) * dFracDenominator);

  uSPIData[REG_IDX(58, 27, 60)] = (iRampComp0 >>  0) & 0xFF;
  uSPIData[REG_IDX(58, 27, 61)] = (iRampComp0 >>  8) & 0xFF;
  uSPIData[REG_IDX(58, 27, 62)] = (iRampComp0 >> 16) & 0xFF;
  uSPIData[REG_IDX(58, 27, 63)] = (iRampComp0 >> 24) & 0xFF;

  uSPIData[REG_IDX(58, 27, 65)] = (iRampComp1 >>  0) & 0xFF;
  uSPIData[REG_IDX(58, 27, 66)] = (iRampComp1 >>  8) & 0xFF;
  uSPIData[REG_IDX(58, 27, 67)] = (iRampComp1 >> 16) & 0xFF;
  uSPIData[REG_IDX(58, 27, 68)] = (iRampComp1 >> 24) & 0xFF;

  uSPIData[REG_IDX(58, 27, 75)] = (iRampLimitLow >>  0) & 0xFF;
  uSPIData[REG_IDX(58, 27, 76)] = (iRampLimitLow >>  8) & 0xFF;
  uSPIData[REG_IDX(58, 27, 77)] = (iRampLimitLow >> 16) & 0xFF;
  uSPIData[REG_IDX(58, 27, 78)] = (iRampLimitLow >> 24) & 0xFF;

  uSPIData[REG_IDX(58, 27, 79)] = (iRampLimitHigh >>  0) & 0xFF;
  uSPIData[REG_IDX(58, 27, 80)] = (iRampLimitHigh >>  8) & 0xFF;
  uSPIData[REG_IDX(58, 27, 81)] = (iRampLimitHigh >> 16) & 0xFF;
  uSPIData[REG_IDX(58, 27, 82)] = (iRampLimitHigh >> 24) & 0xFF;

  uSPIData[REG_IDX(58, 27, 70)] = ((iRampComp0    & 0x100000000) ? 1 << 0 : 0) |
                                  ((iRampComp1    & 0x100000000) ? 1 << 1 : 0) |
                                  ((iRampLimitLow   & 0x100000000) ? 1 << 3 : 0) |
                                  ((iRampLimitHigh  & 0x100000000) ? 1 << 4 : 0);

  /* define FSK deviation */
  /* -------------------- */
  int64_t iFSKDev = (int64_t) ((pGlobalSettings->dDeviationFrequency - pGlobalSettings->dBaseFrequency)
		  	  	  	  	  	  * dFrequencyToNFactor * (1 << 24));

  uSPIData[REG_IDX(58, 27, 71)] = (iFSKDev >>  0) & 0xFF;
  uSPIData[REG_IDX(58, 27, 72)] = (iFSKDev >>  8) & 0xFF;
  uSPIData[REG_IDX(58, 27, 73)] = (iFSKDev >> 16) & 0xFF;
  uSPIData[REG_IDX(58, 27, 74)] = (iFSKDev >> 24) & 0xFF;

  uSPIData[REG_IDX(58, 27, 70)] |= ((iFSKDev    & 0x100000000) ? 1 << 2 : 0)
                                | (pGlobalSettings->eDevTrigger << 5);

  /* define Ramp Trigger Sources and Modulation type */
  /* ----------------------------------------------- */
// #pragma message "External clock and phase modulation not supported yet."
  pThis->uReg58 = (pGlobalSettings->eRampClock) /* 0 << 1) */  | /* internal or external clock */
                  (pGlobalSettings->eModulation) /* 0 << 2) */ | /* phase or frequency modulation */
                  (pGlobalSettings->eTriggerA << 4);          /* Trigger A source */
  uSPIData[REG_IDX(58, 27, 58)] = pThis->uReg58;

  uSPIData[REG_IDX(58, 27, 59)] = (pGlobalSettings->eTriggerB)   | /* Trigger B source */
                                  (pGlobalSettings->eTriggerC << 4);   /* Trigger C source */

  /* setup ramp counter */
  uSPIData[REG_IDX(58, 27, 83)] = (pGlobalSettings->uNumRamps & 0xFF);
  uSPIData[REG_IDX(58, 27, 84)] = ((pGlobalSettings->uNumRamps >> 8)& 0x1F)  |
                                  (pGlobalSettings->eAutoOff << 5)     |
                                  (pGlobalSettings->eRampCountTrigger << 6);

  /* send register sequence to PLL */
  uSPIData[0] = 0;
  uSPIData[1] = 58 + 27 - 1;
//  pThis->sendSPI(uSPIData, 27 + 2, pThis->pDataForSendSPI);
  pThis->sendSPI(uSPIData, 27 + 2);

  return LMX249x_ERROR_CODE_OK;
}

/* ----------------------------------------------------------------------------- LMX249x_enableRamps */
void LMX249x_enableRamps(LMX249x_Object_s* pThis, uint8_t bEnable)
{
  uint8_t uSPIData[29];
  uSPIData[0] = 0;              /* High address byte */
  uSPIData[1] = 58;             /* Low address byte */
  uSPIData[2] = pThis->uReg58;  /* Configuration of register 58 */

  /* enable ramps */
  if (bEnable != 0)
    uSPIData[2] |= 1;
  else
	  uSPIData[2] &= 0xFE;

//  pThis->sendSPI(uSPIData, 3, pThis->pDataForSendSPI);
  pThis->sendSPI(uSPIData, 3);
}

/* ----------------------------------------------------------------------------- LMX249x_getRealFrequencyShift */
double LMX249x_getRealFrequencyShift(LMX249x_Object_s* pThis, double FreqShift_MHz, uint32_t FreqShift_time_us)
{
	double RealFreqShift;
	const double FracDenominator = (double)(1<<24);
    uint32_t FreqShiftSteps;
    double FreqShiftPerStep;

    FreqShiftSteps =  (uint32_t) (FreqShift_time_us / pThis->dPFDCycleTime);

    FreqShiftPerStep = (FreqShift_MHz * pThis->dExternalDivideFactor) / FreqShiftSteps;

    FreqShiftPerStep = ((uint32_t)((FreqShiftPerStep * FracDenominator) * pThis->dPFDCycleTime) ) / FracDenominator / pThis->dPFDCycleTime;

    RealFreqShift = FreqShiftPerStep * FreqShiftSteps /  pThis->dExternalDivideFactor;

	return RealFreqShift;
}

