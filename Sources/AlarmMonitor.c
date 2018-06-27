/*
 * AlarmMonitor.c
 *
 *  Created on: 27 Jun 2018
 *      Author: 11238639
 */

#include "PIT.h"

// ----------------------------------------
// Scaling Factors
// ----------------------------------------
static int32_t HighVoltageBound = (3 * 65536) / 65536;
static int32_t LowVoltageBound = (2 * 65536) / 65536;
static int32_t FiveVolts = (5 * 65536) / 65536;


static bool InverseTimingModeEnabled;

bool AlarmMonitor_Init()
{
	InverseTimingModeEnabled = false;
}

bool BeyondHighBound(const float RMS)
{
	if (RMS > HighVoltageBound)
		return true;
	else
		return false;
}

bool BeyondLowBound(const float RMS)
{
	if (RMS < LowVoltageBound)
		return true;
	else
		return false;
}

float GetDeviation(const float* RMS)
{
	if (*RMS > 3)
		return *RMS - 3;
	else
		return 2 - *RMS;
}

void SetRMS(const uint32_t sampleArray[SAMPLES_ARRAY_SIZE], const uint8_t surroundingCrossValues[4], float* RMS)
{

  uint8_t cycleStart = surroundingCrossValues[1];
  uint8_t cycleEnd = surroundingCrossValues[2];

  float allSquared = 0;

  for (uint8_t iterator = cycleStart; iterator < cycleEnd; iterator++)
  {
	allSquared += (sampleArray[iterator] * sampleArray[iterator]);
  }

  float meanSquared = allSquared * (1 / (cycleEnd - cycleStart));

  // store result in passed and return any error
  *RMS = sqrt(meanSquared);
}

void AlarmMonitoring(TAlarmMonitoringData* RMSData)
{
  //initial data
  float RMS = RMSData->RMS;
  uint32_t DeviationValue;
  // deviation = Deviation(RMS, &DeviationValue);
  if (RMSData->alarmingHigh)
  {
    if (!BeyondHighBound(RMS))
    // No longer alarming
    {
      RMSData->alarmingHigh = false;
      PIT_Disable(RMSData->channelNb);
      Analog_Put(0, 0);
    }
    else
    {
      if (InverseTimingModeEnabled)
        PIT_Update(GetDeviation(&RMS), RMSData->channelNb);
    }
  }
  else if (RMSData->alarmingLow)
  {
    if (!BeyondLowBound(RMS))
    // No longer alarming
    {
      RMSData->alarmingLow = false;
      PIT_Disable(RMSData->channelNb);
      Analog_Put(1, 0);
    }
    else
    {
      if (InverseTimingModeEnabled)
        PIT_Update(GetDeviation(&RMS), RMSData->channelNb);
    }
  }
  else
  {
    if (BeyondHighBound(RMS))
    {
      RMSData->alarmingHigh = true;
      Analog_Put(0, FiveVolts);
    }
    else if (BeyondLowBound(RMS))
    {
	  RMSData->alarmingLow = true;
	  Analog_Put(1, FiveVolts);
    }
  }

}

void VoltageRaiseEvent()
{
	Analog_Put(3, FiveVolts);
	Analog_Put(0, 0);
}

void VoltageLowerEvent()
{
	Analog_Put(3, FiveVolts);
	Analog_Put(1, 0);
}

void AlarmControlThread(void* pData)
{
  // shared vars
  #define alarmControlThreadData ((TAlarmControlThreadData*)pData)

  // at end put local var = to either 5 or nothing
  float RMS = alarmControlThreadData->RMS;

  for (;;)
  {
	  OS_SemaphoreWait(alarmControlThreadData->PITAlarmSemaphore, 0);

	  PIT_Disable(alarmControlThreadData->channelNb);

	  if (alarmControlThreadData->alarmingHigh)
	  {
	    if (BeyondHighBound(RMS))
	    {
	      VoltageRaiseEvent();
	      alarmControlThreadData->alarmingHigh = false;
	    }

	  }
	  else if (alarmControlThreadData->alarmingLow)
	  {
	    if (BeyondLowBound(RMS))
	    {
	      VoltageLowerEvent();
	      alarmControlThreadData->alarmingLow = false;
	    }
	  }

  }


}

void setTimingMode(const bool inverse)
{
  InverseTimingModeEnabled = inverse; // if !inverse --> in definite timing mode;
}


