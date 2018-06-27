/*
 * AlarmMonitor.c
 *
 *  Created on: 27 Jun 2018
 *      Author: 11238639
 */

#include "PIT.h"
#include "Flash.h"

// ----------------------------------------
// Scaling Factors
// ----------------------------------------
static int32_t HighVoltageBound = (3 * 65536) / 65536;
static int32_t LowVoltageBound = (2 * 65536) / 65536;
static int32_t FiveVolts = (5 * 65536) / 65536;


static uint8_t Raises;
static uint8_t Lowers;
static bool InverseTimingModeEnabled;

/*! @brief Initialises the static members
 *
 */
bool AlarmMonitor_Init()
{
	InverseTimingModeEnabled = false;
	Raises = _FH(FLASH_DATA_START + 2);
	Lowers = _FH(FLASH_DATA_START + 4);
}

/*! @brief Checks that RMS is past high bound or not
 *
 *	@param RMS The RMS value to test
 *	@return bool - TRUE if outside of bounds, FALSE if not
 */
bool BeyondHighBound(const float RMS)
{
	if (RMS > HighVoltageBound)
		return true;
	else
		return false;
}

/*! @brief Checks that RMS is past low bound or not
 *
 *	@param RMS The RMS value to test
 *	@return bool - TRUE if outside of bounds, FALSE if not
 */
bool BeyondLowBound(const float RMS)
{
	if (RMS < LowVoltageBound)
		return true;
	else
		return false;
}

/*! @brief Gets the deviation past the out of bounds voltage
 *
 *  @param RMS pointer to the RMS value to test
 *	@return bool - TRUE if outside of bounds, FALSE if not
 *
 */
float GetDeviation(const float* RMS)
{
	if (*RMS > 3)
		return *RMS - 3;
	else
		return 2 - *RMS;
}

/*! @brief Monitors the RMS deviation and performs alarm management for a channel
 *
 *  @param sampleArray array of values that show at least one cycle.
 *  @param surroundingCrossValues array of values that show at least one cycle.
 *  @param RMS pointer holding the RMS to be updated.
 *
 *  @note Assumes that PIT has been setup correctly.
 */
void SetRMS(const uint32_t sampleArray[SAMPLES_ARRAY_SIZE], const uint8_t surroundingCrossValues[4], float* RMS)
{

	uint8_t cycleStart = surroundingCrossValues[1];
	uint8_t cycleEnd = surroundingCrossValues[2];

	float allSquared = 0;

	// Square root function
	for (uint8_t iterator = cycleStart; iterator < cycleEnd; iterator++)
	{
		allSquared += (sampleArray[iterator] * sampleArray[iterator]);
	}

	float meanSquared = allSquared * (1 / (cycleEnd - cycleStart));

	// store result in passed and return any error
	*RMS = sqrt(meanSquared);
}

/*! @brief Monitors the RMS deviation and performs alarm management for a channel
 *
 *  @param RMSData data pointer to check and affect RMS value and communication bools.
 *  @note Assumes that PIT has been setup correctly.
 */
void AlarmMonitoring(TAlarmMonitoringData* RMSData)
{
	//initial data
	float RMS = RMSData->RMS;
	uint32_t DeviationValue;
	// deviation = Deviation(RMS, &DeviationValue);
	if (RMSData->alarmingHigh)
	{
		if (!BeyondHighBound(RMS))
			// No longer alarming, cancel the alarm
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
			// No longer alarming, cancel the alarm
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
	// No alarming currently
	else
	{
		if (BeyondHighBound(RMS))
		{
			RMSData->alarmingHigh = true;
			Analog_Put(0, FiveVolts);
			PIT_Update(0, RMSData->channelNb);
		}
		else if (BeyondLowBound(RMS))
		{
			RMSData->alarmingLow = true;
			Analog_Put(1, FiveVolts);
			PIT_Update(0, RMSData->channelNb);
		}
	}

}

/*! @brief Performs a voltage raise, writing current Raises to flash.
 *
 */
void VoltageRaiseEvent()
{
	Analog_Put(3, FiveVolts);
	Analog_Put(0, 0);
	Flash_Write16(FLASH_DATA_START + 4, ++Raises);
}

/*! @brief Performs a voltage lower, writing current Lowers to flash.
 *
 */
void VoltageLowerEvent()
{
	Analog_Put(3, FiveVolts);
	Analog_Put(1, 0);
	Flash_Write16(FLASH_DATA_START + 4, ++Lowers);
}

/*! @brief Runs the thread to handle PIT alarm interrupts.
 *
 *  @param pData Alarm control thread data.
 *  @note Assumes that PIT has been setup correctly.
 */
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

		// perform one last check to see whether it has returned to safe limits
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

void AlarmsetTimingMode(const bool inverse)
{
	InverseTimingModeEnabled = inverse; // if !inverse --> in definite timing mode;
}


