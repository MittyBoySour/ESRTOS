#include "types.h"
#include "MK70F12.h"
// #include "PIT.h"

#include "analog.h"
#include "OS.h"
#include "PIT.h"

static TPITData* PITData;
static uint32_t ModuleClock;
static bool FrequencyTracking;
static uint32_t SampleTickPeriod;

/*! @brief Sets up the PIT before first use.
 *
 *  Enables the PIT and freezes the timer when debugging.
 *  @param moduleClk The module clock rate in Hz.
 *  @param userFunction is a pointer to a user callback function.
 *  @param userArguments is a pointer to the user arguments to use with the user callback function.
 *  @return bool - TRUE if the PIT was successfully initialized.
 *  @note Assumes that moduleClk has a period which can be expressed as an integral number of nanoseconds.
 */
bool PIT_Init(const uint32_t moduleClk, TPITData* pITData) {

  ModuleClock = moduleClk;
  PITData = pITData;

  // Enable clock gate to PIT module
  SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;

  // Enable timer
  PIT_MCR &= ~PIT_MCR_MDIS_MASK;

  // set timer to freeze when debugging
  /* PIT_MCR |= PIT_MCR_FRZ_MASK; */

  // Clear any pending interrupts on all PITs
  NVICICPR2 = (15 << 4);
  // Enable interrupts on all PITs
  NVICISER2 = (15 << 4);

  return true;
}

/*! @brief Enables or disables the PIT.
 *
 *  @param enable - TRUE if the PIT is to be enabled, FALSE if the PIT is to be disabled.
 */
void Enable(const bool enable, const uint8_t pitNb) {

  if (enable) {
    // Enable timer and interrupts
    PIT_TCTRL(pitNb) |= (PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK);
  } else {
    // Disables timer and interrupts
    PIT_TCTRL(pitNb) &= ~(PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK);
  }

}

/*! @brief Sets the value of the desired period of the PIT.
 *
 *  @param period The desired value of the timer period in nanoseconds.
 *  @param restart TRUE if the PIT is disabled, a new value set, and then enabled.
 *                 FALSE if the PIT will use the new value after a trigger event.
 *  @note The function will enable the timer and interrupts for the PIT.
 */
void PIT_Set(const uint32_t sampleTickPeriod, const bool restart, const bool frequencyTracking) {

  SampleTickPeriod = sampleTickPeriod;
  FrequencyTracking = frequencyTracking;

  if (restart) {
    // Disable the timer and interrupts first
    Enable(false, 0);

    // Clear pending interrupts on the PIT (w1c)
    PIT_TFLG0 = PIT_TFLG_TIF_MASK;

  }

  // Setting PIT_LDVAL to period in nanoseconds converted to moduleClk ticks
  PIT_LDVAL0 = sampleTickPeriod;

  // Enable the timer and interrupts
  Enable(true, 0);

}

/*! @brief Sets the value of the desired period of the PIT.
 *
 *  @param deviation The voltage deviation out of bounds
 *  @param channelNb The channel that alarmed
 *
 *  @note If no current value, defaults to 5, otherwise assumes inverse mode and updates the timer according to deviation
 *  0 is passed in if not in inverse mode for deviation, as it is not required
 */
void PIT_Update(float deviation, uint8_t channelNb)
{

  uint32_t loadValue = PIT_LDVAL(channelNb);
  uint32_t currentValue = PIT_CVAL(channelNb);

  if (loadValue == 0)
  {
	  loadValue = (5 * ModuleClock); // 5 seconds
	  currentValue = 0;
	  PIT_LDVAL(channelNb + 1) = loadValue;
  }
  else
  {
	  float deviationFactor = ((1 / 2) / deviation);
	  float timeRemainingFactor = ((loadValue - currentValue) / loadValue);

	  float newLoad = deviationFactor * loadValue * timeRemainingFactor;

	  // Disable the timer and interrupts first
	  Enable(false, (channelNb + 1));

	  // Clear pending interrupts on the PIT (w1c)
	  PIT_TFLG(channelNb + 1) = PIT_TFLG_TIF_MASK;

	  newLoad -= OS_TimeGet();

	  if (newLoad > 1)
	    // Setting PIT_LDVAL to period in nanoseconds converted to moduleClk ticks
	    PIT_LDVAL(channelNb + 1) = (uint32_t)newLoad;

  }

  // Enable the timer and interrupts
  Enable(true, (channelNb + 1));

}

/*! @brief Disables the specified PIT timer.
 *
 *  @param channelNb The channel to be disabled
 *  @note called externally
 */
void PIT_Disable(uint8_t channelNb)
{
  Enable(false, channelNb);
}

/*! @brief Interrupt service routine for the PIT.
 *
 *  The periodic interrupt timer has timed out.
 *  Alarm for channel 0 has been triggered.
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) PIT1_ISR(void) {

  OS_ISREnter();

  PIT_TFLG1 = PIT_TFLG_TIF_MASK;

  OS_SemaphoreSignal(PITData->PITChannelData[0]->PITAlarmSemaphore);

  OS_ISRExit();

}

/*! @brief Interrupt service routine for the PIT.
 *
 *  The periodic interrupt timer has timed out.
 *  Alarm for channel 1 has been triggered.
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) PIT2_ISR(void) {

  OS_ISREnter();

  PIT_TFLG1 = PIT_TFLG_TIF_MASK;

  OS_SemaphoreSignal(PITData->PITChannelData[1]->PITAlarmSemaphore);

  OS_ISRExit();

}

/*! @brief Interrupt service routine for the PIT.
 *
 *  The periodic interrupt timer has timed out.
 *  Alarm for channel 2 has been triggered.
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) PIT3_ISR(void) {

  OS_ISREnter();

  PIT_TFLG1 = PIT_TFLG_TIF_MASK;

  OS_SemaphoreSignal(PITData->PITChannelData[2]->PITAlarmSemaphore);

  OS_ISRExit();

}

/*! @brief Interrupt service routine for the PIT.
 *
 *  The periodic interrupt timer has timed out.
 *  Samples will be taken for the channels and semaphores signalled
 *  If in frequency tracking mode, will only take samples for channel 1, otherwise all 3
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) PIT0_ISR(void) {

  OS_ISREnter();

  PIT_TFLG0 = PIT_TFLG_TIF_MASK;

  if (FrequencyTracking)
  {
    Analog_Get(0, PITData->PITChannelData[0]->analogSample);
    OS_SemaphoreSignal(PITData->PITFrequencySampleTakenSemaphore);
  }
  else
  {
    for (uint8_t channelNb; channelNb < NB_SAMPLER_CHANNELS; channelNb++)
    {
      // take samples
      Analog_Get(PITData->PITChannelData[channelNb]->channelNb, PITData->PITChannelData[channelNb]->analogSample);
    }
    // signal semaphore
    OS_SemaphoreSignal(PITData->PITDataSampleTakenSemaphore);
  }

  OS_ISRExit();

}
