#include "types.h"
#include "MK70F12.h"
// #include "PIT.h"

#include "analog.h"
#include "OS.h"

static TPITData PITData;
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
bool PIT_Init(TPITData pITData) {

  PITData = pITData

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
    PIT_Enable(false);

    // Clear pending interrupts on the PIT (w1c)
    PIT_TFLG0 = PIT_TFLG_TIF_MASK;

  }

  // Setting PIT_LDVAL to period in nanoseconds converted to moduleClk ticks
  PIT_LDVAL0 = sampleTickPeriod;

  // Enable the timer and interrupts
  PIT_Enable(true);

}

void PIT_UpdateAlarm(uint8_t deviation, bool start, uint8_t channelNb)
{

  OS_TimeSet(0);
  uint32_t loadValue = PIT_LDVAL(channelNb); // 5 seconds
  uint32_t currentValue = 0;

  if (!start) {
    loadValue = PIT_LDVAL(channelNb);
    currentValue = PIT_LDVAL(channelNb);

  }



}

/*! @brief Enables or disables the PIT.
 *
 *  @param enable - TRUE if the PIT is to be enabled, FALSE if the PIT is to be disabled.
 */
void PIT_Enable(const bool enable) {

  if (enable) {
    // Enable timer and interrupts
    PIT_TCTRL0 |= (PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK);
  } else {
    // Disables timer and interrupts
    PIT_TCTRL0 &= ~(PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK);
  }

}

/*! @brief Interrupt service routine for the PIT.
 *
 *  The periodic interrupt timer has timed out.
 *  The user callback function will be called.
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) PIT1_ISR(void) {

  OS_ISREnter();

  PIT_TFLG1 = PIT_TFLG_TIF_MASK;

  OS_SemaphoreSignal(PITData.PITChannelData[0].PITAlarmSemaphore);

  OS_ISRExit();

}

void __attribute__ ((interrupt)) PIT2_ISR(void) {

  OS_ISREnter();

  PIT_TFLG1 = PIT_TFLG_TIF_MASK;

  OS_SemaphoreSignal(PITData.PITChannelData[1].PITAlarmSemaphore);

  OS_ISRExit();

}

void __attribute__ ((interrupt)) PIT3_ISR(void) {

  OS_ISREnter();

  PIT_TFLG1 = PIT_TFLG_TIF_MASK;

  OS_SemaphoreSignal(PITData.PITChannelData[2].PITAlarmSemaphore);

  OS_ISRExit();

}


void __attribute__ ((interrupt)) PIT0_ISR(void) {

  OS_ISREnter();

  PIT_TFLG0 = PIT_TFLG_TIF_MASK;

  if (FrequencyTracking)
  {

    Analog_Get(0, &PITData.PITChannelData[0].analogSample);
    OS_SemaphoreSignal(PITData.PITFrequencySampleTakenSemaphore);
  }
  else
  {
    for (uint8_t channelNb; channelNb < NB_SAMPLER_CHANNELS; channelNb++)
    {
      // take samples
      Analog_Get(PITData.PITChannelData[channelNb].channelNb, &PITData.PITChannelData[channelNb].analogSample);
    }
    // signal semaphore
    OS_SemaphoreSignal(PITData.PITDataSampleTakenSemaphore);
  }


  // if other PITs have flagged, then raise alarm for appropriate PIT no.

  OS_ISRExit();

}
