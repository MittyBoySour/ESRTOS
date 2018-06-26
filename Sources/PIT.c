#include "types.h"
#include "MK70F12.h"
#include "PIT.h"

#include "analog.h"
#include "OS.h"

static TPITRequirements PITData;

/*! @brief Sets up the PIT before first use.
 *
 *  Enables the PIT and freezes the timer when debugging.
 *  @param moduleClk The module clock rate in Hz.
 *  @param userFunction is a pointer to a user callback function.
 *  @param userArguments is a pointer to the user arguments to use with the user callback function.
 *  @return bool - TRUE if the PIT was successfully initialized.
 *  @note Assumes that moduleClk has a period which can be expressed as an integral number of nanoseconds.
 */
bool PIT_Init() {

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
void PIT_Set(const uint32_t sampleTickPeriod, const bool restart, const bool alarm, void* pData) {

  #define samplerData ((TSamplerData*)pData)

  if (alarm)
  {
    // save different semaphore
    // use alarm semaphore
  }

  TPITRequirements PITRequirements =
  {
    .FrequencyAnalyzerSemaphore = samplerData->FrequencyAnalyzerSemaphore,
    .channelNb = samplerData->channelNb,
    .analogSample = SamplerData->analogSample
  }

  PITData = PITRequirements;

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
void __attribute__ ((interrupt)) PIT_ISR(void) {

  OS_ISREnter();

  PIT_TFLG0 = PIT_TFLG_TIF_MASK; // check which flag
  // check for both alarm semaphores active & all channels
  // check for these by which flag is set and semaphore passed in for that PIT no.

  // PITData
  // check if PIT0 first
  for (/* current channels */) // this will only be channel 0 for freq analysis, 0,1,2 for sampling
  {

    // take sample & raise semaphore
    Analog_Get(PITData->channelNb, &PITData->analogSample); // check this works
    // TODO: may still need to analog put?
    OS_SemaphoreSignal(PITData->TimerCompleteSemaphore);
  }

  // if other PITs have flagged, then raise alarm for appropriate PIT no.

  OS_ISRExit();

}
