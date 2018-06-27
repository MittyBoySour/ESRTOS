/*
 * PIT.h
 *
 *  Created on: 27 Jun 2018
 *      Author: 11238639
 */

#ifndef SOURCES_PIT_H_
#define SOURCES_PIT_H_

#include "types.h"

/*! @brief Sets up the PIT before first use.
 *
 *  Enables the PIT and freezes the timer when debugging.
 *  @param moduleClk The module clock rate in Hz.
 *  @param userFunction is a pointer to a user callback function.
 *  @param userArguments is a pointer to the user arguments to use with the user callback function.
 *  @return bool - TRUE if the PIT was successfully initialized.
 *  @note Assumes that moduleClk has a period which can be expressed as an integral number of nanoseconds.
 */
bool PIT_Init(const uint32_t moduleClk, TPITData* pITData);

/*! @brief Sets the value of the desired period of the PIT.
 *
 *  @param deviation The voltage deviation out of bounds
 *  @param channelNb The channel that alarmed
 *
 *  @note Is only used in Inverse mode
 */
void PIT_Update(float deviation, uint8_t channelNb);

/*! @brief Disables the specified PIT timer.
 *
 *  @param channelNb The channel to be disabled
 *
 *  @note called externally
 */
void PIT_Disable(uint8_t channelNb);

/*! @brief Interrupt service routine for the PIT.
 *
 *  The periodic interrupt timer has timed out.
 *  Alarm for channel 0 has been triggered.
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) PIT1_ISR(void);

/*! @brief Interrupt service routine for the PIT.
 *
 *  The periodic interrupt timer has timed out.
 *  Alarm for channel 1 has been triggered.
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) PIT2_ISR(void);

/*! @brief Interrupt service routine for the PIT.
 *
 *  The periodic interrupt timer has timed out.
 *  Alarm for channel 2 has been triggered.
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) PIT3_ISR(void);

/*! @brief Interrupt service routine for the PIT.
 *
 *  The periodic interrupt timer has timed out.
 *  Samples will be taken for the channels and semaphores signalled
 *  @note Assumes the PIT has been initialized.
 */
void __attribute__ ((interrupt)) PIT0_ISR(void);

#endif /* SOURCES_PIT_H_ */
