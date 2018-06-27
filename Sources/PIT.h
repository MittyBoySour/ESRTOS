/*
 * PIT.h
 *
 *  Created on: 27 Jun 2018
 *      Author: 11238639
 */

#ifndef SOURCES_PIT_H_
#define SOURCES_PIT_H_

#include "types.h"

bool PIT_Init(const uint32_t moduleClk, TPITData pITData);

void PIT_Update(float deviation, uint8_t channelNb);

void PIT_Disable(uint8_t channelNb);

void __attribute__ ((interrupt)) PIT1_ISR(void);

void __attribute__ ((interrupt)) PIT2_ISR(void);

void __attribute__ ((interrupt)) PIT3_ISR(void);

void __attribute__ ((interrupt)) PIT0_ISR(void);

#endif /* SOURCES_PIT_H_ */
