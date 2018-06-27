/*
 * Sampler.h
 *
 *  Created on: 27 Jun 2018
 *      Author: 11238639
 */

#ifndef SOURCES_SAMPLER_H_
#define SOURCES_SAMPLER_H_

#include <math.h>

#include "OS.h"
#include "PIT.h"
#include "types.h"
#include "AlarmMonitor.h"


bool Sampler_Init(const uint32_t moduleClk);

void AnalyzerThread(void* pData);

void PassSampleThread(void* pData);

#endif /* SOURCES_SAMPLER_H_ */
