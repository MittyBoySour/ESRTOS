/*
 * Sampler.h
 *
 *  Created on: 27 Jun 2018
 *      Author: 11238639
 */

#ifndef SOURCES_SAMPLER_H_
#define SOURCES_SAMPLER_H_

#include "types.h"

bool Sampler_Init(const uint32_t moduleClk);

void AlarmControlThread(void* pData);

void AnalyzerThread(void* pData);

void PassSampleThread(void* pData);

void setTimingMode(const bool inverse);

#endif /* SOURCES_SAMPLER_H_ */
