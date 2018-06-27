/*
 * AlarmMonitor.h
 *
 *  Created on: 27 Jun 2018
 *      Author: 11238639
 */

#ifndef SOURCES_ALARMMONITOR_H_
#define SOURCES_ALARMMONITOR_H_

#include "types.h"

bool AlarmMonitor_Init();

void AlarmMonitoring(TAlarmMonitoringData* RMSData);

void AlarmControlThread(void* pData);

void AlarmsetTimingMode(const bool inverse);

#endif /* SOURCES_ALARMMONITOR_H_ */
