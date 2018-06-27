/*
 * main.c
 *
 *  Created on: 26 Jun 2018
 *      Author: 11238639
 */

#include "OS.h"
#include "types.h"
#include "analog.h"

// ----------------------------------------
// Thread set up
// ----------------------------------------
// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 100
#define NB_SAMPLER_CHANNELS 3

// ----------------------------------------
// Threads
/*
 * InitModulesThread
 * AnalyzerThread[3]
 * AlarmControlThread[3]
 * PassSamplesThread
 * FrequencyMatcherThread
 *
 * UARTThread[2]
 * FIFOThread[2]
 * PacketThread
 */
// ----------------------------------------

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE); /*!< The stack for the LED Init thread. */
static uint32_t SamplerThreadStacks[NB_SAMPLER_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

// ----------------------------------------
// Thread priorities
// 0 = highest priority
// ----------------------------------------
const uint8_t UART_THREAD_PRIORITY[2] = {1, 2};
const uint8_t ANALYZER_THREAD_PRIORITIES[NB_SAMPLER_CHANNELS] = {11, 12, 13};
const uint8_t ALARM_CONTROL_THREAD_PRIORITIES[NB_SAMPLER_CHANNELS] = {5, 6, 7};

// ----------------------------------------
// Static shared thread data
static TFrequencyThreadData FrequencyThreadData;
static TPassSampleData PassSampleData;
static TPassSampleChannelData PassSampleChannelData[NB_SAMPLER_CHANNELS];
static TPITData PITData;
static TPITChannelData PITChannelData[NB_ANALOG_CHANNELS];
static TAnalyzerThreadData AnalyzerThreadData[NB_SAMPLER_CHANNELS];
static TAlarmMonitoringData AlarmMonitoringData[NB_ANALOG_CHANNELS];
static TAlarmControlThreadData AlarmControlThreadData[NB_ANALOG_CHANNELS];

// ----------------------------------------


static void InitSharedThreadData()
{
  // PITData
  PITData.PITDataSampleTakenSemaphore = OS_SemaphoreCreate(0);
  PITData.PITFrequencySampleTakenSemaphore = OS_Semaphore(0);
  PITChannelData[NB_ANALOG_CHANNELS];
  for (uint8_t channelNb = 0; channelNb < NB_ANALOG_CHANNELS; channelNb++)
  {
    PITChannelData[channelNb].PITAlarmSemaphore = OS_SemaphoreCreate(0);
    PITChannelData[channelNb].analogSample = 0;
    PITChannelData[channelNb].channelNb = channelNb;
  }
  PITData.PITChannelData = PITChannelData;

  // AnalyzerThreadData
  for (uint8_t channelNb = 0; channelNb < NB_ANALOG_CHANNELS; channelNb++)
  {
    if (channelNb == 0)
    {
      AnalyzerThreadData[channelNb].FrequencyTrackerSemaphore = OS_SemaphoreCreate(0);
    }
    AnalyzerThreadData[channelNb].SamplerFullSemaphore = OS_SemaphoreCreate(0);
    AnalyzerThreadData[channelNb].channelNb = channelNb;
    AnalyzerThreadData[channelNb].iterator = 0;
    AnalyzerThreadData[channelNb].analyzerReady = false;
    AnalyzerThreadData[channelNb].fillNewSamples = true;
    AnalyzerThreadData[channelNb].sampleArray = 0;
    AlarmMonitoringData[channelNb].RMS = 0;
    AlarmMonitoringData[channelNb].channelNb = AnalyzerThreadData[channelNb].channelNb;
    AlarmMonitoringData[channelNb].alarmingHigh = false;
    AlarmMonitoringData[channelNb].alarmingLow = false;
    AnalyzerThreadData[channelNb].AlarmMonitoringData = AlarmMonitoringData[channelNb];
  }

  // AlarmControlThreadData
  for (uint8_t channelNb = 0; channelNb < NB_ANALOG_CHANNELS; channelNb++)
  {
    AlarmControlThreadData[channelNb].PITAlarmSemaphore = PITChannelData[channelNb].PITAlarmSemaphore;
    AlarmControlThreadData[channelNb].RMS = AlarmMonitoringData[channelNb].RMS;
    AlarmControlThreadData[channelNb].alarmingHigh = AlarmMonitoringData[channelNb].alarmingHigh;
    AlarmControlThreadData[channelNb].alarmingLow = AlarmMonitoringData[channelNb].alarmingLow;
    AlarmControlThreadData[channelNb].channelNb = AlarmMonitoringData[channelNb].channelNb;
  }

  // PassSampleData
  PassSampleData.PITDataSampleTakenSemaphore = PITData.PITDataSampleTakenSemaphore;
  for (uint8_t channelNb = 0; channelNb < NB_ANALOG_CHANNELS; channelNb++)
  {
    PassSampleChannelData[channelNb].SamplerFullSemaphore = AnalyzerThreadData[channelNb].SamplerFullSemaphore;
    PassSampleChannelData[channelNb].analogSample = PITChannelData[channelNb].analogSample;
    PassSampleChannelData[channelNb].analyzerReady = AnalyzerThreadData[channelNb].analyzerReady;
    PassSampleChannelData[channelNb].fillNewSamples = AnalyzerThreadData[channelNb].fillNewSamples;
    PassSampleChannelData[channelNb].channelNb = AnalyzerThreadData[channelNb].channelNb;
    PassSampleChannelData[channelNb].sampleArray = AnalyzerThreadData[channelNb].sampleArray;
  }

}

static void InitModulesThread(void* pData)
{
  // shared thread data
  InitSharedThreadData();

  // Analog
  (void)Analog_Init(CPU_BUS_CLK_HZ);

  // Sampler
  Sampler_Init(CPU_BUS_CLK_HZ);

  PIT_Init(&PITData);

  // We only do this once - therefore delete this thread
  OS_ThreadDelete(OS_PRIORITY_SELF);
}

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  OS_ERROR error;

  // Initialise low-level clocks etc using Processor Expert code
  PE_low_level_init();

  // Initialize the RTOS
  OS_Init(CPU_CORE_CLK_HZ, true);

  // Create module initialisation thread
  error = OS_ThreadCreate(InitModulesThread,
                          NULL,
                          &InitModulesThreadStack[THREAD_STACK_SIZE - 1],
                          0); // Highest priority

  error = OS_ThreadCreate(PassSampleThread,
                          &PassSampleData,
                          &InitModulesThreadStack[THREAD_STACK_SIZE - 1],
                          3);

  error = OS_ThreadCreate(FrequencyTrackerThread,
                          &FrequencyTrackerThreadData,
                          &InitModulesThreadStack[THREAD_STACK_SIZE - 1],
                          4);

  for (int threadNb = 0; threadNb < NB_SAMPLER_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(AnalyzerThread,
                            &AnalyzerThreadData[threadNb],
                            &SamplerThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            ANALYZER_THREAD_PRIORITIES[threadNb]);
  }

  for (int threadNb = 0; threadNb < NB_SAMPLER_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(AlarmControlThread,
                            &AlarmControlThreadData[threadNb],
                            &SamplerThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            ALARM_CONTROL_THREAD_PRIORITIES[threadNb]);
  }

  // Start multithreading - never returns!
  OS_Start();
}
