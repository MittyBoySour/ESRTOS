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
const uint8_t ANALYZER_THREAD_PRIORITIES[NB_SAMPLER_CHANNELS] = {5, 6, 7};

// ----------------------------------------
// Static shared thread data
static TFrequencyThreadData FrequencyThreadData;
static TPassSampleData PassSampleData;
static TPITData PITData;
static TPITChannelData PITChannelData[NB_ANALOG_CHANNELS];
static TAnalyzerThreadData AnalyzerThreadData[NB_SAMPLER_CHANNELS];
static TAlarmMonitoringData AlarmMonitoringData[NB_ANALOG_CHANNELS];
static TSampleManagerData SamplerManagerData;

// ----------------------------------------


static void InitSharedThreadData()
{
  // AnalyzerThreadData
  for (uint8_t channelNb = 0; channelNb < NB_ANALOG_CHANNELS; channelNb++)
  {
    AnalyzerThreadData[channelNb].SamplerFullSemaphore = OS_SemaphoreCreate(0);
    AnalyzerThreadData[channelNb].FrequencyTrackerSemaphore = OS_SemaphoreCreate(0);
    AnalyzerThreadData[channelNb].channelNb = channelNb;
    AnalyzerThreadData[channelNb].iterator = 0;
    AnalyzerThreadData[channelNb].analyzerReady = false;
    AnalyzerThreadData[channelNb].sampleArray = PassSampleData->PassSampleChannelData[channelNb]->sampleArray;
    AlarmMonitoringData[channelNb].PITTimerCompleteSemaphore;
    AlarmMonitoringData[channelNb].RMS;
    AlarmMonitoringData[channelNb].VoltageCalculatedSemaphore;
    AlarmMonitoringData[channelNb].alarmingHigh;
    AlarmMonitoringData[channelNb].alarmingLow;
    AnalyzerThreadData[channelNb].AlarmMonitoringData = AlarmMonitoringData[channelNb];
  }

  // PITData
  PITData.PITDataSampleTakenSemaphore = OS_SemaphoreCreate(0);
  PITData.PITFrequencySampleTakenSemaphore = OS_Semaphore(0);
  PITChannelData[NB_ANALOG_CHANNELS];
  for (uint8_t channelNb = 0; channelNb < NB_ANALOG_CHANNELS; channelNb++)
  {
    PITChannelData.PITAlarmSemaphore = OS_SemaphoreCreate(0);
    PITChannelData.analogSample = 0;
    PITChannelData.channelNb = channelNb;
  }
  PITData.PITChannelData = PITChannelData;

  //

}

static void InitModulesThread(void* pData)
{
  // shared thread data
  InitSharedThreadData();

  // Analog
  (void)Analog_Init(CPU_BUS_CLK_HZ);

  // Sampler
  Sampler_Init(CPU_BUS_CLK_HZ); // calls PIT_Init

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


  // Start multithreading - never returns!
  OS_Start();
}
