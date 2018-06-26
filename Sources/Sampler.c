/*
 * Sampler.c
 *
 *  Created on: 22 Jun 2018
 *      Author: 11238639
 */


#include "OS.h"

// ----------------------------------------
// Frequency / Sampling set up
// ----------------------------------------
#define DEFAULT_FREQUENCY 50
#define LFR 47.5 // Lowest frequency range
#define HFR 52.5 // Highest frequency range
#define SAMPLES_PER_CYCLE 16
#define SAMPLES_ARRAY_SIZE 32 // may need to make larger to ensure always 2 zeros or just use freq thread
#define SAMPLER_WINDOW_CROSSES 3

#define MICROSECOND 1000000
static uint32_t ModuleClock;

// if samples = null[size - 1], do nothing

// ----------------------------------------
// Thread set up
// ----------------------------------------
// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 100
#define NB_SAMPLER_CHANNELS 3

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE); /*!< The stack for the LED Init thread. */
static uint32_t SamplerThreadStacks[NB_SAMPLER_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

// ----------------------------------------
// Thread priorities
// 0 = highest priority
// ----------------------------------------
const uint8_t SAMPLER_THREAD_PRIORITIES[NB_SAMPLER_CHANNELS] = {1, 2, 3};

/*! @brief Data structure used to pass Analog configuration to a user thread
 *
 */
typedef struct SamplerThreadData
{
  OS_ECB* AlarmSemaphore;
  OS_ECB* StoreSampleSemaphore;
  OS_ECB* AnalyzeSamplesSemaphore;
  uint8_t channelNb;
  int32_t analogSample;
  typedef struct SamplerData
  {
    uint8_t iterator;
    bool samplerFull;
    uint8_t crossCount;
    int32_t analogSamples[SAMPLES_ARRAY_SIZE];
    uint8_t crossPositions[SAMPLE_WINDOW_SIZE];
  } TSamplerData;
} TSamplerThreadData; // TODO: WE WILL HAVE TO BREAK THIS UP

/*! @brief Analog thread configuration data
 *
 */

volatile extern static TPassSampleData PassSampleData;
volatile extern static TPITData PITData;
volatile extern static TAnalyzerThreadData AnalyzerThreadData[NB_SAMPLER_CHANNELS];

// Global data
static bool InverseTimingModeEnabled;
static float Frequency; // possibly needs to be float

Sampler_Init(const uint32_t moduleClk, TPITData PITData) // May have to be thread
{
  // Initial setup
  ModuleClock = moduleClk;
  InverseTimingModeEnabled = false;
  // PIT_Init
  PIT_Init(PITData);

  // Shared thread data
  // SamplePasser data
  PassSampleData.PITDataSampleTakenSemaphore = OS_SemaphoreCreate(0);
  TPassSampleChannelData PassSampleChannelData[NB_SAMPLER_CHANNELS];
  for (int i = 0; i < NB_SAMPLER_CHANNELS; i++)
  {
    PassSampleChannelData[i].SamplerFullSemaphore = OS_SemaphoreCreate(0);
    PassSampleChannelData[i].channelNb = i;
    PassSampleChannelData[i].iterator = 0;
  };

  // PIT data --> this may be used within other functions
  PITData = ConvertPassSampleDataToPITData(PassSampleData);
  // Analyzer data
  for (int channelNb = 0; channelNb < NB_SAMPLER_CHANNELS; channelNb++)
  {
    AnalyzerThreadData[channelNb] = ConvertPassSampleDataToAnalyzerThreadData(PassSampleData, channelNb);
  }
}

// SamplerThread --> runs threads and calls initial setup too
void SamplerManagerThread(void* pData)
{
  // convert freq to ticksPerSample
  uint32_t ticksPerSample = ModuleClock / (FrequencyTrackerData->matchedFrequency * SAMPLES_PER_CYCLE);
  // Pass the new set sampling freq to PIT --> restart: true, alarm: false
  PIT_Set(ticksPerSample, true, false, &PITData);

  // PassSampleThread --> 1 thread
  error = OS_ThreadCreate(PassSampleThread,
                          &PassSampleData,
                          &InitModulesThreadStack[THREAD_STACK_SIZE - 1],
                          6); // pass sample thread priority --> need PIT_ISR at 7 possibly

  // Analyzer thread --> 3 threads
  for (int threadNb = 0; threadNb < NB_SAMPLER_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(AnalyzerThread,
                            &AnalyzerThreadData[threadNb],
                            &SamplerThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            SAMPLER_THREAD_PRIORITIES[threadNb]); // analyzer thread priority
  }

}

// no need to be a thread
void AlarmMonitoring(TAlarmMonitoringData RMSData)
{
  //initial data
  #define RMSData ((TAlarmMonitoringData*)pData)
  OS_SemaphoreWait(RMSData->VoltageCalculatedSemaphore, 0);

  uint32_t RMS = RMSData->RMS;
  uint32_t DeviationValue;
  // deviation = Deviation(RMS, &DeviationValue);
  if (alarmingHigh)
  {
    if (!BeyondHighBound(RMS))
    // No longer alarming
    {
      alarmingHigh = false;
      PIT_DisableAlarm();
    }
    else
    {
      if (InverseTimingModeEnabled)
        PIT_Update();
    }
  }
  else if (alarmingLow)
  {
    if (!BeyondLowBound(RMS))
    // No longer alarming
    {
      alarmingLow = false;
      PIT_DisableAlarm();
    }
    else
    {
      if (InverseTimingModeEnabled)
        PIT_Update();
    }
  }

}

void AlarmControlThread(void* pData)
{
  // shared vars
  #define alarmControlThreadData ((TAlarmControlThreadData*)pData)

  OS_SemaphoreWait(alarmControlThreadData->PITDataSampleTakenSemaphore, 0);

  PIT_Disable();

  if (alarmControlThreadData->alarmingHigh)
  {
    if (BeyondHighBound(RMS))
    {
      VoltageRaiseEvent();
      alarmControlThreadData->alarmingHigh = false;
    }

  }
  else if (alarmControlThreadData->alarmingLow)
  {
    if (BeyondLowBound(RMS))
    {
      VoltageLowerEvent();
      alarmControlThreadData->alarmingLow = false;
    }
  }

  // any necessary resetting

}

// consider 3 threads for this one rather than one as it will be less messy
void AnalyzerThread(void* pData)
{
  #define analyzerThreadData ((TAnalyzerThreadData*)pData)

  // Alarm monitoring thread will have already been initialised with shared values

  // wait on samplesReady semaphore
  OS_SemaphoreWait(analyzerThreadData->SamplerFullSemaphore, 0);
  // start timer
  // OS_TimerSet(0);

  for (;;)
  {
    // get crosses
    uint8_t crossPositions[SAMPLER_WINDOW_CROSSES];
    uint8_t crossesCount;
    for (uint8_t iterator = 1; iterator < SAMPLES_ARRAY_SIZE; iterator++)
    {
      int32_t previous = analyzerThreadData->sampleArray[iterator-1];
      int32_t current = analyzerThreadData->sampleArray[iterator];
      if (polarityChange(previous, current))
      {
        crossPositions[crossesCount++] = iterator;
        if (crossesCount == 3)
          break;
      }
    }
    // if window size is incorrect
    if (!(crossesCount == 3 && (crossPositions[2] - crossPositions[0]) >= 15))
    {
      // while this is a long process, there is no point servicing any interrupts until frequency is matched
      OS_DisableInterrupts();
      PIT_Disable();
      FrequencyMatcher();
      OS_EnableInterrupts();
    }
    //
    else
    {
      GetRMS(analyzerThreadData->sampleArray);
      AlarmMonitoring();
    }
    analyzerThreadData->fillNewSamples = true;

    // check voltage limits
      // signal new voltage calculated

    // send bool readyForMoreSamples

  }
}

// PIT_Update will be where PITs can be turned off

// TODO: check whether we need to use floats
void FrequencyTracker(OS_ECB* TrackingSemaphore, int32_t* analogSample)
{

  uint8_t maxSampleSpeed = HFR * SAMPLES_PER_CYCLE; // samples per second
  uint32_t ticksPerSample = ModuleClock / maxSampleSpeed;

  // continue waiting on and taking samples until 3 crossings found
  uint8_t crossCount;
  bool inWindow = false;
  int32_t surroundingCrossValues[4];
  int8_t sampleCount = 0;
  // set PIT
  PIT_Set(ticksPerSample, true, false, true); // period, restart, alarm, frequencyTracking, data (ptrStorage, chan, sem)

  // wait on first value
  OS_SemaphoreWait(TrackingSemaphore, 0); // maybe drop out if an error
  // place first sample into previous for comparison
  int32_t previousValue = analogSample;
  int32_t currentValue;
  while (crossCount < SAMPLER_WINDOW_CROSSES) // 3
  {
    OS_SemaphoreWait(TrackingSemaphore, 0);
    currentValue = analogSample;
    // check for crossing and update crossing values and count
    if (PolarityChange(previousValue, currentValue))
    {
      crossCount++;
      if (crossCount == 1)
      {
        surroundingCrossValues[0] = previousValue;
        surroundingCrossValues[1] = currentValue;
        inWindow = true;
      }
      else if (crossCount == 3)
      {
        surroundingCrossValues[2] = previousValue;
        surroundingCrossValues[3] = currentValue;
        // could also terminate loop before last sampleCount increment - fullSampleCount
      }
    }

    previousValue = currentValue;
    if (inWindow)
      sampleCount++;
  }
  // calculate and interpolate freq (linear to start with)
  uint32_t waveStart = LinearlyInterpolate(ticksPerSample, surroundingCrossValues[1], surroundingCrossValues[0]);
  uint32_t waveEnd = LinearlyInterpolate(ticksPerSample, surroundingCrossValues[2], surroundingCrossValues[3]);

  // store freq into passed freq param
  uint32_t newSamplePeriod = waveStart + ((sampleCount - 1) * ticksPerSample) + waveEnd;

  PIT_Set(newSamplePeriod);
}

bool PolarityChange(int32_t firstValue, int32_t secondValue)
{
  bool firstStatePos = firstValue > 0;
  bool secondStatePos = secondValue > 0;
  return (firstStatePos != secondStatePos);
}

uint32_t LinearlyInterpolate(const uint32_t ticksPerSample, const int32_t inCycleValue, const int32_t outOfCycleValue)
{
  int32_t sampleAmpChange = abs(inCycleValue) + abs(outOfCycleValue);
  int32_t inCycleAmpChange = abs(inCycleValue - 0);
  int32_t partialSample = ticksPerSample * (inCycleAmpChange / sampleAmpChange);
  return partialSample;
}

void PassSampleThread(void* pData)
{

  #define passSampleData ((TPassSampleData*)pData)
  uint8_t fillCounter = 0;

  for (;;)
  {
    // wait on TimerCompleteSemaphore
    OS_SemaphoreWait(passSampleData->PITDataSampleTakenSemaphore, 0);
    // store pointer into array asap
    OS_DisableInterrupts();
    for (int channelNb = 0; channelNb < NB_SAMPLER_CHANNELS; channelNb++)
    {
      if (passSampleData->PassSampleChannelData[channelNb].analyzerReady)
      {
        passSampleData->PassSampleChannelData[channelNb]->sampleArray[fillCounter]
                                                                      = passSampleData->PassSampleChannelData[channelNb]->analogSample;
      }
    }
    OS_EnableInterrupts();
    for (int channelNb = 0; channelNb < NB_SAMPLER_CHANNELS; channelNb++)
    {
      if (passSampleData->PassSampleChannelData[channelNb].fillNewSamples)
      {
        fillCounter++;
        if (fillCounter == SAMPLES_ARRAY_SIZE)
        {
          passSampleData->PassSampleChannelData[channelNb].fillNewSamples = false;
          fillCounter = 0;
        }
      }
      if (passSampleData->PassSampleChannelData[channelNb].analyzerReady)
      {
        // start timer - use an array that stores multiple timers
        OS_TimerStart(0);
        OS_SemaphoreSignal(passSampleData->PassSampleChannelData[channelNb].SamplerFullSemaphore);
      }

    }

  }

  for (;;)
  {
    // wait on TimerCompleteSemaphore
    OS_SemaphoreWait(passSampleData->PITDataSampleTakenSemaphore, 0);
    // store pointer into array asap
    OS_DisableInterrupts();
    for (int channelNb = 0; channelNb < NB_SAMPLER_CHANNELS; channelNb++)
    {
      // pass storage data
      // if last sample of array is full
        // raise AnalyzeSamplesSemaphore
      passSampleData->PassSampleChannelData[channelNb]->sampleArray[passSampleData->PassSampleChannelData[channelNb]->iterator++] = passSampleData->PassSampleChannelData[channelNb]->analogSample;
    }
    OS_EnableInterrupts();
    // perform array maintenance
    for (int channelNb = 0; channelNb < NB_SAMPLER_CHANNELS; channelNb++)
    {

      if (passSampleData->PassSampleChannelData[channelNb]->iterator == SAMPLES_ARRAY_SIZE)
        passSampleData->PassSampleChannelData[channelNb]->iterator = 0;

      if (passSampleData->PassSampleChannelData[channelNb]->fillNewSamples && passSampleData->PassSampleChannelData[channelNb]->sampleArray[SAMPLES_ARRAY_SIZE - 1] != NULL) // may need to iterate through
      {
        passSampleData->PassSampleChannelData[channelNb]->fillNewSamples = false;
      }
    }
    // if analyzerReady bool and samplerFull bool
      // raise the SamplesReady semaphore

    // raise semaphores all at once
    for (int channelNb = 0; channelNb < NB_SAMPLER_CHANNELS; channelNb++)
    {
      if (passSampleData->PassSampleChannelData[channelNb]->fillNewSamples)
      {
        OS_SemaphoreSignal(passSampleData->PassSampleChannelData[channelNb]->SamplerFullSemaphore);
      }
    }
  }
}

void setTimingMode(const bool inverse)
{
  InverseTimingModeEnabled = inverse; // if !inverse --> in definite timing mode;
}
