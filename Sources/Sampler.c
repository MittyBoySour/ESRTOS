/*
 * Sampler.c
 *
 *  Created on: 22 Jun 2018
 *      Author: 11238639
 */

#include "Sampler.h"
// ----------------------------------------
// Scaling Factors
// ----------------------------------------
static int32_t HighVoltageBound = (3 * 65536) / 65536;
static int32_t LowVoltageBound = (2 * 65536) / 65536;
static int32_t FiveVolts = (5 * 65536) / 65536;

// ----------------------------------------
// Frequency / Sampling set up
// ----------------------------------------
#define DEFAULT_FREQUENCY 50
#define LFR 47.5 // Lowest frequency range
#define HFR 52.5 // Highest frequency range
#define SAMPLES_PER_CYCLE 16
#define SAMPLES_ARRAY_SIZE 32
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


volatile extern TPassSampleData PassSampleData;
volatile extern TPITData PITData;
volatile extern TAnalyzerThreadData AnalyzerThreadData[NB_SAMPLER_CHANNELS];

// Global data
static float CurrentFrequency;

bool Sampler_Init(const uint32_t moduleClk)
{
  // Initial setup
  ModuleClock = moduleClk;
  CurrentFrequency = HFR;

}

bool PolarityChange(const int32_t firstValue, const int32_t secondValue)
{

  bool firstStatePos = firstValue >= 0;
  bool secondStatePos = secondValue >= 0;
  return (firstStatePos != secondStatePos);

}

uint32_t LinearlyInterpolate(const uint32_t ticksPerSample, const int32_t inCycleValue, const int32_t outOfCycleValue)
{

  int32_t sampleAmpChange = abs(inCycleValue) + abs(outOfCycleValue);
  int32_t inCycleAmpChange = abs(inCycleValue - 0);
  uint32_t partialSample = (uint32_t)(ticksPerSample * ((float)inCycleAmpChange / (float)sampleAmpChange));
  return partialSample;
}


void SetNewSamplePeriod(const uint32_t ticksPerSample, const uint32_t surroundingCrossValues[4], uint8_t sampleCount)
{
  // calculate and interpolate freq (linear to start with)
  uint32_t waveStart = LinearlyInterpolate(ticksPerSample, surroundingCrossValues[1], surroundingCrossValues[0]);
  uint32_t waveEnd = LinearlyInterpolate(ticksPerSample, surroundingCrossValues[2], surroundingCrossValues[3]);

  // store freq into passed freq param
  uint32_t newSamplePeriod = waveStart + ((sampleCount - 1) * ticksPerSample) + waveEnd;

  PIT_Set(newSamplePeriod, true, false);

}

void FrequencyTracker(OS_ECB* TrackingSemaphore, int32_t* analogSample)
{

  float maxSampleSpeed = HFR * SAMPLES_PER_CYCLE; // samples per second
  uint32_t ticksPerSample = (uint32_t)(ModuleClock / maxSampleSpeed);

  // internal thread data
  uint8_t crossCount = 0;
  bool inWindow = false;
  int32_t surroundingCrossValues[4];
  int8_t sampleCount = 0;

  // set PIT
  PIT_Set(ticksPerSample, true, true);

  // wait on first value from PIT
  OS_SemaphoreWait(TrackingSemaphore, 0);

  // place first sample into previous for comparison
  int32_t previousValue = *analogSample;
  int32_t currentValue;

  // look for crosses
  while (crossCount < SAMPLER_WINDOW_CROSSES) // 3
  {
    OS_SemaphoreWait(TrackingSemaphore, 0);
    currentValue = *analogSample;

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

  SetNewSamplePeriod(ticksPerSample, surroundingCrossValues, sampleCount);

  float periodInTicks = ticksPerSample * SAMPLES_PER_CYCLE;
  CurrentFrequency = ModuleClock / periodInTicks;
}


void AnalyzerThread(void* pData)
{
  #define analyzerThreadData ((TAnalyzerThreadData*)pData)

  for (;;)
  {
    // wait on samplesReady semaphore
    OS_SemaphoreWait(analyzerThreadData->SamplerFullSemaphore, 0);
    // start timer
    OS_TimeSet(0);

    for (;;)
    {
      // get crosses
      uint8_t crossPositions[SAMPLER_WINDOW_CROSSES];
      uint8_t crossesCount;
      for (uint8_t iterator = 1; iterator < SAMPLES_ARRAY_SIZE; iterator++)
      {
        int32_t previous = analyzerThreadData->sampleArray[iterator-1];
        int32_t current = analyzerThreadData->sampleArray[iterator];
        if (PolarityChange(previous, current))
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
        PIT_Disable(analyzerThreadData->channelNb);
        FrequencyTracker(analyzerThreadData->FrequencyTrackerSemaphore, analyzerThreadData->analogSample);
        OS_EnableInterrupts();
      }
      //
      else
      {
    	// get surrounding cross values for RMS calculation and minor frequency updates
		uint8_t sampleCount = crossPositions[2] - crossPositions[0];
		uint32_t surroundingCrossValues[4];
		surroundingCrossValues[0] = analyzerThreadData->sampleArray[crossPositions[0] - 1];
		surroundingCrossValues[1] = analyzerThreadData->sampleArray[crossPositions[0]];
		surroundingCrossValues[2] = analyzerThreadData->sampleArray[crossPositions[2] - 1];
		surroundingCrossValues[3] = analyzerThreadData->sampleArray[crossPositions[2]];

		// set the RMS value for retrieval and alarm monitoring
		SetRMS(analyzerThreadData->sampleArray, surroundingCrossValues, &analyzerThreadData->AlarmMonitoringData->RMS);

		// perform alarm monitoring
		AlarmMonitoring(&analyzerThreadData->AlarmMonitoringData);

		// Perform minor frequency tracking update and store new frequency
		float currentSampleSpeed = (CurrentFrequency * SAMPLES_PER_CYCLE); // samples per second
		uint32_t ticksPerSample = (uint32_t)(ModuleClock / currentSampleSpeed);

		SetNewSamplePeriod(ticksPerSample, surroundingCrossValues, sampleCount);

      }
      analyzerThreadData->fillNewSamples = true;

    }

  }
}

void PassSampleThread(void* pData)
{

  #define passSampleData ((TPassSampleData*)pData)
  uint8_t fillCounter = 0;

  for (;;)
  {
    // wait on TimerCompleteSemaphore
    OS_SemaphoreWait(passSampleData->PITDataSampleTakenSemaphore, 0);

    OS_DisableInterrupts();
    for (int channelNb = 0; channelNb < NB_SAMPLER_CHANNELS; channelNb++)
    {
      if (passSampleData->PassSampleChannelData[channelNb]->analyzerReady)
      {
        // passSampleData->PassSampleChannelData[channelNb]->sampleArray[fillCounter] = passSampleData->PassSampleChannelData[channelNb]->analogSample;
        passSampleData->PassSampleChannelData[channelNb]->sampleArray[fillCounter] = 1;
      }
    }
    OS_EnableInterrupts();
    for (int channelNb = 0; channelNb < NB_SAMPLER_CHANNELS; channelNb++)
    {
      if (passSampleData->PassSampleChannelData[channelNb]->fillNewSamples)
      {
        fillCounter++;
        if (fillCounter == SAMPLES_ARRAY_SIZE)
        {
          passSampleData->PassSampleChannelData[channelNb]->fillNewSamples = false;
          fillCounter = 0;
        }
      }
      if (passSampleData->PassSampleChannelData[channelNb]->analyzerReady)
      {
        // start timer - use an array that stores multiple timers
	    OS_TimeSet(0);
        OS_SemaphoreSignal(passSampleData->PassSampleChannelData[channelNb]->SamplerFullSemaphore);
      }

    }

  }

}

