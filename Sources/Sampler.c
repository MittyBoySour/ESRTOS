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

static TFrequencyData FrequencyData;
static TPassSampleData PassSampleData;
static TPITData PITData;
static TAnalyzerData AnalyzerData[NB_SAMPLER_CHANNELS];
static TSampleManagerData SamplerManagerData;

// Global data
static bool InverseTimingModeEnabled;

Sampler_Init(const uint32_t moduleClk) // May have to be thread
{
  // Initial setup
  ModuleClock = moduleClk;
  InverseTimingModeEnabled = false;
  // PIT_Init
  PIT_Init();

  // Shared thread data
  // Sampler manager data
  SamplerManagerData =
  {
    .FrequencyLostSemaphore = OS_SemaphoreCreate(0),
    .VoltageOutOfBoundsSemaphore = OS_SemaphoreCreate(0)
  }

  // Frequency data
  FrequencyData =
  {
    .FrequencyTrackedSemaphore = OS_SemaphoreCreate(0),
    .matchedFrequency = matchedFrequency
  }

  // SamplePasser data
  PassSampleData.PITTimerCompleteSemaphore = OS_SemaphoreCreate(0);
  for (int i = 0; i < NB_SAMPLER_CHANNELS; i++)
  {
    .PassSampleChannelData[i] =
    {
      .SamplerFullSemaphore = OS_SemaphoreCreate(0),
      .FrequencyLost = true,
      .channelNb = i,
      .iterator = 0,
      // .analogSample = 0,
      // .sampleArray[0] == 0,
    }
  };

  // PIT data --> this may be used within other functions
  PITData = ConvertPassSampleDataToPITData(PassSampleData);
  // Analyzer data
  AnalyzerData[NB_SAMPLER_CHANNELS];
  for (int channelNb = 0; channelNb < NB_SAMPLER_CHANNELS; channelNb++)
  {
    TAnalyzerData AnalyzerData[channelNb] = ConvertPassSampleDataToAnalyzerData(PassSampleData, channelNb);
  }
}

// SamplerThread --> runs threads and calls initial setup too
void SamplerManagerThread(void* pData)
{
  // FrequencyMatcherThread --> 1 thread
  error = OS_ThreadCreate(FrequencyMatcherThread,
                          &FrequencyData,
                          &InitModulesThreadStack[THREAD_STACK_SIZE - 1],
                          5); // Freq thread priority

  // block this until complete (may need to make not a thread) or pass in semaphore and wait on it
  OS_SemaphoreWait(FrequencyData->FrequencyTrackedSemaphore, 0);
  // convert freq to ticksPerSample
  uint32_t ticksPerSample = ModuleClock / (FrequencyData->matchedFrequency * SAMPLES_PER_CYCLE);
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
                            &AnalyzerData[threadNb],
                            &SamplerThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            SAMPLER_THREAD_PRIORITIES[threadNb]); // analyzer thread priority
  }

}

void AlarmThread(void* pData)
{
  //initial data
  #define RMSData ((TAlarmData*)pData)
  // need to be in shared struct
  bool alarmingHigh = false;
  bool alarmingLow = false;
  for (;;)
  {
    OS_SemaphoreWait(RMSData->VoltageCalculatedSemaphore, 0);

    uint32_t RMS = RMSData->RMS;
    uint32_t DeviationValue;
    // deviation = Deviation(RMS, &DeviationValue);
    if (alarmingHigh)
    {
      if (!AboveHighBound(RMS))
        alarmingHigh = false;
    }
    else if (alarmingLow)
    {
      if (!AboveLowBound(RMS))
        alarmingLow = false;
    }
    if (alarming && OutOfLimits(RMS))
    {
      if (InverseTimingModeEnabled)
        // updatePIT

    }
    else if (!alarming && OutOfLimits(RMS))
    {
      alarming = true;
    }
    else if (alarming && !OutOfLimits(RMS))
    {
      alarming = false;
    }
    // wait on votage calculated semaphore
    // if

  }

}



// consider 3 threads for this one rather than one as it will be less messy
void AnalyzerThread(void* pData)
{
  #define analyzerData ((TAnalyzerData*)pData)

  // put in parent
  TAlarmData AlarmData =
  {
    // .RMS = 0,
    .VoltageCalculatedSemaphore = OS_SemaphoreCreate(0)
  }
  error = OS_ThreadCreate(AlarmThread,
                          &AlarmData,
                          &InitModulesThreadStack[THREAD_STACK_SIZE - 1],
                          5); // Highest priority

  // pass RMS ptr to alarm threads
  // wait on samplesReady semaphore
  // immediately store locally (array and iterator - interrupt disabled)

  // if crosses < 3 & < 15 samples
    // disable PIT
    // raise match freq thread
    // notify passSamp

  // check voltage limits
    // signal new voltage calculated

  // send bool readyForMoreSamples
}

// PIT_Update will be where PITs can be turned off

// TODO: check whether we need to use floats
void FrequencyMatcherThread(void* pData)
{
  // cast ptr
  #define FrequencyData ((TFrequencyData*)pData)

  // don't need want shared semaphore for this one, we create new
  // so that only it will be raised and not sample thread semaphore
  // once freq analysis done, sampling semaphore will be passed to PIT0 in parent function

  // TODO: Change to higher freq, and consider eliminating magic numbers
  uint8_t sampleSpeed = HFR * SAMPLES_PER_CYCLE; // samples per second
  uint32_t ticksPerSample = ModuleClock / sampleSpeed;
  // this will be created in earlier thread and fed into all other threads
  // TODO: modify according to types
  TPITData PITData =
  {
    .FrequencyAnalyzerSemaphore = OS_SemaphoreCreate(0),
    .channelNb = 0
    // .analogSample = 0
  }
  // continue waiting on and taking samples until 3 crossings found
  uint8_t crossCount;
  bool inWindow = false;
  int32_t surroundingCrossValues[4];
  int8_t sampleCount = 0;
  // set PIT, pass it the temp channel and semaphore - or set freq as a mode
  PIT_Set(ticksPerSample, true, PITData) // period, restart, data (ptrStorage, chan, sem)
  // wait on first value
  OS_SemaphoreWait(PITData->FrequencyAnalyzerSemaphore, 0); // maybe drop out if an error
  // place first sample into previous for comparison
  int32_t previousValue = PITData->analogSample;
  int32_t currentValue;
  bool previousStatePos;
  bool currentStatePos;
  while (crossCount < SAMPLER_WINDOW_CROSSES) // 3
  {
    OS_SemaphoreWait(PITData->FrequencyAnalyzerSemaphore, 0);
    currentValue = PITData->analogSample;
    // check for crossing and update crossing values and count
    previousStatePos = previousValue > 0;
    currentStatePos = currentValue > 0;
    if (previousStatePos != currentStatePos)
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
  uint32_t waveStart = linearlyInterpolate(ticksPerSample, surroundingCrossValues[1], surroundingCrossValues[0]);
  uint32_t waveEnd = linearlyInterpolate(ticksPerSample, surroundingCrossValues[2], surroundingCrossValues[3]);

  // store freq into passed freq param
  // Hz = waveStart + ((sampleCount /* - 1 */) * ticksPerSample + waveEnd;
  FrequencyData->matchedFrequency = waveStart + ((sampleCount /* - 1 */) * ticksPerSample + waveEnd;

  OS_SemaphoreSignal(FrequencyData->FrequencyTrackedSemaphore);

  OS_ThreadDelete(OS_PRIORITY_SELF); // or consider using semaphore
}

uint32_t linearlyInterpolate(const uint32_t ticksPerSample, const int32_t inCycleValue, const int32_t outOfCycleValue)
{
  int32_t sampleAmpChange = abs(inCycleValue) + abs(outOfCycleValue);
  int32_t inCycleAmpChange = abs(inCycleValue - 0);
  int32_t partialSample = ticksPerSample * (inCycleAmpChange / sampleAmpChange);
  return partialSample;
}

void PassSampleThread(void* pData)
{

  #define passSampleData ((TPassSampleData*)pData)
  bool samplerArraysNeedFilling = true;

  for (;;)
  {
    // wait on TimerCompleteSemaphore
    OS_SemaphoreWait(passSampleData->PITTimerCompleteSemaphore, 0);
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

      if (passSampleData->PassSampleChannelData[channelNb]->FrequencyLost && passSampleData->PassSampleChannelData[channelNb]->sampleArray[SAMPLES_ARRAY_SIZE - 1] != NULL) // may need to iterate through
      {
        passSampleData->PassSampleChannelData[channelNb]->FrequencyLost = false;
      }
    }
    // if analyzerReady bool and samplerFull bool
      // raise the SamplesReady semaphore

    // raise semaphores all at once
    for (int channelNb = 0; channelNb < NB_SAMPLER_CHANNELS; channelNb++)
    {
      if (passSampleData->PassSampleChannelData[channelNb]->FrequencyLost)
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
