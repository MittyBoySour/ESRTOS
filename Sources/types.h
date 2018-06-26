/*! @file
 *
 *  @brief Declares new types.
 *
 *  This contains types that are especially useful for the Tower to PC Protocol.
 *
 *  @author PMcL
 *  @date 2015-07-23
 */

#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>
#include <stdbool.h>

// ----------------------------------------
// Frequency / Sampling set up
// ----------------------------------------
#define DEFAULT_FREQUENCY 50
#define LFR 47.5 // Lowest frequency range
#define HFR 52.5 // Highest frequency range
#define SAMPLES_PER_CYCLE 16
#define SAMPLES_ARRAY_SIZE 32 // may need to make larger to ensure always 2 zeros or just use freq thread
#define SAMPLER_WINDOW_CROSSES 3

#define NB_SAMPLER_CHANNELS 3

// Unions to efficiently access hi and lo parts of integers and words
typedef union
{
  int16_t l;
  struct
  {
    int8_t Lo;
    int8_t Hi;
  } s;
} int16union_t;

typedef union
{
  uint16_t l;
  struct
  {
    uint8_t Lo;
    uint8_t Hi;
  } s;
} uint16union_t;

// Union to efficiently access hi and lo parts of a long integer
typedef union
{
  uint32_t l;
  struct
  {
    uint16_t Lo;
    uint16_t Hi;
  } s;
} uint32union_t;

// Union to efficiently access hi and lo parts of a "phrase" (8 bytes)
typedef union
{
  uint64_t l;
  struct
  {
    uint32_t Lo;
    uint32_t Hi;
  } s;
} uint64union_t;

// Union to efficiently access individual bytes of a float
typedef union
{
  float d;
  struct
  {
    uint16union_t dLo;
    uint16union_t dHi;
  } dParts;
} TFloat;

// Thread data passing

// struct to allow easy passing of data to PIT module
typedef struct PITData =
{
  OS_ECB* PITTimerCompleteSemaphore;
  uint8_t channelsCount;
  typedef struct PITChannelData[NB_SAMPLER_CHANNELS] =
  {
    uint8_t channelNb;
    int32_t analogSample; // pass struct in as ptr
  } TPITChannelData;
} TPITData;

// struct to allow easy passing of Frequency data to thread
typedef struct FrequencyData =
{
  OS_ECB* FrequencyTrackedSemaphore;
  uint32_t matchedFrequency;
} TFrequencyData;

typedef struct PassSampleData =
{
  OS_ECB* PITTimerCompleteSemaphore; // shared with PIT module
  typedef struct PassSampleChannelData[NB_SAMPLER_CHANNELS] =
  {
    OS_ECB* SamplerFullSemaphore; // shared with analyzer thread
    bool FrequencyLost; // shared with analyzer thread
    uint8_t channelNb; // shared with all
    uint32_t analogSample; // shared with analyzer thread
    uint32_t sampleArray[SAMPLES_ARRAY_SIZE]; // shared with analyzer thread
  } TPassSampleChannelData;
} TPassSampleData;

typedef struct AnalyzerData =
{
  OS_ECB* SamplerFullSemaphore;
  uint8_t channelNb;
  uint8_t iterator;
  uint32_t sampleArray[SAMPLES_ARRAY_SIZE];
} TAnalyzerData;

typedef struct SamplerManagerData =
{
  OS_ECB* FrequencyLostSemaphore;
  OS_ECB* VoltageOutOfBoundsSemaphore;
}

typedef struct AlarmData =
{
  uint32_t RMS;
  OS_ECB* VoltageCalculatedSemaphore;
} TAlarmData


// Thread data morph functions to be implemented in c file

// convert PassSample data to PIT data
TPITData ConvertPassSampleDataToPITData(const TPassSampleData PassSampleData)
{
  TPITData PITData;
  PITData.PITTimerCompleteSemaphore = PassSampleData->PITTimerCompleteSemaphore;
  for (int i = 0; i < PassSampleData->channelsCount; i++)
  {
    PITData.PITChannelData[i] =
    {
      .channelNb = PassSampleData->PassSampleChannelData[i]->channelNb;
      .analogSample = PassSampleData->PassSampleChannelData[i]->analogSample;
    }
  }
  return PITData;
}

// convert PassSample data to Analyzer data
TAnalyzerData ConvertPassSampleDataToAnalyzerData(const TPassSampleData PassSampleData, const uint8_t channelNb)
{
  TAnalyzerData AnalyzerData;
  AnalyzerData.SamplerFullSemaphore = PassSampleData->PassSampleChannelData[channelNb]->SamplerFullSemaphore;
  AnalyzerData.channelNb = channelNb; // for ease as responsibility is on caller
  AnalyzerData.iterator = PassSampleData->PassSampleChannelData[channelNb]->iterator;
  AnalyzerData.sampleArray = PassSampleData->PassSampleChannelData[channelNb]->sampleArray;
  return AnalyzerData;
}

TAnalyzerData ConvertSampleManagerDataToAnalyzerData(const TSampleManagerData SampleManagerData, const uint8_t channelNb)
{
  AnalyzerData.FrequencyLostSemaphore = SampleManagerData[channelNb]->FrequencyLostSemaphore;
  AnalyzerData.VoltageOutOfBoundsSemaphore = SampleManagerData[channelNb]->VoltageOutOfBoundsSemaphore;

}

#endif
