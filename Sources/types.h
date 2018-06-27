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

/////////

/*
 * 2^2 = 4
 * 2^4 = 16
 * 2^8 = 256
 * 2^16 = 65536
 * 2^32 = 4294967296
 * 2^64 = 18446744073709551616
 */
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
typedef struct PITData
{
  OS_ECB* PITDataSampleTakenSemaphore;
  OS_ECB* PITFrequencySampleTakenSemaphore;
  TPITChannelData PITChannelData[NB_SAMPLER_CHANNELS];
} TPITData;

typedef struct PITChannelData
{
  OS_ECB* PITAlarmSemaphore;
  uint8_t channelNb;
  int32_t analogSample; // pass struct in as ptr
} TPITChannelData;

typedef struct PassSampleData
{
  OS_ECB* PITDataSampleTakenSemaphore; // shared with PIT module
  TPassSampleChannelData PassSampleChannelData[NB_SAMPLER_CHANNELS];
} TPassSampleData;

typedef struct PassSampleChannelData
{
  OS_ECB* SamplerFullSemaphore; // shared with analyzer thread
  bool analyzerReady; // shared with analyzer thread
  bool fillNewSamples; // shared with analyzer thread
  uint8_t channelNb; // shared with all
  uint32_t analogSample; // shared with analyzer thread
  uint32_t sampleArray[SAMPLES_ARRAY_SIZE]; // shared with analyzer thread
} TPassSampleChannelData;

typedef struct AnalyzerThreadData
{
  OS_ECB* SamplerFullSemaphore;
  OS_ECB* FrequencyTrackerSemaphore;
  uint8_t channelNb;
  uint8_t iterator;
  uint32_t sampleArray[SAMPLES_ARRAY_SIZE];
  bool analyzerReady;
  bool fillNewSamples;
  TAlarmMonitoringData AlarmMonitoringData;
} TAnalyzerThreadData;

typedef struct AlarmMonitoringData
{
  uint32_t RMS;
  uint8_t channelNb;
  bool alarmingHigh;
  bool alarmingLow;
} TAlarmMonitoringData;

typedef struct AlarmControlThreadData
{
  OS_ECB* PITAlarmSemaphore;
  uint8_t channelNb; // may not be needed
  uint32_t RMS;
  bool alarmingHigh;
  bool alarmingLow;
} TAlarmControlThreadData;

#endif
