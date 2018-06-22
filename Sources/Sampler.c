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
#define SAMPLES_ARRAY_SIZE 28

// if samples = null[size - 1], do nothing

// ----------------------------------------
// Thread set up
// ----------------------------------------
// Arbitrary thread stack size - big enough for stacking of interrupts and OS use.
#define THREAD_STACK_SIZE 100
#define NB_SAMPLER_CHANNELS 3

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE); /*!< The stack for the LED Init thread. */
static uint32_t AnalogThreadStacks[NB_SAMPLER_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

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
  OS_ECB* StoreSampleSemaphore;
  OS_ECB* AnalyzeSampleSemaphore;
  uint8_t channelNb;
  int32_t analogSample;
  int32_t analogSamples[SAMPLES_ARRAY_SIZE];
  typedef struct Sampler
} TSamplerThreadData; // TODO: WE WILL HAVE TO BREAK THIS UP

/*! @brief Analog thread configuration data
 *
 */
static TSamplerThreadData SamplerThreadData[NB_SAMPLER_CHANNELS] =
{
  {
    .StoreSampleSemaphore = NULL,
    .AnalyzeSampleSemaphore = NULL,
    .channelNb = 0,
    .analogSample = NULL,
    .analogSamples = NULL
  },
  {
    .StoreSampleSemaphore = NULL,
    .AnalyzeSampleSemaphore = NULL,
    .channelNb = 1,
    .analogSample = NULL,
    .analogSamples = NULL
  },
  {
    .StoreSampleSemaphore = NULL,
    .AnalyzeSampleSemaphore = NULL,
    .channelNb = 0,
    .analogSample = NULL,
    .analogSamples = NULL
  }
};



Sampler_Init(const uint32_t moduleClk)
{

}

// consider 3 threads for this one rather than one as it will be less messy
AnalyzerThread(void* pData)
{
  #define samplerStruct ((TSamplerThreadData*)pData)

  // Possibly add specialised initial sampler thread that destroys itself (high sampling frequency, tracks from a zero to zero)
  // Don't analyze the samples while the array isn't full
  do {  } while (samplerStruct[NB_SAMPLER_CHANNELS - 1]->analogSamples[SAMPLES_ARRAY_SIZE - 1] == NULL);

  // if sampling freq is wrong, samples[] = null and thread returns to start



}
