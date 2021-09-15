/*
  The Goertzel algorithm is long standing so see 
  http://en.wikipedia.org/wiki/Goertzel_algorithm for a full description.
  It is often used in DTMF tone detection as an alternative to the Fast 
  Fourier Transform because it is quick with low overheard because it
  is only searching for a single frequency rather than showing the 
  occurrence of all frequencies.
  
  This work is entirely based on the Kevin Banks code found at
  http://www.embedded.com/design/configurable-systems/4024443/The-Goertzel-Algorithm 
  so full credit to him for his generic implementation and breakdown. I've
  simply massaged it into an Arduino library. I recommend reading his article
  for a full description of whats going on behind the scenes.

  Created by Jacob Rosenthal, June 20, 2012.
  Released into the public domain.
*/
// include core Wiring API
#include "Arduino.h"

// include this library's description file
#include "Goertzel.h"

Goertzel::Goertzel(float in_TARGET_FREQUENCY, float in_SAMPLING_FREQUENCY, int in_ADCCENTER)
{
  initialize(in_TARGET_FREQUENCY, in_SAMPLING_FREQUENCY, in_ADCCENTER);
}

Goertzel::Goertzel(float in_TARGET_FREQUENCY, float in_SAMPLING_FREQUENCY)
{
  initialize(in_TARGET_FREQUENCY, in_SAMPLING_FREQUENCY, 128);
}

Goertzel::Goertzel()
{
  initialize(24000, 240000, 128);
}

void Goertzel::initialize(float in_TARGET_FREQUENCY, float in_SAMPLING_FREQUENCY, int in_ADCCENTER)
{
  SAMPLING_FREQUENCY = in_SAMPLING_FREQUENCY; //on 16mhz, ~8928.57142857143, on 8mhz ~44444
  TARGET_FREQUENCY = in_TARGET_FREQUENCY;     //should be integer of SAMPLING_RATE/N
  ADCCENTER = in_ADCCENTER;

  float omega = (2.0 * PI * TARGET_FREQUENCY) / SAMPLING_FREQUENCY;

  coeff = 2.0 * cos(omega);
  sinePart = sin(omega);

  ResetGoertzel();
}

void Goertzel::reinit(float in_TARGET_FREQUENCY, float in_SAMPLING_FREQUENCY)
{
  initialize(in_TARGET_FREQUENCY, in_SAMPLING_FREQUENCY, 128);
}

void Goertzel::reinit(float in_TARGET_FREQUENCY, float in_SAMPLING_FREQUENCY, int in_ADCCENTER)
{
  initialize(in_TARGET_FREQUENCY, in_SAMPLING_FREQUENCY, in_ADCCENTER);
}

float Goertzel::getSampleFreq()
{
  return SAMPLING_FREQUENCY;
}

float Goertzel::getTargetFreq()
{
  return TARGET_FREQUENCY;
}

/* Call this routine before every "block" (size=N) of samples. */
void Goertzel::ResetGoertzel()
{
  Q2 = 0;
  Q1 = 0;
  sumOfSquares = 0;
  readIndex = 0;
}

// float Goertzel::applyHammingWindow(int sample)
// {
//   return 0.54 - 0.46 * cos(2 * PI * sample / readIndex);
// }

/* Call this routine for every sample. */
void Goertzel::ProcessSample(byte sample)
{
  float Q0;
  Q0 = coeff * Q1 - Q2 + (sample - ADCCENTER);
  Q2 = Q1;
  Q1 = Q0;
}

int Goertzel::getSampleIndex()
{
  return readIndex;
}

bool Goertzel::addSampleWithCheck(int sample, int n)
{
  if (readIndex < n)
  {
    sumOfSquares += (sample - ADCCENTER) * (sample - ADCCENTER);
    ProcessSample(sample);
    readIndex++;
    return true;
  }
  return false;
}

void Goertzel::addSample(int sample)
{
  sumOfSquares += (sample - ADCCENTER) * (sample - ADCCENTER);
  ProcessSample(sample);
  readIndex++;
}

float Goertzel::detectWithN(int n)
{
  float purity = calcPurity(calcMagnitudeSquared(), n);
  ResetGoertzel();
  return purity;
}

float Goertzel::calcMagnitudeSquared()
{
  return Q1 * Q1 + Q2 * Q2 - coeff * Q1 * Q2;
}

float Goertzel::calcPurity(float magnitudesquared, int n)
{
  return (2 * magnitudesquared) / (n * sumOfSquares);
}

float Goertzel::detect()
{
  float purity = calcPurity(calcMagnitudeSquared(), readIndex);
  ResetGoertzel();
  return purity;
}

int Goertzel::getSumOfSquares(byte samples[], int n)
{
  int sumOfSquaresTemp = 0;
  for (int i = 0; i < n; i++)
  {
    sumOfSquaresTemp += (samples[i] - ADCCENTER) * (samples[i] - ADCCENTER);
  }
  return sumOfSquaresTemp;
}

float Goertzel::detect(byte samples[], int n)
{
  ResetGoertzel();
  testData = samples;

  sumOfSquares = getSumOfSquares(samples, n);

  /* Process the samples. */
  for (int index = 0; index < n; index++)
  {
    ProcessSample(testData[index]);
  }

  float purity = calcPurity(calcMagnitudeSquared(), n);
  ResetGoertzel();
  return purity;
}
