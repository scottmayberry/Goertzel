/*
  The Goertzel algorithm is long standing so see 
  http://en.wikipedia.org/wiki/Goertzel_algorithm for a full description.
  It is often used in DTMF tone detection as an alternative to the Fast 
  Fourier Transform because it is quick with low overheard because it
  is only searching for a single frequency rather than showing the 
  occurrence of all frequencies.
  
  This work is entirely based on the Kevin Banks code found at
  http://www.eetimes.com/design/embedded/4024443/The-Goertzel-Algorithm 
  so full credit to him for his generic implementation and breakdown. I've
  simply massaged it into an Arduino library. I recommend reading his article
  for a full description of whats going on behind the scenes.

  See Contributors.md and add yourself for pull requests
  Released into the public domain.
*/

// ensure this library description is only included once
#ifndef Goertzel_h
#define Goertzel_h

// include types & constants of Wiring core API
#include "Arduino.h"

// library interface description
class Goertzel
{
  // user-accessible "public" interface
public:
  Goertzel(float, float);
  Goertzel(float, float, int);
  Goertzel();
  void reinit(float, float, int);
  void reinit(float, float);

  float detectBatch(byte[], int);
  float detect();
  void addSample(int);
  bool addSampleWithCheck(int, int);
  float detectWithN(int);
  int getSampleIndex();

  float getSampleFreq();
  float getTargetFreq();

  float calcMagnitudeSquared();
  float calcPurity(float, int);

  float applyHammingWindow(float);
  float applyExactBlackman(float);

  void setN(int);

  void setHamming(bool);
  void setExactBlackman(bool);

  // library-accessible "private" interface
private:
  void initialize(float, float, int);
  void GetRealImag(float *, float *);
  void ProcessSample(int);
  void ProcessSample(float);
  void ResetGoertzel();
  int getSumOfSquares(byte[], int);

  bool hamming = false;
  bool exactBlackman = false;
  int N;
  float SAMPLING_FREQUENCY;
  float TARGET_FREQUENCY;
  int readIndex;
  float coeff;
  float sinePart;
  float Q1;
  float Q2;
  int sumOfSquares;
  int ADCCENTER;
  byte *testData;
};

#endif
