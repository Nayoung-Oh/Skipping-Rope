#ifndef COUNTING_H
#define COUNTING_H

#include <Arduino_LSM9DS1.h>
#include <PDM.h>
#include <ArduinoBLE.h>

// define for jump counting
// using builtin mic

namespace {  
  int samplesRead;
  // based on the built in mic spec
  static const int frequency = 16000;
  #define MICROPHONE_BUFFER_SIZE_IN_WORDS (256U)
  #define MICROPHONE_BUFFER_SIZE_IN_BYTES (MICROPHONE_BUFFER_SIZE_IN_WORDS * sizeof(int16_t))
  int16_t sampleBuffer[MICROPHONE_BUFFER_SIZE_IN_WORDS];
  // the mag_threshold can be changed through 'configuration' step
  float mag_threshold = 30;
  float time_threshold = 350;
  long prev_count_time = 0;
}
void SetupPDM();

void initializeCounter(BLEDevice central, BLECharacteristic characteristic);

void Microphone_availablePDMDataCallback();

void countJump(bool *done_just_triggered);

#endif   // counting
