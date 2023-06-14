#include "counting.h"

void SetupPDM() {
  // setup PDM, to get tthe microphone input
  PDM.onReceive(Microphone_availablePDMDataCallback);
  if (!PDM.begin(1, frequency)){
    while(1);
  }
}

void initializeCounter(BLEDevice central, BLECharacteristic characteristic){
  // when the 'configuration' button is pressed, adjust the magnitude threshold, assuming that the people only jumped once
  float temp_values[200];
  float max_value = 0;
  for(int j=0; j<200;j++){
    int sum = 0;
    for (int i=0; i<samplesRead; i++){
      sum += sampleBuffer[i];
    }
    float average = sum / samplesRead;
    float sumRMS = 0.0;
    for (int i=0; i<samplesRead; i++){
      sumRMS += abs(sampleBuffer[i]-average);
    }
    float microphoneRMSValue = sumRMS/samplesRead;
    temp_values[j] = microphoneRMSValue;
    if (microphoneRMSValue > max_value){
      max_value = microphoneRMSValue;
    }
    delay(50);
  }
  float mean_sum = 0;
  int mean_count = 0;
  for(int j = 0; j < 200; j++){
    if (temp_values[j] < (max_value - 10)){
      mean_count += 1;
      mean_sum += temp_values[j];
    }
  }
  float mean_value = mean_sum / mean_count;
  mag_threshold = mean_value + (max_value-mean_value) * 0.4;
}

void Microphone_availablePDMDataCallback()
{
// query the number of bytes available
int bytesAvailable = PDM.available();

  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}

void countJump(bool *done_just_triggered){
if (samplesRead){
  int sum = 0;
  for (int i=0; i<samplesRead; i++){
    sum += sampleBuffer[i];
  }
  float average = sum / samplesRead;
  float sumRMS = 0.0;
  for (int i=0; i<samplesRead; i++){
    sumRMS += abs(sampleBuffer[i]-average);
  }
  float microphoneRMSValue = sumRMS/samplesRead;
  unsigned long timestamp = millis();
  // if RMS value is big enough and the time elapsed long enough
  // it means the person jumped
  if (microphoneRMSValue > mag_threshold & timestamp > prev_count_time + time_threshold){
    prev_count_time = timestamp;
    *done_just_triggered = true;
  }
}
}

