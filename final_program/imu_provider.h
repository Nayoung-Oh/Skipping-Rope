#ifndef IMU_PROVIDER_H
#define IMU_PROVIDER_H

#include <Arduino_LSM9DS1.h>
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
namespace {  
  // A buffer holding the last 2 seconds data of 3-channel values from the accelerometer.
  constexpr int acceleration_data_length = 33 * 3 * 2;
  float acceleration_data[acceleration_data_length] = {};
  int acceleration_data_index = 0;
  float acceleration_sample_rate = 0.0f;
  
  // A buffer holding the last 2 seconds of 3-channel values from the gyroscope.
  constexpr int gyroscope_data_length = 33 * 3 * 2;
  float gyroscope_data[gyroscope_data_length] = {};
  float orientation_data[gyroscope_data_length] = {};
  int gyroscope_data_index = 0;
  float gyroscope_sample_rate = 0.0f;

  // to sample similar to the training data
  long prev_time = 0;
  long duration = 30;
    
  void SetupIMU() {
    acceleration_sample_rate = IMU.accelerationSampleRate();
    gyroscope_sample_rate = IMU.gyroscopeSampleRate();

  }
  
  void ReadAccelerometerAndGyroscope(int* new_accelerometer_samples, int* new_gyroscope_samples) {
    // Keep track of whether we stored any new data
    // modified from the given MagicWand example
    *new_accelerometer_samples = 0;
    *new_gyroscope_samples = 0;
    // Loop through new samples and add to buffer
    while (IMU.accelerationAvailable()) {
      long curr_time = millis();
      // if it does not elapsed 'duration' time, ignore this data
      if ((curr_time - prev_time) > duration){
        const int gyroscope_index = (gyroscope_data_index % gyroscope_data_length);
        gyroscope_data_index += 3;
        float* current_gyroscope_data = &gyroscope_data[gyroscope_index];
        // Read each sample, removing it from the device's FIFO buffer
        if (!IMU.readGyroscope(
            current_gyroscope_data[0], current_gyroscope_data[1], current_gyroscope_data[2])) {
          Serial.println("Failed to read gyroscope data");
          break;
        }
        *new_gyroscope_samples += 1;
    
        const int acceleration_index = (acceleration_data_index % acceleration_data_length);
        acceleration_data_index += 3;
        float* current_acceleration_data = &acceleration_data[acceleration_index];
        // Read each sample, removing it from the device's FIFO buffer
        if (!IMU.readAcceleration(
            current_acceleration_data[0], current_acceleration_data[1], current_acceleration_data[2])) {
          Serial.println("Failed to read acceleration data");
          break;
        }
        *new_accelerometer_samples += 1;
        prev_time = curr_time;
      }
    }
  }

  void GetInput(float_t* out_buffer){
    int cur_gyro_index = (gyroscope_data_index % gyroscope_data_length) + 3;
    int cur_acc_index = (acceleration_data_index % acceleration_data_length) + 3;
    for (int i = 0; i < gyroscope_data_length / 3; i++){
      // put whole data, acc gyro, and next timestamp
      out_buffer[i*6] = acceleration_data[cur_acc_index];
      out_buffer[i*6+1] = acceleration_data[cur_acc_index+1];
      out_buffer[i*6+2] = acceleration_data[cur_acc_index+2];
      out_buffer[i*6+3] = gyroscope_data[cur_gyro_index];
      out_buffer[i*6+4] = gyroscope_data[cur_gyro_index+1];
      out_buffer[i*6+5] = gyroscope_data[cur_gyro_index+2];
      cur_acc_index += 3;
      cur_gyro_index += 3;
      cur_acc_index = (cur_acc_index % acceleration_data_length);
      cur_gyro_index = (cur_gyro_index % gyroscope_data_length);
    }
  }
}
#endif   // MAGIC_WAND_IMU_PROVIDER_H
