// https://github.com/tensorflow/tflite-micro-arduino-examples
// Used the above github instead of KAIST library

#include <TensorFlowLite.h>

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "model_data.h"
#include "imu_provider.h"
#include "counting.h"

#include <ArduinoBLE.h>

namespace {

const int VERSION = 0x00000000;

// Constants for data collection
constexpr int window_size = 66; // 33 Hz for 2 seconds
constexpr int feature_num = 6; // acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z
constexpr int data_byte_count = window_size * feature_num;
float_t data_buffer[data_byte_count];

// Create an area of memory to use for input, output, and intermediate arrays.
constexpr int kTensorArenaSize = 30 * 1024;
alignas(16) uint8_t tensor_arena[kTensorArenaSize];
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;

// label of jump type
constexpr int label_count = 4;
short count_per_label[label_count] = {0, 0, 0, 0};
const char* labels[label_count] = { "NORMAL", "X", "BACK", "DOUBLE" };
int8_t cur_type = 0;

// for communication with the smartphone
BLEService logService("66df5109-edde-4f8a-a5e1-02e02a69cbd5");

// characteristic to send counting values
BLEShortCharacteristic NormalsendLogger("3391f4fc-0293-11ee-be56-0242ac120002", BLERead | BLENotify);
BLEShortCharacteristic XsendLogger("3391fc72-0293-11ee-be56-0242ac120002", BLERead | BLENotify);
BLEShortCharacteristic BacksendLogger("3391fa38-0293-11ee-be56-0242ac120002", BLERead | BLENotify);
BLEShortCharacteristic DoublesendLogger("3391fb64-0293-11ee-be56-0242ac120002", BLERead | BLENotify);

// To get the button click event from the smartphone
BLEShortCharacteristic receiveLogger("3391f8e4-0293-11ee-be56-0242ac120002", BLERead | BLEWrite);
BLEShortCharacteristic resetLogger("33920370-0293-11ee-be56-0242ac120002", BLERead | BLEWrite);

}  // namespace

void setup() {
  // Start serial
  tflite::InitializeTarget();
  Serial.begin(9600);

  // Start IMU
  if (!IMU.begin()) {
    while (1);
  }
  // Start BLE
  if (!BLE.begin()){
    while(1);
  }
  SetupIMU();
  SetupPDM();

  // Set the name of BLE as "JumpCounter", make it easy to find
  BLE.setLocalName("JumpCounter");
  BLE.setAdvertisedService(logService);
  // Add characteristic to the service
  logService.addCharacteristic(NormalsendLogger);
  logService.addCharacteristic(XsendLogger);
  logService.addCharacteristic(BacksendLogger);
  logService.addCharacteristic(DoublesendLogger);

  logService.addCharacteristic(receiveLogger);
  logService.addCharacteristic(resetLogger);
  // Add service
  BLE.addService(logService);
  // set event handler to read button push in the smartphone application
  receiveLogger.setEventHandler(BLEWritten, initializeCounter);
  resetLogger.setEventHandler(BLEWritten, reset);

  // intialize values
  NormalsendLogger.setValue(0);
  XsendLogger.setValue(0);
  BacksendLogger.setValue(0);
  DoublesendLogger.setValue(0);
  receiveLogger.setValue(0);
  resetLogger.setValue(0);
  BLE.advertise();

  model = tflite::GetModel(g_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    MicroPrintf("Model provided is schema version %d not equal "
                         "to supported version %d.",
                         model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // add only used operations, as extracted in the colab code
  static tflite::MicroMutableOpResolver<8> micro_op_resolver;
  micro_op_resolver.AddAdd();
  micro_op_resolver.AddConv2D();
  micro_op_resolver.AddExpandDims();
  micro_op_resolver.AddFullyConnected();
  micro_op_resolver.AddMaxPool2D();
  micro_op_resolver.AddMul();
  micro_op_resolver.AddReshape();
  micro_op_resolver.AddSoftmax();

  // build an interpreter to run the model
  static tflite::MicroInterpreter static_interpreter(
    model, micro_op_resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  // allocate memory from the tensor_arena for the model's tensors.
  interpreter->AllocateTensors();

  // set model input settings, check the model size and dimensions
  TfLiteTensor* model_input = interpreter->input(0);
  if ((model_input->dims->size != 3) || (model_input->dims->data[0] != 1) || (model_input->dims->data[1] != window_size) || (model_input->dims->data[2] != feature_num) || (model_input->type != kTfLiteFloat32)) {
    MicroPrintf("Bad input tensor parameters in model");
    return;
  }

  // set model output settings, check the model output size
  TfLiteTensor* model_output = interpreter->output(0);
  if ((model_output->dims->size != 2) || (model_output->dims->data[0] != 1) || (model_output->dims->data[1] != label_count) || (model_output->type != kTfLiteFloat32)) {
    MicroPrintf("Bad output tensor parameters in model");
    return;
  }
}

void reset(BLEDevice central, BLECharacteristic characteristic){
  // after the reset button pressed, reset the counting history
  for (int i=0; i<label_count; i++){
    count_per_label[i] = 0;
  }
}

void loop() {
  // make sure IMU data is available then read in data
  BLEDevice central = BLE.central();
  if (central){
    while (central.connected()) {
      const bool data_available = IMU.accelerationAvailable() || IMU.gyroscopeAvailable();
      if (!data_available) {
        // if imu data is not available, igonre the loop
        continue;
      }

      // read the accleration and gyroscope data
      int accelerometer_samples_read = 0;
      int gyroscope_samples_read = 0;
      ReadAccelerometerAndGyroscope(&accelerometer_samples_read, &gyroscope_samples_read);

      bool done_just_triggered = false;
      if (gyroscope_samples_read > 0) {
        // check whether the jump is invoked
        countJump(&done_just_triggered);
      }
      // only when the jump is detected
      if (done_just_triggered) {
        // read the imu sensor buffer into the data_buffer
        GetInput(data_buffer);
        TfLiteTensor* model_input = interpreter->input(0);
        // put the data into the model
        for (int i = 0; i < data_byte_count; ++i) {
          model_input->data.f[i] = data_buffer[i];
        }
        // invoke the model
        TfLiteStatus invoke_status = interpreter->Invoke();
        if (invoke_status != kTfLiteOk) {
          MicroPrintf("Invoke failed");
          return;
        }
        // and get the output
        TfLiteTensor* output = interpreter->output(0);

        // get the max score (probability) type
        float max_score;
        int max_index;
        for (int i = 0; i < label_count; ++i) {
          const float score = output->data.f[i];
          if ((i == 0) || (score > max_score)) {
            max_score = score;
            max_index = i;
          }
        }
        cur_type = max_index;
        count_per_label[cur_type] += 1;

        MicroPrintf("Found %s (%f)", labels[max_index], max_score);
        // send the type information into the smartphone through BLE
        switch (cur_type){
          case 0:
            NormalsendLogger.writeValue(count_per_label[cur_type]);
            break;
          case 1:
            XsendLogger.writeValue(count_per_label[cur_type]);
            break;
          case 2:
            BacksendLogger.writeValue(count_per_label[cur_type]);
            break;
          case 3:
            DoublesendLogger.writeValue(count_per_label[cur_type]);
            break;
        }
      }
    }
  } 
}

