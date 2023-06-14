#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <PDM.h>

// used for collecting data as fast as possible
// using notify
// set the BLE related parts
BLEService sensorService("66df5109-edde-4f8a-a5e1-02e02a69cbd5");
BLECharacteristic testLevel("3391f4fc-0293-11ee-be56-0242ac120002", BLERead | BLENotify, 20);
BLEUnsignedShortCharacteristic jumpCount("339204c4-0293-11ee-be56-0242ac120002", BLERead | BLENotify);

// last sensor data
float oldax = 0;
float olday = 0;
float oldaz = 0;
float oldgx = 0;
float oldgy = 0;
float oldgz = 0;
long previousMillis = 0;
const int ledPin = LED_BUILTIN;

// default PCM output frequency
static const int frequency = 16000;
#define MICROPHONE_BUFFER_SIZE_IN_WORDS (256U)
#define MICROPHONE_BUFFER_SIZE_IN_BYTES (MICROPHONE_BUFFER_SIZE_IN_WORDS * sizeof(int16_t))

// Buffer to read samples into, each sample is 16-bits
int16_t sampleBuffer[MICROPHONE_BUFFER_SIZE_IN_WORDS];

// Number of audio samples read
volatile int samplesRead;
volatile long prev = 0;
volatile short count = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // initialize BLE, IMU, and PDM (mic)
  if (!BLE.begin()) {
  while (1);
  }
  if (!IMU.begin()) {
  while (1);
  }

  PDM.onReceive(Microphone_availablePDMDataCallback);
  if (!PDM.begin(1, frequency)){
    while(1);
  }

  BLE.setLocalName("DataCollection");
  BLE.setAdvertisedService(sensorService);
  sensorService.addCharacteristic(testLevel);
  sensorService.addCharacteristic(jumpCount);
  BLE.addService(sensorService);
  uint8_t zero = 0;
  testLevel.writeValue(zero, 20);
  jumpCount.writeValue(0);
  BLE.advertise();
  count = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  BLEDevice central = BLE.central();
  if (central) {
    digitalWrite(LED_BUILTIN, HIGH);
    while (central.connected()) {
      updateGyroscopeLevel();
      // additionally check the jump counting algorithm
      countJump();
    }
  digitalWrite(LED_BUILTIN, LOW);
  count = 0;
  }
}

void countJump(){
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
    if (microphoneRMSValue > 20 & timestamp > prev + 500){
      count += 1;
      prev = timestamp;
      jumpCount.writeValue(count);
    }
  }
}


void updateGyroscopeLevel() {
 float ax, ay, az, gx, gy, gz;
 if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);
  short timestamp = millis();
  // for faster communication, concat data into 20byte which is the maximum data size for one packet
  uint8_t bytes [20]{
    ((uint16_t)timestamp >> 8) & 0xFF,
    ((uint16_t)timestamp >> 0) & 0xFF,
    ((short)(ax*100) >> 8) & 0xFF,
    ((short)(ax*100) >> 0) & 0xFF,
    ((short)(ay*100) >> 8) & 0xFF,
    ((short)(ay*100) >> 0) & 0xFF,
    ((short)(az*100) >> 8) & 0xFF,
    ((short)(az*100) >> 0) & 0xFF,
    ((int32_t)(gx*100) >> 24) & 0xFF,
    ((int32_t)(gx*100) >> 16) & 0xFF,
    ((int32_t)(gx*100) >> 8) & 0xFF,
    ((int32_t)(gx*100) >> 0) & 0xFF,
    ((int32_t)(gy*100) >> 24) & 0xFF,
    ((int32_t)(gy*100) >> 16) & 0xFF,
    ((int32_t)(gy*100) >> 8) & 0xFF,
    ((int32_t)(gy*100) >> 0) & 0xFF,
    ((int32_t)(gz*100) >> 24) & 0xFF,
    ((int32_t)(gz*100) >> 16) & 0xFF,
    ((int32_t)(gz*100) >> 8) & 0xFF,
    ((int32_t)(gz*100) >> 0) & 0xFF,
  };
  testLevel.writeValue(bytes, 20);
 }
}

void Microphone_availablePDMDataCallback()
{
 // query the number of bytes available
 int bytesAvailable = PDM.available();

  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}

