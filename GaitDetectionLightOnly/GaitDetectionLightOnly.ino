//Created by Nitzan Breitman and Omri Yosfan. Contact: nitzan.breitman@gmail.com

/*
   This version has the following:

   - When sending 02 from nRF connect, the LED turns-on on heel-strike events and turns-off according to VIBRATION_END_PERCENTAGE
   - Recording data to buffer on 02
   - Record data from nRF connect when sending 03
   - To send gyro data on 02, uncomment "Used to be line 356"
   - To use the heel-strike detection for other applications, then adjust the "vibrationTask" function according to your needs
   - Follow the explanation on our GitHub page to control and record from the phone
   - Licensed under the Educational Community License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. https://opensource.org/licenses/ECL-2.0
   - Copyright 2022 Nitzan Breitman 
*/
#include <MovingAverage.h>    //By Alexandre Hiroyuki
#include <ArduinoBLE.h>       //A general thanks to Klaus_K from the Arduino forum for helping with BLE data transmission
#include <Arduino_LSM9DS1.h>  //By Femme Verbeek 2.0, Calibration per enclosure is advised https://www.youtube.com/watch?v=BLvYFXoP33o
#include <Wire.h>
#include "Adafruit_DRV2605.h"

#define MOTOR_LED D6
#define RED 22
#define BLUE 24
#define GREEN 23
#define LED_PWR 25
Adafruit_DRV2605 drv;
MovingAverage<unsigned long> stepLengthAvg(3);  //Makes MovingAvg object with 3 points
MovingAverage<float> peakValueAvg(3);

//Multiplexer
#define TCAADDR 0x70
#define FIRST_DRV 2   //I don't know why but this line has to stay for the two motors to work independently, even when motor 1 is not connected to the multiplexer.02.08.2022
#define SECOND_DRV 1  //Number is channel # on multiplexer

//----------------------------------------------------------------------------------------------------------------------
// User-friendly variable stuff
//----------------------------------------------------------------------------------------------------------------------

#define RECORDING_LENGTH_SECONDS 15           //Length of each trial (seconds)
#define VIBRATION_START_PERCENTAGE_MOTOR1 0   //Percentage of gait cycle when vibration activates (20 = motor starts 20% after HS) motor1= upper motor
#define VIBRATION_START_PERCENTAGE_MOTOR2 30  //MOTOR2=lower motor
#define VIBRATION_END_PERCENTAGE 60           //Percentage of gait cycle when vibration deactivates (60 = motor stops 60% after HS)
#define STEP_LENGTH_THRESHOLD 1.5             //Percentage at which step length won't be added to stepLengthAvg (1.5 = 150%)
int SAMPLES_TO_IGNORE_AFTER_HS = 5;           //Prevents early detection of HS on a spike
float MS_THRESHOLD_PCT_OF_PEAK = 0.5;         //Percentage of moving avg peak value for MS detection threshold
float INITIAL_MS_THRESHOLD = 100;             //Fixed-value threshold for initial two steps

//----------------------------------------------------------------------------------------------------------------------
// BLE UUIDs
//----------------------------------------------------------------------------------------------------------------------

#define BLE_UUID_SENSOR_SERVICE "19120000-1713-4AE2-5A50-946B02A2FFAF"
#define BLE_UUID_SENSOR "19120001-1713-4AE2-5A50-946B02A2FFAF"
#define BLE_UUID_SENSOR_CONTROL "19120002-1713-4AE2-5A50-946B02A2FFAF"
#define BLE_UUID_CHARACTERISTIC_USER_DESCRIPTION "2901"

//----------------------------------------------------------------------------------------------------------------------
// BLE Setup
//----------------------------------------------------------------------------------------------------------------------

typedef struct __attribute__((packed)) {
  uint32_t position;
  uint32_t time;
} sensor_data_t;

typedef union {
  sensor_data_t values;
  uint8_t bytes[sizeof(sensor_data_t)];
} sensor_data_ut;

sensor_data_ut sensorData;

bool sensorDataUpdated = false;
//----- DEVICE NAME -----//
#define BLE_DEVICE_NAME "Nitzan Arduino"
#define BLE_LOCAL_NAME "Nitzan Arduino"

BLEService sensorService(BLE_UUID_SENSOR_SERVICE);
BLECharacteristic sensorCharacteristic(BLE_UUID_SENSOR, BLERead | BLENotify, sizeof(sensor_data_ut));
BLEByteCharacteristic controlCharacteristic(BLE_UUID_SENSOR_CONTROL, BLERead | BLEWrite);

BLEDescriptor controlDescriptor(BLE_UUID_CHARACTERISTIC_USER_DESCRIPTION, "CMD:01-RESET,02-RECORD,03-REPLAY");

//----------------------------------------------------------------------------------------------------------------------
// APP & I/O
//----------------------------------------------------------------------------------------------------------------------

// Reversing Byte order allows uint32_t data to be converted in Excel with HEX2DEC function OR MATLAB code
#define REVERSE_BYTE_ORDER

#define DATA_BUFFER_SIZE (RECORDING_LENGTH_SECONDS * 100)  // 20 sec X 100 Hz    Changes buffer size

enum BUFER_STATE_TYPE { BUFFER_STATE_READY,
                        BUFFER_STATE_RECORD,
                        BUFFER_STATE_REPLAY,
                        BUFFER_STATE_FULL
};

typedef struct __attribute__((packed)) {
  sensor_data_t values[DATA_BUFFER_SIZE];
  uint32_t index;
  uint32_t state = BUFFER_STATE_READY;
} sensor_buffer_t;

sensor_buffer_t dataBuffer;
bool sensorActive = false;


#define BLE_LED_PIN LED_BUILTIN
#define BUFFER_LED_PIN LED_PWR
#define SENSOR_PIN A0


//----------------------------------------------------------------------------------------------------------------------
// Setup
//----------------------------------------------------------------------------------------------------------------------

void setup() {



  Wire.begin();
  Wire.setClock(400000);
  // Serial.begin(115200);
  pinMode(BLE_LED_PIN, OUTPUT);
  digitalWrite(BLE_LED_PIN, LOW);
  pinMode(BUFFER_LED_PIN, OUTPUT);
  digitalWrite(BUFFER_LED_PIN, LOW);
  pinMode(MOTOR_LED, OUTPUT);

  // Initialize and Setup IMU
  if (!IMU.begin()) {  //Serial.println("Failed to initialize IMU!");
    while (1) {
      digitalWrite(RED, HIGH);
      delay(300);
      digitalWrite(RED, LOW);
      delay(300);
      digitalWrite(RED, HIGH);
      delay(80);
      digitalWrite(RED, LOW);
      delay(80);
      digitalWrite(RED, HIGH);
      delay(80);
      digitalWrite(RED, LOW);
      delay(300);
    }
  }

  // Accelerometer code UPDATED 25.01.2022 to the enclosure with haptic driver, motor, and battery
  IMU.setAccelFS(2);
  IMU.setAccelODR(4);
  IMU.setAccelOffset(-0.020568, -0.017173, -0.021025);
  IMU.setAccelSlope(1.001473, 0.994344, 1.008665);

  // Gyroscope code
  IMU.setGyroFS(1);
  IMU.setGyroODR(4);
  IMU.setGyroOffset(0.778946, -0.252731, -0.057968);
  IMU.setGyroSlope(1.186009, 1.148361, 1.121111);

  // Magnetometer code
  IMU.setMagnetFS(0);
  IMU.setMagnetODR(8);
  IMU.setMagnetOffset(-35.944824, -79.736938, 13.367920);
  IMU.setMagnetSlope(1.197732, 1.230049, 1.217827);

  IMU.gyroUnit = DEGREEPERSECOND;


  if (!setupBleMode()) {
    //Serial.println( "Failed to initialize BLE!" );
    while (1) {
      digitalWrite(RED, LOW);
      delay(200);
      digitalWrite(RED, HIGH);
      delay(200);
      digitalWrite(BLUE, LOW);
      delay(200);
      digitalWrite(BLUE, HIGH);
      delay(200);
      digitalWrite(RED, LOW);
      delay(200);
      digitalWrite(RED, HIGH);
      delay(200);
    }
  } else {
    //Serial.println( "BLE initialized. Waiting for clients to connect." );
    digitalWrite(GREEN, LOW);
    delay(500);
    digitalWrite(GREEN, HIGH);
    digitalWrite(BLUE, LOW);
    delay(500);
    digitalWrite(BLUE, HIGH);
  }
}

//----------------------------------------------------------------------------------------------------------------------
// Variable Declarations
//----------------------------------------------------------------------------------------------------------------------

//For HS Detection HS=heel-strike
float preVal = 0;
float curVal = 0;

//Gait phase markers
bool MSDetected = false;
bool ZeroCrossP2N = false;  //Crossing from Positive sign to Negative
bool HSDetected = false;
bool firstGyro = true;
unsigned long previousMillis = 0;

//For timing considerations
bool firstHS = true;
bool secondHS = true;
unsigned long firstMillis;
unsigned long newMillis;
unsigned long oldMillis;

//Vibration Task
bool readyToVibrate = false;
bool vibrationPhase = false;
unsigned long currVibeMillis;
unsigned long prevVibeMillis;

// HS detection
unsigned long currDetectMillis;
unsigned long prevDetectMillis;
int sampleCounter = 0;

//MS detection MS=mid-swing
float MS_THRESHOLD = INITIAL_MS_THRESHOLD;  //Minimum value that only above it a mid-swing might be detected. It prevents false positive
float peakValue;
bool checkForPeak = true;
bool setMSThreshold = false;


//For moving average
static int AVG_ARRAY_LENGTH = 3;
unsigned long stepLengthAverage = 0;

//----------------------------------------------------------------------------------------------------------------------
//  Loop
//----------------------------------------------------------------------------------------------------------------------

void loop() {
  bleTask();
  sensorTask();
  sendDataTask(RECORDING_LENGTH_SECONDS);  //UPDATE13/12/2021:This task is relevant only when we want to resend the data AKA REPLAY 03.
  vibrationTask();
}

//----------------------------------------------------------------------------------------------------------------------
//  BLE Task
//----------------------------------------------------------------------------------------------------------------------

void bleTask() {
  const uint32_t BLE_UPDATE_INTERVAL = 10;
  static uint32_t previousMillis = 0;

  uint32_t currentMillis = millis();
  if (currentMillis - previousMillis >= BLE_UPDATE_INTERVAL) {
    previousMillis = currentMillis;
    BLE.poll();
  }

  if (sensorDataUpdated) {
    sensorDataUpdated = false;
    sensorCharacteristic.writeValue(sensorData.bytes, sizeof sensorData.bytes);
  }
}

enum CMD_TYPE { CMD_RESET_BUFFER = 1,
                CMD_RECORD,
                CMD_REPLAY
                //                CMD_DISCOPARTY #Respect Omri, NB
};


void controlCharacteristicWrittenHandler(BLEDevice central, BLECharacteristic bleCharacteristic) {
  //Serial.print( "BLE characteristic written. UUID: " );
  //Serial.println( bleCharacteristic.uuid() );

  uint8_t cmd;
  if (bleCharacteristic.uuid() == (const char*)BLE_UUID_SENSOR_CONTROL) {
    bleCharacteristic.readValue(cmd);
    ////Serial.print( " CMD: " );
    ////Serial.println( cmd, HEX );
    switch (cmd) {
      case CMD_RESET_BUFFER:
        dataBuffer.index = 0;
        dataBuffer.state = BUFFER_STATE_READY;
        digitalWrite(BUFFER_LED_PIN, LOW);
        ////Serial.println( " Reset" );
        break;
      case CMD_RECORD:
        dataBuffer.index = 0;
        dataBuffer.state = BUFFER_STATE_RECORD;
        digitalWrite(BUFFER_LED_PIN, LOW);
        sensorActive = true;
        ////Serial.println( " Record" );
        firstHS = true;
        secondHS = true;
        break;
      case CMD_REPLAY:
        dataBuffer.index = 0;
        dataBuffer.state = BUFFER_STATE_REPLAY;
        digitalWrite(BUFFER_LED_PIN, LOW);
        sensorActive = false;
        ////Serial.println( " Replay" );
        break;

      default:
        //Serial.println( " unknown" );
        break;
    }
  }
}
//----------------------------------------------------------------------------------------------------------------------
//  Sensor Task
//----------------------------------------------------------------------------------------------------------------------

void sensorTask() {
  const uint32_t SENSOR_SAMPLING_INTERVAL = 10;  //Update 13.09.2022: Affect the Sample rate. 10 gives 95 in recording 03. 15 gives 60Hz. 7 gives noisy/spikey 105Hz.
                                                 //For some reason, it does not record well on my phone for some parameters.
  static uint32_t previousMillis = 0;


  if (!sensorActive) {  //This statement is true if we are in mode 03, meaning "replay" data and not measuring.
    return;
  }

  uint32_t currentMillis = millis();
  if (currentMillis - previousMillis >= SENSOR_SAMPLING_INTERVAL) {
    previousMillis = currentMillis;

    float gyroX, gyroY, gyroZ;

    if (IMU.gyroAvailable()) {
      IMU.readGyro(gyroX, gyroY, gyroZ);
      HeelStrikeDetect(gyroY);
      firstGyro = false;
      int32_t gyroYint = (int32_t)round(gyroY * 1000000);  //I chose a factor of 1 million to get a good compromise between digits on either side of the dot. You can choose another factor depending on your value range.
      sensorData.values.position = gyroYint;
      sensorData.values.time = currentMillis;
    }

#ifdef REVERSE_BYTE_ORDER
    sensorData.values.position = __REV(sensorData.values.position);
    sensorData.values.time = __REV(sensorData.values.time);
#endif

    //        sensorDataUpdated = true;  //Used to be line 356. uncomment to record when sending 02!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    if (dataBuffer.state == BUFFER_STATE_RECORD)  //See if the buffer is ready to record, meaning 02 was sent.
    {
      dataBuffer.values[dataBuffer.index] = sensorData.values;
      dataBuffer.index = (dataBuffer.index + 1) % DATA_BUFFER_SIZE;
      if (dataBuffer.index == 0) {
        dataBuffer.state = BUFFER_STATE_FULL;
        digitalWrite(BUFFER_LED_PIN, HIGH);
        sensorActive = false;
        //Serial.println( "Recording done" );
      }
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
//  Send Data Task
//----------------------------------------------------------------------------------------------------------------------

int32_t sendDataTask(uint32_t interval)  //This task is only for when we want to resend the data AKA REPLAY
{
  const uint32_t SEND_DELAY_MS = 5000;  //Wait 5 sec from when REPLAY was asked to when it is republished
  static uint32_t previousMillis = 0;
  static bool startUpDelay = true;

  if (dataBuffer.state != BUFFER_STATE_REPLAY) {  //See if the buffer is ready to replay data, meaning 03 was sent. If not then return to loop.
    return -1;
  }

  if (dataBuffer.index == 0) {
    if (startUpDelay) {
      startUpDelay = false;
      previousMillis = millis();
      // Serial.println( "Get ready for send" );
      return -1;
    }
    uint32_t currentMillis = millis();
    if (currentMillis - previousMillis < SEND_DELAY_MS) {
      return -1;
    } else {
      startUpDelay = true;
      // Serial.println( "Sending data" );
    }
  }

  uint32_t currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    sensorData.values = dataBuffer.values[dataBuffer.index];
    sensorDataUpdated = true;
    dataBuffer.index = (dataBuffer.index + 1) % DATA_BUFFER_SIZE;
    if (dataBuffer.index == 0) {
      dataBuffer.state = BUFFER_STATE_READY;
      digitalWrite(BUFFER_LED_PIN, HIGH);
      // Serial.println( "Sending done" );
    }
    return dataBuffer.index;
  }
  return -1;
}

//----------------------------------------------------------------------------------------------------------------------
// Vibration Task
//----------------------------------------------------------------------------------------------------------------------

void vibrationTask() {
  if (readyToVibrate) {
    currVibeMillis = millis();

    if (currVibeMillis - prevVibeMillis >= stepLengthAvg.get() * VIBRATION_START_PERCENTAGE_MOTOR1 * 0.01) {  //Delay before vibration starts

      digitalWrite(MOTOR_LED, HIGH);  //LED on
      vibrationPhase = true;
      readyToVibrate = false;
    }
  }

  if (vibrationPhase) {
    currVibeMillis = millis();

    if (currVibeMillis - prevVibeMillis >= stepLengthAvg.get() * (VIBRATION_END_PERCENTAGE - VIBRATION_START_PERCENTAGE_MOTOR1) * 0.01) {  //Duration of vibration until stop

      digitalWrite(MOTOR_LED, LOW);  //LED off
      vibrationPhase = false;
      prevVibeMillis = currVibeMillis;
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
// Helper Methods / Handlers / Functions
//----------------------------------------------------------------------------------------------------------------------

void HeelStrikeDetect(float val) {

  //Detection delay to ensure no false positives after heel-strike. Depending on the sample rate, this value should be high for a high sample rate.
  if (sampleCounter <= SAMPLES_TO_IGNORE_AFTER_HS) {
    sampleCounter++;
    return;
  }

  if (firstGyro) {
    curVal = val;
  } else {
    preVal = curVal;
    curVal = val;

    if (curVal >= MS_THRESHOLD) {  //Checks mid-swing occurring
      //Serial.print("MS_THRESHOLD: ");
      //Serial.println(MS_THRESHOLD);
      if (preVal >= curVal && checkForPeak == true) {  //Finds mid-swing value and adds to avg
        peakValue = preVal;
        //Serial.print("--------PEAK VALUE: ");
        //Serial.println(peakValue);
        peakValueAvg.push(peakValue);
        //Serial.print("--------AVG VALUE: ");
        //Serial.println(peakValueAvg.get());
        checkForPeak = false;
      }
      if (setMSThreshold == true) {  //Sets threshold to moving avg after second step
        //Serial.println("SET THRESH TO AVG");
        MS_THRESHOLD = MS_THRESHOLD_PCT_OF_PEAK * peakValueAvg.get();
      }

      MSDetected = true;  //For creating a mid-swing event-dependent feedback, insert a statement here to trigger the desired feedback.
      ZeroCrossP2N = false;
    }

    if (MSDetected == true && preVal >= 0 && curVal <= 0) {  //Checks zero cross
      ZeroCrossP2N = true;
      MSDetected = false;
    }

    if (ZeroCrossP2N == true) {
      if (preVal < curVal) {  //After mid-swing and slope goes from positive to negative (ZeroCrossP2N true) we know a heel-strike is coming. When the preVal is smaller than curVal, heel-strike has just happened on the prev sample.
        MSDetected = false;
        ZeroCrossP2N = false;
        sampleCounter = 0;

        if (firstHS) {
          firstMillis = millis();
          firstHS = false;
        } else if (secondHS && firstMillis - newMillis > 350) {  //Ensures that the first point added to the average is not too short
          newMillis = millis();
          stepLengthAvg.push(newMillis - firstMillis);
          stepLengthAverage = newMillis - firstMillis;  //Not average yet, updates after the third step
          //Serial.print("Step Length (ms): ");
          //Serial.println(newMillis - firstMillis);
          secondHS = false;
        } else {
          oldMillis = newMillis;  //Sets old to the previous new time
          newMillis = millis();

          if (newMillis - oldMillis <= STEP_LENGTH_THRESHOLD * stepLengthAverage) {  //Step must be under 150% of prev step to updated step length avg.
            //Serial.print("Step Length (ms): ");
            //Serial.println(newMillis - oldMillis);
            stepLengthAvg.push(newMillis - oldMillis);
            stepLengthAverage = stepLengthAvg.get();
          } else {
          }
          setMSThreshold = true;  //Sets the threshold to moving MS avg after the second step
        }

        readyToVibrate = true;
        checkForPeak = true;
        prevVibeMillis = millis();
      }
    }
  }
}



//BLE helpers
void blePeripheralConnectHandler(BLEDevice central) {
  digitalWrite(BLE_LED_PIN, HIGH);
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, HIGH);
}


void blePeripheralDisconnectHandler(BLEDevice central) {
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, LOW);
  digitalWrite(BLE_LED_PIN, LOW);
}


bool setupBleMode() {
  if (!BLE.begin()) {
    digitalWrite(RED, LOW);
    delay(500);
    digitalWrite(RED, HIGH);
    delay(500);
    return false;
  }

  // Set advertised local name and service UUID
  BLE.setDeviceName(BLE_DEVICE_NAME);
  BLE.setLocalName(BLE_LOCAL_NAME);
  BLE.setAdvertisedService(sensorService);

  // BLE add characteristics
  sensorService.addCharacteristic(sensorCharacteristic);
  sensorService.addCharacteristic(controlCharacteristic);

  // BLE add descriptors
  controlCharacteristic.addDescriptor(controlDescriptor);

  // Add service
  BLE.addService(sensorService);

  // Set the initial value for the characteristic
  sensorCharacteristic.writeValue(sensorData.bytes, sizeof sensorData.bytes);
  controlCharacteristic.writeValue(0);

  // Set BLE event handlers
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // Set service and characteristic-specific event handlers
  controlCharacteristic.setEventHandler(BLEWritten, controlCharacteristicWrittenHandler);

  // Start advertising
  BLE.advertise();

  return true;
}