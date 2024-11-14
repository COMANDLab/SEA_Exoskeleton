/* 
Author: Hailey Levan
Last Update: 7/8/24

This code utilizes 2 Adafruit LSM6DS3TR-C 6-DoF Accel + Gyro IMUs. It records the gryoscope readings in the xyz directions and prints to the serial
monitor so the data can be copied into a file for later processing in MATLAB.
*/

#include <Wire.h>
#include <Adafruit_LSM6DS3TRC.h>

// Create instant for the accelerometer
Adafruit_LSM6DS3TRC forearm; // forearm (dynamic)
Adafruit_LSM6DS3TRC upperarm; //upperarm (static)

// Variables to store angular velocity
float omegaX_forearm = 0.0, omegaY_forearm = 0.0, omegaZ_forearm = 0.0;

float omegaX_upper = 0.0, omegaY_upper = 0.0, omegaZ_upper = 0.0;

// Time variables
unsigned long time = 0;
float time_sec = 0.0;

void setup() {
  // Initialize serial communication at 115200 baud rate
  Serial.begin(115200);
  while (!Serial); // Wait for Serial to be ready

  // Initialize the I2C bus
  Wire.begin();

  // Initialize IMU 1
  if (!forearm.begin_I2C(0x6A)) {
    Serial.println("Failed to find Forearm IMU (0x6A)");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Forearm IMU (0x6A) initialized");

  // Initialize IMU 2
  if (!upperarm.begin_I2C(0x6B)) {
    Serial.println("Failed to find Upperarm IMU (0x6B)");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Upperarm IMU (0x6B) initialized");

   // Setting up the sensors
   forearm.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
   forearm.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
   forearm.setAccelDataRate(LSM6DS_RATE_104_HZ);
   forearm.setGyroDataRate(LSM6DS_RATE_104_HZ);

   upperarm.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
   upperarm.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
   upperarm.setAccelDataRate(LSM6DS_RATE_104_HZ);
   upperarm.setGyroDataRate(LSM6DS_RATE_104_HZ);
}

void loop() {
   // Start time recording
   time = millis();

   time_sec = time/1000.000; //Converts from ms to s 

   // Read data from IMU 1 (moving IMU)
   sensors_event_t accel_forearm, gyro_forearm, temp_forearm;
   forearm.getEvent(&accel_forearm, &gyro_forearm, &temp_forearm);

   // Read data from IMU 2 (fixed IMU)
   sensors_event_t accel_upper, gyro_upper, temp_upper;
   upperarm.getEvent(&accel_upper, &gyro_upper, &temp_upper);

   // Store data from gyros into defined variables
   omegaX_forearm = gyro_forearm.gyro.x;
   omegaY_forearm = gyro_forearm.gyro.y;
   omegaZ_forearm = gyro_forearm.gyro.z;

   omegaX_upper = gyro_upper.gyro.x;
   omegaY_upper = gyro_upper.gyro.y;
   omegaZ_upper = gyro_upper.gyro.z;

   /* Print data to serial monitor... printed in order: 
   time, ang velo X forearm, ang velo Y forearm, ang velo Z forearm, ang velo X upperarm, ang velo Y upperarm, ang velo Z upperarm */
   Serial.print(time_sec); Serial.print(" , ");
   Serial.print(omegaX_forearm); Serial.print(" , ");
   Serial.print(omegaY_forearm); Serial.print(" , ");
   Serial.print(omegaZ_forearm); Serial.print(" , ");
   Serial.print(omegaX_upper); Serial.print(" , ");
   Serial.print(omegaY_upper); Serial.print(" , ");
   Serial.println(omegaZ_upper);

   delay(0.01); // Delay seconds before next read
}