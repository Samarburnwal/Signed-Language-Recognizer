#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Kalman.h"

#define MULTIPLEXER_ADDR 0x70  // PCA9548A default I2C address

Adafruit_MPU6050 mpu;

// Kalman filters for 5 sensors (roll and pitch only, yaw estimated via gyro)
Kalman kalmanRoll[5];
Kalman kalmanPitch[5];

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float roll, pitch, yaw;

unsigned long prevTime[5] = {0};

void selectChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(MULTIPLEXER_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

bool initMPU(uint8_t channel) {
  selectChannel(channel);
  if (!mpu.begin()) {
//    Serial.print("Failed to find MPU6050 on channel ");
//    Serial.println(channel);
    return false;
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  for (int i = 0; i < 5; i++) {
    if (initMPU(i)) {
//      Serial.print("MPU6050 #");
//      Serial.print(i);
//      Serial.println(" initialized.");
    }
  }
}

void loop() {
  for (int i = 0; i < 5; i++) {
    selectChannel(i);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    accX = a.acceleration.x;
    accY = a.acceleration.y;
    accZ = a.acceleration.z;

    gyroX = g.gyro.x * 180/PI;
    gyroY = g.gyro.y * 180/PI;
    gyroZ = g.gyro.z * 180/PI;

    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime[i]) / 1000.0;
    prevTime[i] = currentTime;

    float newRoll  = atan2(accY, accZ) * RAD_TO_DEG;
    float newPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

    if (dt > 0) {
      roll  = kalmanRoll[i].getAngle(newRoll, gyroX, dt);
      pitch = kalmanPitch[i].getAngle(newPitch, gyroY, dt);
      yaw += gyroZ * dt; // simple gyro integration (subject to drift)
    }

    //Serial.print("MPU");
    //Serial.print(i);
    //Serial.print(" -> Roll: ");
    Serial.print(roll);
    Serial.print(" ");
    //Serial.print(" | Pitch: ");
    Serial.print(pitch);
    //Serial.print(" | Yaw: ");
//    Serial.print(" ");
    //Serial.print(yaw);
    Serial.print(" ");
  }
  Serial.println();
  delay(50); // Small delay for readability
}
