#include "YawSensor.h"
#include <Arduino.h>

YawSensor::YawSensor() {
  rateYaw = 0.0;
  calibrationYaw = 0.0;
  yawAngle = 0.0;
  lastUpdateTime = 0;
}

void YawSensor::begin() {
  Wire.begin();
  delay(250);

  // Wake up MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Configurar filtros y rangos UNA SOLA VEZ
  Wire.beginTransmission(0x68);
  Wire.write(0x1A); 
  Wire.write(0x05); // Filtro pasa-bajos
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x10); // Rango del giroscopio ±1000°/s
  Wire.endTransmission();

  calibrateGyro();
  lastUpdateTime = micros(); // Inicializar tiempo
}

void YawSensor::calibrateGyro() {
  calibrationYaw = 0.0;

  for (int i = 0; i < 2000; i++) {
    readGyroYaw();
    calibrationYaw += rateYaw;
    delay(1);
  }

  calibrationYaw /= 2000.0;
}

void YawSensor::readGyroYaw() {
  // Leer datos del giroscopio (solo Z)
  Wire.beginTransmission(0x68);
  Wire.write(0x47); // Registro directo de GyroZ (0x47-0x48)
  Wire.endTransmission();
  Wire.requestFrom(0x68, 2); // Solo leer 2 bytes para GyroZ

  if (Wire.available() >= 2) {
    int16_t gyroZ = Wire.read() << 8 | Wire.read();
    rateYaw = (float)gyroZ / 32.8; // Conversión para rango ±1000°/s
  }
}

void YawSensor::update(float deltaTime) {
    readGyroYaw();
    rateYaw -= calibrationYaw;
    yawAngle += rateYaw * deltaTime;
}

float YawSensor::getYaw() const {
  return yawAngle;
}