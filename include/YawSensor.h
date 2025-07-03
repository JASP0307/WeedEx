#ifndef YAWSENSOR_H
#define YAWSENSOR_H

#include <Wire.h>

class YawSensor {
private:
  float rateYaw;
  float calibrationYaw;
  float yawAngle;
  unsigned long lastUpdateTime; // Cambio: usar para tiempo real

  void readGyroYaw();
  void calibrateGyro();

public:
  YawSensor();
  void begin();
  void update(float deltaTime);
  float getYaw() const;
  void resetYaw() { yawAngle = 0.0; } // Función adicional útil
};

#endif