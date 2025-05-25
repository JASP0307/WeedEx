// main.ino o main.cpp
#include <Arduino.h>
#include "SingleMotor.h"

// Pines de los motores
#define L_PWM1 5
#define L_PWM2 6
#define L_ENC_A 2  // interrupt 0
#define L_ENC_B 22

#define R_PWM1 9
#define R_PWM2 10
#define R_ENC_A 3  // interrupt 1
#define R_ENC_B 23

// Instancias de motores
SingleMotor motorIzq(L_PWM1, L_PWM2, L_ENC_A, L_ENC_B);
SingleMotor motorDer(R_PWM1, R_PWM2, R_ENC_A, R_ENC_B);

// Funciones ISR
void ISR_EncIzq() { motorIzq.handleEncoderTick(); }
void ISR_EncDer() { motorDer.handleEncoderTick(); }

// Tarea FreeRTOS para actualizar motores
void TaskMotorControl(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    motorIzq.update();
    motorDer.update();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void setup() {
  Serial.begin(9600);
  motorIzq.init();
  motorDer.init();

  attachInterrupt(digitalPinToInterrupt(L_ENC_A), ISR_EncIzq, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), ISR_EncDer, RISING);

  motorIzq.setTargetSpeed(50); // RPM objetivo
  motorDer.setTargetSpeed(50);

  xTaskCreate(
    TaskMotorControl,
    "MotorControl",
    2048,
    NULL,
    1,
    NULL
  );
}

void loop() {
  // No usado si todo corre con FreeRTOS
}
