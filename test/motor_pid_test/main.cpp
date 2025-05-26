// pio run -t custom_upload_plot

#include <Arduino.h>
#include "Motor_Control.h"
#include <Arduino_FreeRTOS.h>

// Pines motor izquierdo
#define L_PWM1 5
#define L_PWM2 6
#define L_ENC_A 2
#define L_ENC_B 22

// Instancia del motor izquierdo
Motor_Control motorIzq(L_PWM1, L_PWM2, L_ENC_A, L_ENC_B);

// ISR para el encoder del motor izquierdo
void ISR_EncIzq() { motorIzq.handleEncoderTick(); }

// Tarea FreeRTOS para actualizar el motor
void TaskMotorUpdate(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    motorIzq.update();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Tarea para variar el setpoint cada 3 segundos y monitorear respuesta
void TaskPIDTest(void *pvParameters) {
  (void) pvParameters;
  float setpoints[] = {30, 60, 90, 45, 0};
  const int n = sizeof(setpoints)/sizeof(setpoints[0]);
  int index = 0;
  for (;;) {
    motorIzq.setTargetSpeed(setpoints[index]);
    index = (index + 1) % n;
    for (int i = 0; i < 30; i++) { // 30 * 100ms = 3s
      Serial.print(setpoints[index]);
      Serial.print(",");
      Serial.println(motorIzq.getCurrentRPM());
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

void setup() {
  Serial.begin(9600);
  motorIzq.init();
  motorIzq.setTunings(1.2, 0.5, 0.1); // Ajusta según tu sistema

  attachInterrupt(digitalPinToInterrupt(L_ENC_A), ISR_EncIzq, RISING);

  xTaskCreate(TaskMotorUpdate, "MotorUpdate", 2048, NULL, 1, NULL);
  xTaskCreate(TaskPIDTest, "PIDTest", 2048, NULL, 1, NULL);
}

void loop() {
  // No utilizado
}