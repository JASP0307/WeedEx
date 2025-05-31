// Máquina de Estados para Arduino Mega - Robot de Desmalezado
// Comunicación Serial con MiniPC - Versión Dual Motor
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include "MotorModule.h"

// Definición de estados
enum RobotState {
  IDLE,
  NAVIGATING,
  WEED_FOUND,
  ACTUATING,
  ERROR_STATE
};

volatile RobotState currentState = IDLE;

// Crear motores con pines de interrupción diferentes
// Motor izquierdo: encoder en pines 2,4
// Motor derecho: encoder en pines 3,5
MotorModule motorIzq(10, 11, 22, 23, 2, 4, 360, 50);
MotorModule motorDer(12, 13, 24, 25, 3, 5, 360, 50);

// Prototipos de tareas
void TaskFSM(void *pvParameters);
void TaskLocomotion(void *pvParameters);
void TaskActuation(void *pvParameters);
void TaskSensors(void *pvParameters);
void TaskBattery(void *pvParameters);
void TaskComms(void *pvParameters);

void setup() {
  Serial.begin(115200);
  
  // Inicializar motores
  Serial.println("Inicializando motores...");
  
  motorIzq.inicializar();
  motorDer.inicializar();
  
  // Configurar PID para ambos motores
  motorIzq.configurarPID(1.5, 3.0, 0.01);
  motorDer.configurarPID(1.5, 3.0, 0.01);
  
  // Establecer velocidades objetivo
  motorIzq.establecerSetpoint(30.0);
  motorDer.establecerSetpoint(30.0);
  
  // Activar control PID
  motorIzq.activarPID(true);
  motorDer.activarPID(true);
  
  Serial.println("Motores inicializados");
  
  // Crear tareas
  xTaskCreate(TaskFSM,        "FSM",        128, NULL, 1, NULL);
  xTaskCreate(TaskLocomotion, "Locomotion", 256, NULL, 2, NULL);
  xTaskCreate(TaskActuation,  "Actuation",  128, NULL, 2, NULL);
  xTaskCreate(TaskSensors,    "Sensors",    128, NULL, 2, NULL);
  xTaskCreate(TaskBattery,    "Battery",    128, NULL, 2, NULL);
  xTaskCreate(TaskComms,      "Comms",      128, NULL, 2, NULL);
  
  Serial.println("Tareas FreeRTOS creadas");
}

void loop() {
  // No se usa con FreeRTOS
}

// FSM principal
void TaskFSM(void *pvParameters) {
  (void) pvParameters;
  
  for (;;) {
    switch (currentState) {
      case IDLE:
        // Esperar comando de navegación
        // Detener motores si están en movimiento
        if (motorIzq.pidEstaActivo()) {
          motorIzq.activarPID(false);
          motorDer.activarPID(false);
          motorIzq.establecerSetpoint(0);
          motorDer.establecerSetpoint(0);
        }
        break;

      case NAVIGATING:
        // Activar motores para navegación
        if (!motorIzq.pidEstaActivo()) {
          motorIzq.activarPID(true);
          motorDer.activarPID(true);
        }
        motorIzq.establecerSetpoint(30.0);
        motorDer.establecerSetpoint(30.0);
        break;

      case WEED_FOUND:
        // Detener motores y cambiar a actuación
        motorIzq.establecerSetpoint(0);
        motorDer.establecerSetpoint(0);
        currentState = ACTUATING;
        break;

      case ACTUATING:
        // Esperar a que actuador confirme finalización
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Simular actuación
        currentState = NAVIGATING;
        break;

      case ERROR_STATE:
        // Detener todos los motores
        motorIzq.habilitarMotor(false);
        motorDer.habilitarMotor(false);
        break;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void TaskLocomotion(void *pvParameters) {
  (void) pvParameters;
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  for (;;) {
    // Actualizar ambos motores
    bool izqActualizado = motorIzq.actualizar();
    bool derActualizado = motorDer.actualizar();
    
    // Imprimir datos solo si algún motor se actualizó
    if (izqActualizado || derActualizado) {
      // Formato: MotorID,Setpoint,RPM,PWM
      if (izqActualizado) {
        //motorIzq.imprimirCSV();
      }
      if (derActualizado) {
        //motorDer.imprimirCSV();
      }
    }
    
    // Usar vTaskDelayUntil para mantener frecuencia constante
    vTaskDelayUntil(&xLastWakeTime, 20 / portTICK_PERIOD_MS); // 50Hz
  }
}

void TaskActuation(void *pvParameters) {
  (void) pvParameters;
  
  for (;;) {
    // Activar pistón o láser si se requiere
    // TODO: Implementar control de actuadores
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void TaskSensors(void *pvParameters) {
  (void) pvParameters;
  
  for (;;) {
    // Leer sensores de seguridad
    // TODO: Implementar lectura de sensores
    
    // Ejemplo: detectar obstáculo
    // if (sensorObstaculo.leer() < DISTANCIA_MINIMA) {
    //     currentState = ERROR_STATE;
    // }
    
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void TaskBattery(void *pvParameters) {
  (void) pvParameters;
  
  for (;;) {
    // Leer voltaje y chequear estado
    // TODO: Implementar monitoreo de batería
    
    // Ejemplo:
    // float voltaje = analogRead(PIN_BATERIA) * (5.0/1023.0) * FACTOR_DIVISION;
    // if (voltaje < VOLTAJE_MINIMO) {
    //     currentState = ERROR_STATE;
    // }
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void TaskComms(void *pvParameters) {
  (void) pvParameters;
  
  for (;;) {
    // Comunicación Serial con Mini PC
    if (Serial.available()) {
      String msg = Serial.readStringUntil('\n');
      msg.trim();
      
      if (msg.startsWith("CMD,NAVIGATE")) {
        currentState = NAVIGATING;
        Serial.println("ACK,NAVIGATE");
      } 
      else if (msg.startsWith("CMD,STOP")) {
        currentState = IDLE;
        Serial.println("ACK,STOP");
      }
      else if (msg.startsWith("CMD,SPEED,")) {
        // Formato: CMD,SPEED,LEFT,RIGHT
        int firstComma = msg.indexOf(',', 10);
        int secondComma = msg.indexOf(',', firstComma + 1);
        
        if (firstComma != -1 && secondComma != -1) {
          float speedLeft = msg.substring(firstComma + 1, secondComma).toFloat();
          float speedRight = msg.substring(secondComma + 1).toFloat();
          
          motorIzq.establecerSetpoint(speedLeft);
          motorDer.establecerSetpoint(speedRight);
          
          Serial.println("ACK,SPEED");
        }
      }
      else if (msg.startsWith("CMD,PID,")) {
        // Formato: CMD,PID,KP,KI,KD
        int firstComma = msg.indexOf(',', 8);
        int secondComma = msg.indexOf(',', firstComma + 1);
        int thirdComma = msg.indexOf(',', secondComma + 1);
        
        if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
          float kp = msg.substring(firstComma + 1, secondComma).toFloat();
          float ki = msg.substring(secondComma + 1, thirdComma).toFloat();
          float kd = msg.substring(thirdComma + 1).toFloat();
          
          motorIzq.configurarPID(kp, ki, kd);
          motorDer.configurarPID(kp, ki, kd);
          
          Serial.println("ACK,PID");
        }
      }
      else if (msg.startsWith("CMD,STATUS")) {
        // Enviar estado detallado
        Serial.print("STATUS,");
        Serial.print(currentState);
        Serial.print(",L:");
        Serial.print(motorIzq.obtenerVelocidadActual());
        Serial.print(",R:");
        Serial.println(motorDer.obtenerVelocidadActual());
      }
    }
    
    // Enviar heartbeat periódico
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 5000) {
      Serial.print("HEARTBEAT,");
      Serial.print(currentState);
      Serial.print(",");
      //Serial.print(xPortGetFreeHeapSize());
      Serial.println();
      lastHeartbeat = millis();
    }
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}