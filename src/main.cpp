// Máquina de Estados para Arduino Mega - Robot de Desmalezado

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include "MotorModule.h"
#include "ServoModule.h"
#include "YawSensor.h"
#include "Ultrasonico.h"
#include "PinOut.h"

// Definición de estados
enum RobotState {
  IDLE,
  NAVIGATING,
  WEED_FOUND,
  ACTUATING,
  ERROR_STATE,
  LOW_BATTERY,
  OBSTACLE
};

typedef enum {
  EVENT_NONE,
  EVENT_NAVIGATE,
  EVENT_STOP,
  EVENT_OBSTACLE,
  EVENT_LOW_BATTERY,
  EVENT_ERROR,
  EVENT_RESUME
} FSMEvent;

// Handles globales
QueueHandle_t fsmQueue;
SemaphoreHandle_t stateMutex;

// Estado protegido por mutex
volatile RobotState currentState = IDLE;

// Crear motores con pines de interrupción diferentes
MotorModule motorIzq(Pinout::Locomocion::MotorIzquierdo::rPWM,
                     Pinout::Locomocion::MotorIzquierdo::lPWM,
                     Pinout::Locomocion::MotorIzquierdo::rEN,
                     Pinout::Locomocion::MotorIzquierdo::lEN,
                     Pinout::Locomocion::MotorIzquierdo::ENC_A,
                     Pinout::Locomocion::MotorIzquierdo::ENC_B,
                     360, 50);
                     
MotorModule motorDer(Pinout::Locomocion::MotorDerecho::rPWM,
                     Pinout::Locomocion::MotorDerecho::lPWM,
                     Pinout::Locomocion::MotorDerecho::rEN,
                     Pinout::Locomocion::MotorDerecho::lEN,
                     Pinout::Locomocion::MotorDerecho::ENC_A,
                     Pinout::Locomocion::MotorDerecho::ENC_B,
                     360, 50);

ServoModule SERV_01(Pinout::BrazoDelta::SERVO_1);
ServoModule SERV_02(Pinout::BrazoDelta::SERVO_2);
ServoModule SERV_03(Pinout::BrazoDelta::SERVO_3);

YawSensor yawSensor;
Ultrasonico UltrDer(Pinout::Sensores::Ultra_Der::TRIG, Pinout::Sensores::Ultra_Der::ECHO);
Ultrasonico UltrIzq(Pinout::Sensores::Ultra_Izq::TRIG, Pinout::Sensores::Ultra_Izq::ECHO);
Ultrasonico UltrFront(Pinout::Sensores::Ultra_Front::TRIG, Pinout::Sensores::Ultra_Front::ECHO);

Ultrasonico sensores[] = {UltrFront}; 
const int NUM_SENSORES = sizeof(sensores) / sizeof(sensores[0]);

const uint8_t DISTANCIA_MINIMA = 5;

// Prototipos de tareas
void TaskFSM(void *pvParameters);
void TaskLocomotion(void *pvParameters);
void TaskActuation(void *pvParameters);
void TaskSensors(void *pvParameters);
void TaskBattery(void *pvParameters);
void TaskComms(void *pvParameters);
void TaskServoControl(void *pvParameters);

// Funciones auxiliares
RobotState getState();
void setState(RobotState newState);

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando sistema...");
  
  // Crear mutex para proteger estado
  stateMutex = xSemaphoreCreateMutex();
  if (stateMutex == NULL) {
    Serial.println("Error: No se pudo crear mutex de estado.");
    while (1);
  }
  
  // Crear cola FSM
  fsmQueue = xQueueCreate(10, sizeof(FSMEvent));
  if (fsmQueue == NULL) {
    Serial.println("Error: No se pudo crear la cola FSM.");
    while (1);
  }

  // Inicializar motores
  Serial.println("Inicializando motores...");
  
  motorIzq.inicializar();
  motorDer.inicializar();
  
  // Configurar PID para ambos motores
  motorIzq.configurarPID(1.5, 3.0, 0.01);
  motorDer.configurarPID(1.5, 3.0, 0.01);
  
  // Establecer velocidades objetivo
  motorIzq.establecerSetpoint(0.0);
  motorDer.establecerSetpoint(0.0);
  
  // Activar control PID
  motorIzq.activarPID(true);
  motorDer.activarPID(true);
  SERV_01.begin();
  Serial.println("Motores inicializados");
  
  // Crear tareas
  xTaskCreate(TaskFSM,        "FSM",        512, NULL, 1, NULL);
  xTaskCreate(TaskLocomotion, "Locomotion", 256, NULL, 2, NULL);
  xTaskCreate(TaskActuation,  "Actuation",  128, NULL, 2, NULL);
  xTaskCreate(TaskSensors,    "Sensors",    256, NULL, 2, NULL);
  xTaskCreate(TaskBattery,    "Battery",    128, NULL, 2, NULL);
  xTaskCreate(TaskComms,      "Comms",      256, NULL, 2, NULL);
  xTaskCreate(TaskServoControl,  "ServoControl",  256, NULL, 1, NULL);
  
  Serial.println("Tareas FreeRTOS creadas");
}

void loop() {}

// Funciones thread-safe para estado
RobotState getState() {
  RobotState state;
  if (xSemaphoreTake(stateMutex, portMAX_DELAY)) {
    state = currentState;
    xSemaphoreGive(stateMutex);
  }
  return state;
}

void setState(RobotState newState) {
  if (xSemaphoreTake(stateMutex, portMAX_DELAY)) {
    currentState = newState;
    xSemaphoreGive(stateMutex);
  }
}

void TaskServoControl(void *pvParameters) {
  (void) pvParameters;

  vTaskDelay(pdMS_TO_TICKS(1000)); 
  
  SERV_01.begin();
  SERV_01.setTarget(120);

  for (;;) {
    SERV_01.update();
    vTaskDelay(pdMS_TO_TICKS(50)); 
  }
}

// FSM principal
void TaskFSM(void *pvParameters) {
  (void) pvParameters;

  FSMEvent receivedEvent;

  for (;;) {
    if (xQueueReceive(fsmQueue, &receivedEvent, portMAX_DELAY)) {
      RobotState state = getState();
      
      switch (state) {
        case IDLE:
          if (receivedEvent == EVENT_NAVIGATE) {
            // Asegurar que motores estén listos
            if (!motorIzq.pidEstaActivo()) {
              motorIzq.activarPID(true);
              motorDer.activarPID(true);
            }
            SERV_01.setTarget(30);
            setState(NAVIGATING);
            Serial.println("Estado: NAVIGATING");
          } else if (receivedEvent == EVENT_LOW_BATTERY) {
            setState(LOW_BATTERY);
            Serial.println("Estado: LOW_BATTERY");
          }
          break;

        case NAVIGATING:
          if (receivedEvent == EVENT_OBSTACLE) {
            setState(OBSTACLE);
            Serial.println("Estado: OBSTACLE");
          } else if (receivedEvent == EVENT_LOW_BATTERY) {
            setState(LOW_BATTERY);
            Serial.println("Estado: LOW_BATTERY");
          } else if (receivedEvent == EVENT_STOP) {
            // Detener motores gradualmente
            motorIzq.establecerSetpoint(0);
            motorDer.establecerSetpoint(0);
            SERV_01.setTarget(180); // Mover a 120 grados
            setState(IDLE);
            Serial.println("Estado: IDLE");
          }
          break;

        case WEED_FOUND:
          // Detener motores y cambiar a actuación
          motorIzq.establecerSetpoint(0);
          motorDer.establecerSetpoint(0);
          // Iniciar el movimiento del actuador (servo)
          Serial.println("Comando: Mover servo a posición de ataque (120°)");
          SERV_01.setTarget(30); // Mover a 120 grados
          
          //setState(ACTUATING);
          Serial.println("Estado: ACTUATING");
          break;

        case ACTUATING:
          // Esperar a que actuador confirme finalización
          vTaskDelay(pdMS_TO_TICKS(1000));
          // Volver a navegación después de actuar
          setState(NAVIGATING);
          Serial.println("Estado: NAVIGATING (post-actuación)");
          break;

        case ERROR_STATE:
          // Detener todos los motores inmediatamente
          motorIzq.habilitarMotor(false);
          motorDer.habilitarMotor(false);
          if (receivedEvent == EVENT_RESUME) {
            motorIzq.habilitarMotor(true);
            motorDer.habilitarMotor(true);
            setState(IDLE);
            Serial.println("Estado: IDLE (recuperado de error)");
          }
          break;
        
        case LOW_BATTERY:
          // Detener motores para conservar energía
          motorIzq.establecerSetpoint(0);
          motorDer.establecerSetpoint(0);
          if (receivedEvent == EVENT_RESUME) {
            setState(IDLE);
            Serial.println("Estado: IDLE (batería recuperada)");
          }
          break;
        
        case OBSTACLE:
          // Detener motores
          motorIzq.establecerSetpoint(0);
          motorDer.establecerSetpoint(0);
          if (receivedEvent == EVENT_RESUME) {
            setState(NAVIGATING);
            Serial.println("Estado: NAVIGATING (obstáculo evitado)");
          } else if (receivedEvent == EVENT_ERROR) {
            setState(ERROR_STATE);
            Serial.println("Estado: ERROR_STATE");
          }
          break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // Reducir delay para mejor respuesta
  }
}

void TaskLocomotion(void *pvParameters) {
  (void) pvParameters;
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  for (;;) {
    // Actualizar ambos motores
    bool izqActualizado = motorIzq.actualizar();
    bool derActualizado = motorDer.actualizar();
    /*
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
    */
    // Usar vTaskDelayUntil para mantener frecuencia constante
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20)); // 50Hz
  }
}

void TaskActuation(void *pvParameters) {
  (void) pvParameters;
  
  for (;;) {
    // Activar pistón o láser si se requiere
    // TODO: Implementar control de actuadores
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void TaskSensors(void *pvParameters) {
  (void) pvParameters;
  
  // Reducir el delay para mejor resolución temporal
  const TickType_t delayTicks = pdMS_TO_TICKS(50); // 50ms en lugar de 500ms
  
  // Inicializar el sensor
  yawSensor.begin();
  
  TickType_t lastWakeTime = xTaskGetTickCount();
  
  for (;;) {
    yawSensor.update();
    
    float yaw = yawSensor.getYaw();
    //Serial.print("Yaw Angle [°]: ");
    //Serial.println(yaw, 2); // 2 decimales

    for (int i = 0; i < NUM_SENSORES; i++) {
      if (sensores[i].distancia() < DISTANCIA_MINIMA) {
          FSMEvent e = EVENT_OBSTACLE;
          xQueueSend(fsmQueue, &e, 0);
          break;
      }
    }
    vTaskDelayUntil(&lastWakeTime, delayTicks);
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
    //     FSMEvent e = EVENT_LOW_BATTERY;
    //     xQueueSend(fsmQueue, &e, 0);
    // }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void TaskComms(void *pvParameters) {
  (void) pvParameters;
  
  TickType_t lastHeartbeat = xTaskGetTickCount();
  
  for (;;) {
    // Comunicación Serial con Mini PC
    if (Serial.available()) {
      String msg = Serial.readStringUntil('\n');
      msg.trim();
      
      if (msg.equals("CMD,NAVIGATE")) {
        FSMEvent e = EVENT_NAVIGATE;
        xQueueSend(fsmQueue, &e, 0);
        Serial.println("ACK,NAVIGATE");
      } 
      else if (msg.equals("CMD,STOP")) {
        FSMEvent e = EVENT_STOP;
        xQueueSend(fsmQueue, &e, 0);
        Serial.println("ACK,STOP");
      }
      else if (msg.equals("CMD,RESUME")) {
        FSMEvent e = EVENT_RESUME;
        xQueueSend(fsmQueue, &e, 0);
        Serial.println("ACK,RESUME");
      }
      else if (msg.startsWith("CMD,SPEED,")) {
        // Formato: CMD,SPEED,LEFT,RIGHT
        int firstComma = msg.indexOf(',', 10);
        int secondComma = msg.indexOf(',', firstComma + 1);
        
        if (firstComma > 10 && secondComma > firstComma) {
          String leftStr = msg.substring(10, firstComma);
          String rightStr = msg.substring(firstComma + 1, secondComma);
          
          float speedLeft = leftStr.toFloat();
          float speedRight = rightStr.toFloat();
          
          // Validar rangos razonables
          speedLeft = constrain(speedLeft, -100.0, 100.0);
          speedRight = constrain(speedRight, -100.0, 100.0);
          
          motorIzq.establecerSetpoint(speedLeft);
          motorDer.establecerSetpoint(speedRight);
          
          Serial.println("ACK,SPEED");
        } else {
          Serial.println("ERROR,SPEED_FORMAT");
        }
      }
      else if (msg.startsWith("CMD,PID,")) {
        // Formato: CMD,PID,KP,KI,KD
        int firstComma = msg.indexOf(',', 8);
        int secondComma = msg.indexOf(',', firstComma + 1);
        int thirdComma = msg.indexOf(',', secondComma + 1);
        
        if (firstComma > 8 && secondComma > firstComma && thirdComma > secondComma) {
          float kp = msg.substring(8, firstComma).toFloat();
          float ki = msg.substring(firstComma + 1, secondComma).toFloat();
          float kd = msg.substring(secondComma + 1, thirdComma).toFloat();
          
          motorIzq.configurarPID(kp, ki, kd);
          motorDer.configurarPID(kp, ki, kd);
          
          Serial.println("ACK,PID");
        } else {
          Serial.println("ERROR,PID_FORMAT");
        }
      }
      else if (msg.equals("CMD,STATUS")) {
        // Enviar estado detallado
        RobotState state = getState();
        Serial.print("STATUS,");
        Serial.print(state);
        Serial.print(",L:");
        Serial.print(motorIzq.obtenerVelocidadActual());
        Serial.print(",R:");
        Serial.println(motorDer.obtenerVelocidadActual());
      }
      else {
        Serial.println("ERROR,UNKNOWN_CMD");
      }
    }
    
    // Enviar heartbeat periódico usando FreeRTOS ticks
    if ((xTaskGetTickCount() - lastHeartbeat) > pdMS_TO_TICKS(5000)) {
      RobotState state = getState();
      Serial.print("HEARTBEAT,");
      Serial.print(state);
      Serial.print(",");
      //Serial.print(xPortGetFreeHeapSize());
      Serial.println();
      lastHeartbeat = xTaskGetTickCount();
    }
    
    vTaskDelay(pdMS_TO_TICKS(50)); // Reducir delay para mejor respuesta
  }
}