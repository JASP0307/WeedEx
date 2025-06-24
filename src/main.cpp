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
#include "Laser.h"
#include "RobotDefinitions.h"

#define btSerial Serial1

// Handles globales
QueueHandle_t fsmQueue;
SemaphoreHandle_t stateMutex;

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

//ServoModule SERV_01(Pinout::BrazoDelta::SERVO_1);
//ServoModule SERV_02(Pinout::BrazoDelta::SERVO_2);
//ServoModule SERV_03(Pinout::BrazoDelta::SERVO_3);
//ServoModule SERV_04(Pinout::Rastrillos::SERVO_4);

YawSensor yawSensor;
Ultrasonico UltrDer(Pinout::Sensores::Ultra_Der::TRIG, Pinout::Sensores::Ultra_Der::ECHO);
Ultrasonico UltrIzq(Pinout::Sensores::Ultra_Izq::TRIG, Pinout::Sensores::Ultra_Izq::ECHO);
Ultrasonico UltrFront(Pinout::Sensores::Ultra_Front::TRIG, Pinout::Sensores::Ultra_Front::ECHO);

Ultrasonico sensores[] = {UltrDer, UltrFront, UltrIzq}; 
const int NUM_SENSORES = sizeof(sensores) / sizeof(sensores[0]);



// --- Variables para la gestión de la acción ---

TickType_t g_rakeStartTime = 0;

Laser Laser_01(Pinout::Laser::Laser_1);

// Tiempos de simulación en milisegundos

TickType_t obstacleEntryTime = 0;
TickType_t laserStartTime  = 0;



// Prototipos de tareas
void TaskFSM(void *pvParameters);
void TaskLocomotion(void *pvParameters);
//void TaskActuation(void *pvParameters);
void TaskSensors(void *pvParameters);
void TaskBattery(void *pvParameters);
void TaskComms(void *pvParameters);
void TaskServoControl(void *pvParameters);
void TaskSimulateArm(void *pvParameters);
void TaskBluetoothCommunication(void *pvParameters);

// Funciones auxiliares
RobotState getState();
void setState(RobotState newState);

void setup() {
  Serial.begin(115200);
  btSerial.begin(9600); 
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

  pinMode(Pinout::TiraLED::LEDs, OUTPUT);

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
  //SERV_01.begin();
  Serial.println("Motores inicializados");
  
  // Crear tareas
  xTaskCreate(TaskFSM,                    "FSM",        512, NULL, 3, NULL);
  xTaskCreate(TaskLocomotion,             "Locomotion", 256, NULL, 2, NULL);
  //xTaskCreate(TaskActuation,              "Actuation",  128, NULL, 2, NULL);
  xTaskCreate(TaskSensors,                "Sensors",    256, NULL, 2, NULL);
  xTaskCreate(TaskBattery,                "Battery",    128, NULL, 2, NULL);
  xTaskCreate(TaskComms,                  "Comms",      256, NULL, 2, NULL);
  xTaskCreate(TaskServoControl,           "ServoControl",  256, NULL, 1, NULL);
  xTaskCreate(TaskSimulateArm,            "Arm Sim Task", 256, NULL, 2, NULL);
  xTaskCreate(TaskBluetoothCommunication, "Bluetooth Test Task", 2048, NULL, 2, NULL);

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

String robotStateToString(RobotState state) {
  switch(state) {
    
    case IDLE: return "IDLE";
    case NAVIGATING: return "NAVIGATING";
    case MOVING_TO_WEED: return "MOVING_TO_WEED";
    case LASERING: return "LASERING";
    case RETURNING_HOME: return "RETURNING_HOME";
    case ACTUATING: return "ACTUATING";
    case ERROR_STATE: return "ERROR_STATE";
    case LOW_BATTERY: return "LOW_BATTERY";
    case OBSTACLE: return "OBSTACLE";
    default: return "UNKNOWN";
  }
}

// FSM principal
void TaskFSM(void *pvParameters) {
  (void) pvParameters;

  FSMEvent receivedEvent;

  for (;;) {
    if (xQueueReceive(fsmQueue, &receivedEvent, pdMS_TO_TICKS(100))) {
      RobotState state = getState();
      
      switch (state) {
        case IDLE:
          if (receivedEvent == EVENT_NAVIGATE) {
            // Asegurar que motores estén listos
            if (!motorIzq.pidEstaActivo()) {
              motorIzq.activarPID(true);
              motorDer.activarPID(true);
            }
            digitalWrite(Pinout::TiraLED::LEDs, HIGH);
            //SERV_01.setTarget(30);
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
            digitalWrite(Pinout::TiraLED::LEDs, LOW);
            obstacleEntryTime = xTaskGetTickCount();
          } else if (receivedEvent == EVENT_LOW_BATTERY) {
            setState(LOW_BATTERY);
            Serial.println("Estado: LOW_BATTERY");
          } else if (receivedEvent == EVENT_STOP) {
            // Detener motores gradualmente
            motorIzq.establecerSetpoint(0);
            motorDer.establecerSetpoint(0);
            digitalWrite(Pinout::TiraLED::LEDs, LOW);
            //SERV_01.setTarget(180); // Mover a 120 grados
            setState(IDLE);
            Serial.println("Estado: IDLE");
          }else if (receivedEvent == EVENT_WEED_FOUND) {
            // Detener la navegación
            motorIzq.establecerSetpoint(0);
            motorDer.establecerSetpoint(0);
            
            // Iniciar el movimiento del brazo (función no bloqueante)
            //brazoDelta.startMoveTo(posicionMaleza); // Asume que esta función retorna de inmediato
            g_armCommand = CMD_MOVE_TO_TARGET;     
            setState(MOVING_TO_WEED);
            Serial.println("Estado: MOVING_TO_WEED");
          }
          else if (receivedEvent == EVENT_RAKE_WEED_FOUND) {
        // Solo inicia la secuencia si no está ya en progreso
            if (!g_isRaking) {
                Serial.println("RASTRILLO: Iniciando secuencia de rastrillado.");
                g_isRaking = true;
                g_rakeStartTime = xTaskGetTickCount();
                //SERV_04.setTarget(POS_TRABAJO_RASTRILLO); // Bajar el rastrillo
            }
          } 
          break;

        case MOVING_TO_WEED:
          // Detener motores y cambiar a actuación
          motorIzq.establecerSetpoint(0);
          motorDer.establecerSetpoint(0);
          // El robot está esperando a que el brazo llegue.
          // La tarea que controla el brazo delta deberá enviar este evento cuando termine.
          if (receivedEvent == EVENT_ARM_AT_TARGET) {
              // El brazo llegó, ahora encendemos el láser
              Laser_01.on();
              laserStartTime = xTaskGetTickCount(); // Inicia el temporizador del láser
              setState(LASERING);
              Serial.println("Estado: LASERING (2s)");
          }
          break;
        
        case LASERING:
          // En este estado, no esperamos un evento, sino que el tiempo pase.
          // Usamos la misma técnica no bloqueante que con el obstáculo.
          // La comprobación se haría fuera del 'if (xQueueReceive...)'
          // ... (ver lógica de temporizador más abajo)
          
          // Podríamos también recibir un evento de finalización del temporizador
          if (receivedEvent == EVENT_LASER_COMPLETE) {
              Laser_01.off();
              //brazoDelta.startReturnToHome(); // Inicia el movimiento de vuelta (no bloqueante)
              g_armCommand = CMD_RETURN_HOME;
              setState(RETURNING_HOME);
              Serial.println("Estado: RETURNING_HOME");
          }
          break;

        case RETURNING_HOME:
          // Esperando a que el brazo vuelva a su posición inicial.
          // La tarea del brazo delta enviará este evento.
          if (receivedEvent == EVENT_ARM_AT_HOME) {
              // Secuencia completada, volvemos a navegar
              setState(NAVIGATING);
              Serial.println("Estado: NAVIGATING (secuencia de actuacion finalizada)");
              // Reactivar motores para navegar
              //motorIzq.establecerSetpoint(VELOCIDAD_NORMAL);
              //motorDer.establecerSetpoint(VELOCIDAD_NORMAL);
          }
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
    RobotState currentState = getState();
    if (currentState == OBSTACLE) {
      // Comprobar si han pasado 5 segundos desde que entramos en el estado
      if ((xTaskGetTickCount() - obstacleEntryTime) >= pdMS_TO_TICKS(5000)) {
        setState(NAVIGATING);
        Serial.println("Estado: NAVIGATING (5s de espera finalizados)");
        // Aquí podrías también reanudar la velocidad de los motores si es necesario
        // motorIzq.establecerSetpoint(VELOCIDAD_ANTERIOR);
        // motorDer.establecerSetpoint(VELOCIDAD_ANTERIOR);
      }
    }

    if (currentState == LASERING) {
        if ((xTaskGetTickCount() - laserStartTime) >= pdMS_TO_TICKS(2000)) {
            // El tiempo ha pasado, enviamos un evento a nuestra propia cola
            // para mantener la lógica de la FSM basada en eventos.
            FSMEvent e = EVENT_LASER_COMPLETE;
            xQueueSend(fsmQueue, &e, 0);
        }
    }

    if (g_isRaking && (xTaskGetTickCount() - g_rakeStartTime >= pdMS_TO_TICKS(5000))) {
            Serial.println("RASTRILLO: 5 segundos completados. Subiendo rastrillo.");
            //SERV_04.setTarget(POS_INICIAL_RASTRILLO); // Subir el rastrillo
            g_isRaking = false; // Finalizar la secuencia
    }
    //vTaskDelay(pdMS_TO_TICKS(50)); // Reducir delay para mejor respuesta
  }
}

void TaskServoControl(void *pvParameters) {
  (void) pvParameters;

  vTaskDelay(pdMS_TO_TICKS(1000)); 
  
  //SERV_01.begin();
  //SERV_01.setTarget(120);
  //SERV_02.begin();
  //SERV_03.begin();
  //SERV_04.begin();

  for (;;) {
    //SERV_01.update();
    //SERV_02.update();
    //SERV_03.update();
    //SERV_04.update();
//
    vTaskDelay(pdMS_TO_TICKS(50)); 
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

void TaskSensors(void *pvParameters) {
  (void) pvParameters;
  
  // Reducir el delay para mejor resolución temporal
  const TickType_t delayTicks = pdMS_TO_TICKS(150); // 50ms en lugar de 500ms
  
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
          FSMEvent e = EVENT_WEED_FOUND;
          //Serial.println(i);
          xQueueSend(fsmQueue, &e, 0);
          break;
      }
    }
    
    vTaskDelayUntil(&lastWakeTime, delayTicks);
  }
}

void TaskBattery(void *pvParameters) {
  (void) pvParameters;
  
  const int NUM_LECTURAS = 10;
  float lecturas[NUM_LECTURAS];
  int indiceLectura = 0;

  // --- Inicialización del Buffer (CORRECCIÓN) ---
  // Para evitar falsos positivos de batería baja al inicio, "cebamos" el
  // buffer con la primera lectura real antes de empezar el bucle principal.
  vTaskDelay(pdMS_TO_TICKS(100)); // Pequeña pausa para estabilizar el ADC
  
  int valorInicial = analogRead(Pinout::Sensores::Battery::Battery);
  float voltajePinInicial = (valorInicial / 1023.0) * 5.0;
  float voltajeBateriaInicial = voltajePinInicial * ((R1 + R2) / R2);
  
  // Llenar todo el array con esta primera lectura realista
  for (int i = 0; i < NUM_LECTURAS; i++) {
    lecturas[i] = voltajeBateriaInicial;
  }
  for (;;) {
    // 1. Leer el valor crudo del pin analógico (0-1023)
    int valorSensor = analogRead(Pinout::Sensores::Battery::Battery);

    float voltajePin = (valorSensor / 1023.0) * 5.0;

    float voltajeBateria = voltajePin * ((R1 + R2) / R2);

    lecturas[indiceLectura] = voltajeBateria;
    indiceLectura++;
    if (indiceLectura >= NUM_LECTURAS) {
      indiceLectura = 0;
    }

    float voltajePromedio = 0;
    for (int i = 0; i < NUM_LECTURAS; i++) {
      voltajePromedio += lecturas[i];
    }
    voltajePromedio /= NUM_LECTURAS;
    
    // Imprimir para depuración
    Serial.print("Voltaje Batería: ");
    Serial.println(voltajePromedio);

    // 5. Comprobar si el voltaje es bajo y enviar evento a la FSM
    if (voltajePromedio < VOLTAJE_BATERIA_BAJA && voltajePromedio > 5.0) { // El > 5.0 evita falsos positivos al desconectar
        FSMEvent e = EVENT_LOW_BATTERY;
        xQueueSend(fsmQueue, &e, 0); // Asume que fsmQueue es global
    }

    // Comprobar el voltaje cada 2 segundos
    vTaskDelay(pdMS_TO_TICKS(2000));
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

void TaskSimulateArm(void *pvParameters) {
  (void) pvParameters; // Evitar advertencia de parámetro no usado

  FSMEvent eventToSend;

  Serial.println("[SIM_ARM] Tarea de simulación de brazo iniciada.");

  for (;;) {
    // 1. Comprobar si hay un nuevo comando
    if (g_armCommand != CMD_IDLE) {
      
      // 2. Procesar el comando recibido
      switch (g_armCommand) {
        
        case CMD_MOVE_TO_TARGET:
          Serial.println("[SIM_ARM] Comando recibido: Mover a objetivo. Simulando por 3s...");
          vTaskDelay(pdMS_TO_TICKS(SIMULATED_MOVE_TIME_MS)); // Simulación no bloqueante
          
          Serial.println("[SIM_ARM] Simulación completada. Enviando EVENT_ARM_AT_TARGET.");
          eventToSend = EVENT_ARM_AT_TARGET;
          xQueueSend(fsmQueue, &eventToSend, 0);
          break;

        case CMD_RETURN_HOME:
          Serial.println("[SIM_ARM] Comando recibido: Volver a casa. Simulando por 3s...");
          vTaskDelay(pdMS_TO_TICKS(SIMULATED_MOVE_TIME_MS)); // Simulación no bloqueante

          Serial.println("[SIM_ARM] Simulación completada. Enviando EVENT_ARM_AT_HOME.");
          eventToSend = EVENT_ARM_AT_HOME;
          xQueueSend(fsmQueue, &eventToSend, 0);
          break;
        
        default:
          // Comando desconocido, no hacer nada.
          break;
      }

      // 3. Resetear el comando para no volver a ejecutarlo
      g_armCommand = CMD_IDLE;
    }

    // Esperar un poco antes de volver a comprobar para no consumir el 100% de la CPU
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void TaskBluetoothCommunication(void *pvParameters) {
  (void) pvParameters;
  String incomingString = ""; // Buffer para comandos entrantes
  String lastSentState = "";  // Para no saturar el canal enviando el mismo estado

  for (;;) {
    // --- 1. Escuchar comandos entrantes desde el Celular (vía HC-05) ---
    if (btSerial.available() > 0) {
      char c = btSerial.read();
      if (c == '\n') {
        // Comando completo recibido
        incomingString.trim(); 
        
        if (incomingString == "START") {
          // Usamos el Serial principal para depuración en el Monitor Serie
          Serial.println("DEBUG: Comando START recibido por Bluetooth");
          FSMEvent e = EVENT_NAVIGATE;
          xQueueSend(fsmQueue, &e, 0);
        } else if (incomingString == "STOP") {
          Serial.println("DEBUG: Comando STOP recibido por Bluetooth");
          FSMEvent e = EVENT_STOP;
          xQueueSend(fsmQueue, &e, 0);
        }
        
        incomingString = ""; // Limpiar el buffer
      } else {
        incomingString += c;
      }
    }

    // --- 2. Enviar estado actual al Celular ---
    RobotState currentState = getState();

    String stateStr = "STATE:" + robotStateToString(currentState); // Usa la misma función de ayuda

    if (stateStr != lastSentState) {
        // Enviar el estado al celular a través del HC-05
        btSerial.println(stateStr);
        // También lo imprimimos en el monitor serie para depurar
        Serial.println("DEBUG: Enviando estado a Bluetooth -> " + stateStr);
        lastSentState = stateStr;
    }
    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}