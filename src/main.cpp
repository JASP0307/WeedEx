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
#include "DeltaKinematics.h"

#define btSerial Serial1

// Handles globales
QueueHandle_t fsmQueue;
SemaphoreHandle_t stateMutex;

// Mutex para proteger el acceso a la variable del voltaje
SemaphoreHandle_t batteryMutex;

QueueHandle_t deltaQueue;

// --- LÓGICA DEL BRAZO DELTA ---
// Parámetros físicos y de compensación
DeltaKinematics DK(52, 113, 37, 63);
const int SERVO1_HORIZONTAL = 172; 
const int SERVO2_HORIZONTAL = 168;
const int SERVO3_HORIZONTAL = 178;
const double ROTATION_ANGLE_DEG = -30.0;
double cos_theta, sin_theta;

// Posiciones
const double HOME_X = 0, HOME_Y = 0, HOME_Z = -140;
const double Z_ATAQUE = -105;

// Estructura y array para el grid de ataque
struct GridPoint { double x; double y; double z;};
const int NUM_GRID_POINTS = 20;
GridPoint grid[NUM_GRID_POINTS];


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
ServoModule SERV_04(Pinout::Rastrillos::SERVO_4);

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
void TaskSensors(void *pvParameters);
void TaskBattery(void *pvParameters);
void TaskComms(void *pvParameters);
void TaskServoControl(void *pvParameters);
void TaskDeltaControl(void *pvParameters);
void TaskBluetoothCommunication(void *pvParameters);

// Funciones auxiliares del brazo delta
void delta_init();
void delta_moveTo(double x, double y, double z);
void delta_moveTo_Compensated(double userX, double userY, double userZ);

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

  batteryMutex = xSemaphoreCreateMutex();
  if (batteryMutex == NULL) {
      Serial.println("Error: No se pudo crear el batteryMutex");
  }

  locomotionMutex = xSemaphoreCreateMutex();
  if (locomotionMutex == NULL) {
      Serial.println("Error: No se pudo crear el locomotionMutex");
  }

  deltaQueue = xQueueCreate(5, sizeof(int));



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

  delta_init();
  //SERV_01.begin();
  Serial.println("Motores inicializados");
  
  // Crear tareas
  xTaskCreate(TaskFSM,                    "FSM",        512, NULL, 3, NULL);
  xTaskCreate(TaskLocomotion,             "Locomotion", 256, NULL, 1, NULL);
  xTaskCreate(TaskSensors,                "Sensors",    256, NULL, 2, NULL);
  xTaskCreate(TaskBattery,                "Battery",    128, NULL, 2, NULL);
  xTaskCreate(TaskComms,                  "Comms", 256, NULL, 2, NULL);
  xTaskCreate(TaskServoControl,           "ServoControl", 256, NULL, 1, NULL);
  xTaskCreate(TaskBluetoothCommunication, "Bluetooth Test Task", 1024, NULL, 2, NULL);
  xTaskCreate(TaskDeltaControl,           "DeltaControl", 256, NULL, 2, NULL);

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
    case ERROR_STATE: return "ERROR_STATE";
    case LOW_BATTERY: return "LOW_BATTERY";
    case OBSTACLE: return "OBSTACLE";
    default: return "UNKNOWN";
  }
}

float getBatteryVoltage() {
  float voltage;
  if (xSemaphoreTake(batteryMutex, portMAX_DELAY)) {
    voltage = g_batteryVoltage;
    xSemaphoreGive(batteryMutex);
  }
  return voltage;
}

void setTargetYaw(float newYaw) {
  if (xSemaphoreTake(locomotionMutex, portMAX_DELAY)) {
    g_targetYaw = newYaw;
    xSemaphoreGive(locomotionMutex);
  }
}

void setBaseSpeed(float newSpeed) {
  if (xSemaphoreTake(locomotionMutex, portMAX_DELAY)) {
    g_baseSpeedRPM = newSpeed;
    xSemaphoreGive(locomotionMutex);
  }
}

void poblarGrid() {
    // Aquí pegas las 20 coordenadas que ya calibraste
    // Ejemplo:
    grid[0] = { -40.0, 50.0, -105.0 };
    grid[1] = { -30.0, 50.0, -105.0 };
    grid[2] = { -20.0, 50.0, -105.0 };
    grid[3] = { -40.0, 50.0, -105.0 };
    grid[4] = { -30.0, 50.0, -105.0 };
    grid[5] = { -20.0, 50.0, -105.0 };
    grid[6] = { -40.0, 50.0, -105.0 };
    grid[7] = { -30.0, 50.0, -105.0 };
    grid[8] = { -20.0, 50.0, -105.0 };
    grid[9] = { -40.0, 50.0, -105.0 };
    grid[10] = { -30.0, 50.0, -105.0};
    grid[11] = { -40.0, 50.0, -105.0 };
    grid[12] = { -30.0, 50.0, -105.0 };
    grid[13] = { -20.0, 50.0, -105.0 };
    grid[14] = { -40.0, 50.0, -105.0 };
    grid[15] = { -30.0, 50.0, -105.0 };
    grid[16] = { -20.0, 50.0, -105.0 };
    grid[17] = { -40.0, 50.0, -105.0 };
    grid[18] = { -30.0, 50.0, -105.0 };
    grid[19] = { -20.0, 50.0, -105.0 };

    Serial.println("Grid de ataque poblado.");
}

void delta_init() {

    // Poblar el grid con tus coordenadas
    poblarGrid();

    // Pre-calcular valores para la rotación
    double rotation_rad = ROTATION_ANGLE_DEG * PI / 180.0;
    cos_theta = cos(rotation_rad);
    sin_theta = sin(rotation_rad);

    // Mover a la posición HOME inicial
    delta_moveTo_Compensated(HOME_X, HOME_Y, HOME_Z);
    Serial.println("Brazo Delta inicializado y en HOME.");
}

void delta_moveTo_Compensated(double userX, double userY, double userZ) {
    double deltaX = userX * cos_theta - userY * sin_theta;
    double deltaY = userX * sin_theta + userY * cos_theta;
    delta_moveTo(deltaX, deltaY, userZ);
}

void delta_moveTo(double x, double y, double z) {
    if (DK.inverse(x, y, z) == no_error) {
        int s1 = SERVO1_HORIZONTAL - DK.a;
        int s2 = SERVO2_HORIZONTAL - DK.b;
        int s3 = SERVO3_HORIZONTAL - DK.c;
        SERV_01.setTarget(s1);
        SERV_02.setTarget(s2);
        SERV_03.setTarget(s3);
    } else {
        Serial.println("ERROR: Posicion DELTA inalcanzable.");
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
            setState(NAVIGATING);
            Serial.println("Estado: NAVIGATING");
          } else if (receivedEvent == EVENT_LOW_BATTERY) {
            setState(LOW_BATTERY);
            digitalWrite(Pinout::TiraLED::LEDs, LOW);
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
            digitalWrite(Pinout::TiraLED::LEDs, LOW);
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
            //g_armCommand = CMD_MOVE_TO_TARGET;     
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
          } else if (receivedEvent == EVENT_LOW_BATTERY) {
              setState(LOW_BATTERY);
              Serial.println("Estado: LOW_BATTERY");
              digitalWrite(Pinout::TiraLED::LEDs, LOW);
          }
          break;
        
        case LASERING:

          // Podríamos también recibir un evento de finalización del temporizador
          if (receivedEvent == EVENT_LASER_COMPLETE) {
            Laser_01.off();
            //g_armCommand = CMD_RETURN_HOME;
            setState(RETURNING_HOME);
            Serial.println("Estado: RETURNING_HOME");
          } else if (receivedEvent == EVENT_LOW_BATTERY) {
            setState(LOW_BATTERY);
            Serial.println("Estado: LOW_BATTERY");
            digitalWrite(Pinout::TiraLED::LEDs, LOW);
          }
          break;

        case RETURNING_HOME:
          // Esperando a que el brazo vuelva a su posición inicial.
          // La tarea del brazo delta enviará este evento.
          if (receivedEvent == EVENT_ATTACK_COMPLETE) {
              // Secuencia completada, volvemos a navegar
              setState(NAVIGATING);
              Serial.println("Estado: NAVIGATING (secuencia de actuacion finalizada)");
          } else if (receivedEvent == EVENT_LOW_BATTERY) {
            setState(LOW_BATTERY);
            Serial.println("Estado: LOW_BATTERY");
            digitalWrite(Pinout::TiraLED::LEDs, LOW);
          }
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
            motorIzq.establecerSetpoint(10);
            motorDer.establecerSetpoint(10);
            setState(NAVIGATING);
            Serial.println("Estado: NAVIGATING (obstáculo evitado)");
          } else if (receivedEvent == EVENT_ERROR) {
            setState(ERROR_STATE);
            Serial.println("Estado: ERROR_STATE");
          } else if (receivedEvent == EVENT_LOW_BATTERY) {
            setState(LOW_BATTERY);
            Serial.println("Estado: LOW_BATTERY");
            digitalWrite(Pinout::TiraLED::LEDs, LOW);
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

  for (;;) {
    SERV_01.update();
    SERV_02.update();
    SERV_03.update();
    //SERV_04.update();
    vTaskDelay(pdMS_TO_TICKS(30)); 
  }
}

// --- DECLARACIONES PARA LA TAREA PID ---
float integralError = 0.0;
float previousError = 0.0;

void TaskLocomotion(void *pvParameters) {
  (void) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);

  for (;;) {
    if (getState() == NAVIGATING) {
      float currentYaw = yawSensor.getYaw();
      float targetYaw;
      float baseSpeedRPM;
      
      if (xSemaphoreTake(locomotionMutex, portMAX_DELAY)) {
        targetYaw = g_targetYaw;
        baseSpeedRPM = g_baseSpeedRPM;
        xSemaphoreGive(locomotionMutex);
      }

      float headingError = targetYaw - currentYaw;
      if (headingError > 180.0) headingError -= 360.0;
      else if (headingError < -180.0) headingError += 360.0;

      integralError += headingError;
      if (integralError > 100.0) integralError = 100.0;
      else if (integralError < -100.0) integralError = -100.0;

      float derivativeError = headingError - previousError;
      previousError = headingError;

      float pidCorrection = (Kp_heading * headingError) + (Ki_heading * integralError) + (Kd_heading * derivativeError);
      float leftSetpoint = baseSpeedRPM - pidCorrection;
      float rightSetpoint = baseSpeedRPM + pidCorrection;
      //Serial.print(leftSetpoint);
      //Serial.print(" | ");
      //Serial.println(rightSetpoint);
      
      //motorIzq.establecerSetpoint(leftSetpoint);
      //motorDer.establecerSetpoint(-rightSetpoint);
    } else {
      motorIzq.establecerSetpoint(0);
      motorDer.establecerSetpoint(0);
      integralError = 0.0;
      previousError = 0.0;
    }
    
    motorIzq.actualizar();
    motorDer.actualizar();
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);  
  }
}

void TaskDeltaControl(void *pvParameters){
  (void) pvParameters;

  int grid_index;
  FSMEvent eventToSend;

  for(;;){
    // Esperar a que llegue un comando (índice del grid) a la cola
    if(xQueueReceive(deltaQueue, &grid_index, portMAX_DELAY)){

      // 1. Validar el índice recibido
      if(grid_index >= 0 && grid_index < NUM_GRID_POINTS){
        
        // 2. Notificar a la FSM que el brazo se está moviendo
        eventToSend = EVENT_WEED_FOUND; // O un nuevo evento como EVENT_ARM_MOVING
        xQueueSend(fsmQueue, &eventToSend, 0);
        
        // 3. Obtener las coordenadas del grid y moverse
        GridPoint target = grid[grid_index];
        delta_moveTo_Compensated(target.x, target.y, target.z);
        vTaskDelay(pdMS_TO_TICKS(1500)); // Simular tiempo de movimiento

        // 4. Notificar a la FSM que el brazo llegó al objetivo
        eventToSend = EVENT_ARM_AT_TARGET;
        xQueueSend(fsmQueue, &eventToSend, 0);

        // --- Aquí se quedaría esperando hasta que termine la acción (ej. láser) ---
        // La FSM se encarga de la temporización del láser.
        // Después de que la FSM complete el estado LASERING, enviará un comando de regreso a casa.
        // Podríamos manejarlo aquí o en la FSM. Por simplicidad, la FSM lo maneja.

        // Simulación de espera durante el lasering
        vTaskDelay(pdMS_TO_TICKS(3000)); 

        // 5. Volver a la posición HOME
        delta_moveTo_Compensated(HOME_X, HOME_Y, HOME_Z);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Simular tiempo de regreso

        // 6. Notificar a la FSM que la secuencia completa ha terminado
        eventToSend = EVENT_ATTACK_COMPLETE; // <<<--- NUEVO EVENTO
        xQueueSend(fsmQueue, &eventToSend, 0);

      } else {
        Serial.println("ERROR: Índice de grid inválido recibido.");
      }
    }
  }
}

void TaskSensors(void *pvParameters) {
  (void) pvParameters;
  
  // Reducir el delay para mejor resolución temporal
  //const TickType_t delayTicks = pdMS_TO_TICKS(10); // 50ms en lugar de 500ms
  
  // Inicializar el sensor
  yawSensor.begin();
  
  const int DELAY_MS = 20; // 100 Hz
  const TickType_t delayTicks = pdMS_TO_TICKS(DELAY_MS);
  const float DELAY_SEC = DELAY_MS / 1000.0;
  //Serial.print("DEBUG: DELAY_SEC calculado es: ");
  //Serial.println(DELAY_SEC, 4); 
  TickType_t lastWakeTime = xTaskGetTickCount();
  unsigned long lastPrintTime = 0;
  for (;;) {
    yawSensor.update(DELAY_SEC);
    
    float yaw = yawSensor.getYaw();
    unsigned long now = millis();
    if (now - lastPrintTime > 250) {
      float yaw = yawSensor.getYaw();
      //Serial.print("Yaw Angle [°]: ");
      //Serial.println(yaw, 2);
      lastPrintTime = now;
    }/*
    for (int i = 0; i < NUM_SENSORES; i++) {
      if (sensores[i].distancia() < DISTANCIA_MINIMA) {
          //FSMEvent e = EVENT_OBSTACLE;
          //xQueueSend(fsmQueue, &e, 0);
          break;
      }
    }
  */
    vTaskDelayUntil(&lastWakeTime, delayTicks);
  }
}

void TaskBattery(void *pvParameters) {
  (void) pvParameters;
  
  const int NUM_LECTURAS = 10;
  float lecturas[NUM_LECTURAS];
  int indiceLectura = 0;

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


    if (voltajePromedio < VOLTAJE_BATERIA_BAJA && voltajePromedio > 5.0) { // El > 5.0 evita falsos positivos al desconectar
        FSMEvent e = EVENT_LOW_BATTERY;
        xQueueSend(fsmQueue, &e, 0);
    }

    if (xSemaphoreTake(batteryMutex, portMAX_DELAY)) {
        g_batteryVoltage = voltajePromedio;
        xSemaphoreGive(batteryMutex);
    }

    // Comprobar el voltaje cada 5 segundos
    vTaskDelay(pdMS_TO_TICKS(5000));
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
      else if (msg.startsWith("ATTACK,")) {
        String indexStr = msg.substring(7); // "ATTACK," tiene 7 caracteres
        int grid_index = indexStr.toInt();
        
        // Enviar el índice a la cola del brazo delta
        if (xQueueSend(deltaQueue, &grid_index, pdMS_TO_TICKS(100)) == pdPASS) {
            Serial.println("ACK,ATTACK_CMD_RECEIVED");
        } else {
            Serial.println("ERROR,DELTA_QUEUE_FULL");
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

    vTaskDelay(pdMS_TO_TICKS(100)); // Reducir delay para mejor respuesta
  }
}

void TaskBluetoothCommunication(void *pvParameters) {
  (void) pvParameters;
  String incomingString = "";
  
  // Temporizador para enviar toda la telemetría a un intervalo fijo
  uint32_t lastTelemetrySendTime = 0;
  const uint32_t TELEMETRY_INTERVAL_MS = 1000; // Enviar todo cada 1 segundo

  for (;;) {
    // --- Parte 1: Escuchar comandos (sin cambios) ---
    if (btSerial.available() > 0) {
      char c = btSerial.read();
      if (c == '\n') {
        incomingString.trim(); 
        if (incomingString == "START") {
          //Serial.println("DEBUG: Comando START recibido");
          FSMEvent e = EVENT_NAVIGATE;
          xQueueSend(fsmQueue, &e, 0);
        } else if (incomingString == "STOP") {
          //Serial.println("DEBUG: Comando STOP recibido");
          FSMEvent e = EVENT_STOP;
          xQueueSend(fsmQueue, &e, 0);
        } else if (incomingString.startsWith("YAW:")) {
          // Extraer el valor después de "YAW:"
          String yawValueStr = incomingString.substring(4); // 4 es la longitud de "YAW:"
          float newYaw = yawValueStr.toFloat();
          setTargetYaw(newYaw); // Llamar a nuestra función segura
          Serial.print("DEBUG: Nuevo Yaw Objetivo establecido a -> ");
          Serial.println(newYaw);
        }
        incomingString = "";
      } else {
        incomingString += c;
      }
    }

    // --- Parte 2: Construir y enviar el paquete de telemetría periódicamente ---
    if (millis() - lastTelemetrySendTime > TELEMETRY_INTERVAL_MS) {
      
      RobotState currentState = getState();
      float currentVoltage = getBatteryVoltage();

      String stateStr = robotStateToString(currentState);
      
      char voltageBuffer[10]; // Buffer para convertir el float del voltaje
      dtostrf(currentVoltage, 4, 2, voltageBuffer); // Formato: 4 caracteres en total, 2 decimales
      String voltageStr = String(voltageBuffer);

      String telemetryPacket = "STATE:" + stateStr + ":BATT:" + voltageStr;

      btSerial.println(telemetryPacket);
      //Serial.println("DEBUG: Enviando Telemetría -> " + telemetryPacket);

      lastTelemetrySendTime = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}
