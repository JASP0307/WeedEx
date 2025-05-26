#include <Arduino.h>
#include "Motor_Control.h"

// Máquina de Estados - Arduino Mega Robot Quitamaleza
// Comunicación Serial USB con MiniPC

// Estados del sistema
enum RobotState {
  IDLE,
  NAVIGATING,
  RAKE_OPERATION,
  LASER_POSITIONING,
  LASER_OPERATION,
  LASER_RETURNING,
  EMERGENCY_STOP
};

// Variables de estado
RobotState currentState = IDLE;
RobotState previousState = IDLE;
unsigned long stateTimer = 0;
unsigned long lastUltrasonicCheck = 0;
unsigned long lastSerialCheck = 0;

// Constantes de tiempo
const unsigned long ULTRASONIC_INTERVAL = 50;    // Check cada 50ms
const unsigned long SERIAL_INTERVAL = 10;       // Check cada 10ms
const unsigned long RAKE_DURATION = 2000;       // 2 segundos
const unsigned long LASER_DURATION = 3000;      // 3 segundos

// Variables de operación
int navigationSpeed = 0;
int laserX = 0, laserY = 0;
bool obstacleDetected = false;
String serialBuffer = "";

// Pines (ajustar según tu hardware)
const int MOTOR_LEFT_PWM = 5;
const int MOTOR_RIGHT_PWM = 6;
const int RAKE_SERVO = 9;
const int LASER_ENABLE = 7;
const int ULTRASONIC_TRIG = 2;
const int ULTRASONIC_ECHO = 3;

void setup() {
  Serial.begin(115200);
  
  // Configurar pines
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(LASER_ENABLE, OUTPUT);
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  
  // Inicializar actuadores
  digitalWrite(LASER_ENABLE, LOW);
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
  
  Serial.println("STATUS:ARDUINO_READY");
  currentState = IDLE;
}

void loop() {
  unsigned long currentTime = millis();
  
  // Monitoreo continuo de ultrasonidos
  if (currentTime - lastUltrasonicCheck >= ULTRASONIC_INTERVAL) {
    checkUltrasonics();
    lastUltrasonicCheck = currentTime;
  }
  
  // Procesamiento de comandos serial
  if (currentTime - lastSerialCheck >= SERIAL_INTERVAL) {
    processSerialCommands();
    lastSerialCheck = currentTime;
  }
  
  // Ejecutar máquina de estados
  executeStateMachine();
}

void executeStateMachine() {
  switch (currentState) {
    case IDLE:
      handleIdleState();
      break;
      
    case NAVIGATING:
      handleNavigatingState();
      break;
      
    case RAKE_OPERATION:
      handleRakeOperationState();
      break;
      
    case LASER_POSITIONING:
      handleLaserPositioningState();
      break;
      
    case LASER_OPERATION:
      handleLaserOperationState();
      break;
      
    case LASER_RETURNING:
      handleLaserReturningState();
      break;
      
    case EMERGENCY_STOP:
      handleEmergencyStopState();
      break;
  }
}

void handleIdleState() {
  // Motores apagados, actuadores en posición segura
  stopMotors();
  disableLaser();
  retractRakes();
}

void handleNavigatingState() {
  // Mantener velocidad de navegación
  setMotorSpeed(navigationSpeed);
  
  // Verificar si hay comando de cambio desde MiniPC
  // La lógica principal está en processSerialCommands()
}

void handleRakeOperationState() {
  static bool rakesDeployed = false;
  
  if (!rakesDeployed) {
    deployRakes();
    rakesDeployed = true;
    stateTimer = millis();
    Serial.println("STATUS:RAKES_DEPLOYED");
  }
  
  // Mantener navegación durante operación de rastrillos
  setMotorSpeed(navigationSpeed);
  
  // Verificar timeout
  if (millis() - stateTimer >= RAKE_DURATION) {
    retractRakes();
    rakesDeployed = false;
    changeState(NAVIGATING);
    Serial.println("STATUS:RAKE_COMPLETE");
  }
}

void handleLaserPositioningState() {
  stopMotors();
  
  // Posicionar láser (implementar según tu brazo delta)
  if (positionLaser(laserX, laserY)) {
    changeState(LASER_OPERATION);
    Serial.println("STATUS:LASER_POSITIONED");
  }
}

void handleLaserOperationState() {
  static bool laserStarted = false;
  
  if (!laserStarted) {
    enableLaser();
    laserStarted = true;
    stateTimer = millis();
    Serial.println("STATUS:LASER_ON");
  }
  
  // Verificar timeout
  if (millis() - stateTimer >= LASER_DURATION) {
    disableLaser();
    laserStarted = false;
    changeState(LASER_RETURNING);
    Serial.println("STATUS:LASER_OFF");
  }
}

void handleLaserReturningState() {
  // Devolver láser a posición original
  if (returnLaserHome()) {
    changeState(NAVIGATING);
    Serial.println("STATUS:LASER_HOME");
  }
}

void handleEmergencyStopState() {
  stopMotors();
  disableLaser();
  retractRakes();
  
  // Solo salir si no hay obstáculo
  if (!obstacleDetected) {
    changeState(previousState);
    Serial.println("STATUS:EMERGENCY_CLEARED");
  }
}

void processSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleSerialCommand(serialBuffer);
      serialBuffer = "";
    } else {
      serialBuffer += c;
    }
  }
}

void handleSerialCommand(String command) {
  if (command.startsWith("START_NAV:")) {
    navigationSpeed = command.substring(10).toInt();
    changeState(NAVIGATING);
    Serial.println("STATUS:NAV_STARTED");
    
  } else if (command.startsWith("DEPLOY_RAKES")) {
    if (currentState == NAVIGATING) {
      changeState(RAKE_OPERATION);
    }
    
  } else if (command.startsWith("LASER_POS:")) {
    int commaIndex = command.indexOf(',');
    laserX = command.substring(10, commaIndex).toInt();
    laserY = command.substring(commaIndex + 1).toInt();
    if (currentState == NAVIGATING) {
      changeState(LASER_POSITIONING);
    }
    
  } else if (command == "EMERGENCY_STOP") {
    changeState(EMERGENCY_STOP);
    
  } else if (command == "GET_SENSORS") {
    reportSensorStatus();
    
  } else if (command == "STOP") {
    changeState(IDLE);
    Serial.println("STATUS:STOPPED");
  }
}

void checkUltrasonics() {
  long distance = getUltrasonicDistance();
  
  if (distance < 20 && distance > 0) { // Menos de 20cm
    if (!obstacleDetected) {
      obstacleDetected = true;
      if (currentState != EMERGENCY_STOP) {
        previousState = currentState;
        changeState(EMERGENCY_STOP);
        Serial.println("OBSTACLE:distance_" + String(distance) + "cm");
      }
    }
  } else {
    obstacleDetected = false;
  }
}

long getUltrasonicDistance() {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);
  if (duration == 0) return -1; // Timeout
  
  return duration * 0.034 / 2; // Convertir a cm
}

void changeState(RobotState newState) {
  previousState = currentState;
  currentState = newState;
  stateTimer = millis();
}

void reportSensorStatus() {
  long distance = getUltrasonicDistance();
  Serial.println("SENSORS:US_" + String(distance) + "cm,STATE_" + String(currentState));
}

// Funciones de actuadores (implementar según tu hardware)
void setMotorSpeed(int speed) {
  analogWrite(MOTOR_LEFT_PWM, speed);
  analogWrite(MOTOR_RIGHT_PWM, speed);
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
}

void deployRakes() {
  // Implementar control de servo/actuador para bajar rastrillos
}

void retractRakes() {
  // Implementar control de servo/actuador para subir rastrillos
}

void enableLaser() {
  digitalWrite(LASER_ENABLE, HIGH);
}

void disableLaser() {
  digitalWrite(LASER_ENABLE, LOW);
}

bool positionLaser(int x, int y) {
  // Implementar control de brazo delta
  // Retornar true cuando esté en posición
  return true; // Placeholder
}

bool returnLaserHome() {
  // Implementar retorno a posición home
  // Retornar true cuando esté en home
  return true; // Placeholder
}