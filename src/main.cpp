#include <Arduino.h>

// Versión simplificada para pruebas de comunicación serial
// Arduino Mega Robot Quitamaleza - Prueba de Estados

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
const unsigned long ULTRASONIC_INTERVAL = 100;   // Check cada 100ms
const unsigned long SERIAL_INTERVAL = 10;        // Check cada 10ms
const unsigned long RAKE_DURATION = 2000;        // 2 segundos
const unsigned long LASER_DURATION = 3000;       // 3 segundos

// Variables de operación
int navigationSpeed = 0;
int laserX = 0, laserY = 0;
bool obstacleDetected = false;
String serialBuffer = "";

// Simulación de sensores
int simulatedDistance = 50;
unsigned long lastObstacleSimulation = 0;

void setup() {
  Serial.begin(115200);
  
  // Mensaje de inicialización
  Serial.println("=== ARDUINO MEGA ROBOT QUITAMALEZA - TEST ===");
  delay(1000);
  Serial.println("STATUS:ARDUINO_READY");
  currentState = IDLE;
  
  // Seed para números aleatorios
  randomSeed(analogRead(A0));
}

void loop() {
  unsigned long currentTime = millis();
  
  // Monitoreo de ultrasonidos simulados
  if (currentTime - lastUltrasonicCheck >= ULTRASONIC_INTERVAL) {
    checkSimulatedUltrasonics();
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
  // Estado de reposo - no hacer nada especial
}

void handleNavigatingState() {
  // Mantener velocidad de navegación
  // En la implementación real aquí controlarías los motores
}

void handleRakeOperationState() {
  static bool rakesDeployed = false;
  
  if (!rakesDeployed) {
    // Simular despliegue de rastrillos
    rakesDeployed = true;
    stateTimer = millis();
    Serial.println("STATUS:RAKES_DEPLOYED");
    Serial.println("DEBUG:Simulando operación de rastrillos...");
  }
  
  // Verificar timeout
  if (millis() - stateTimer >= RAKE_DURATION) {
    rakesDeployed = false;
    changeState(NAVIGATING);
    Serial.println("STATUS:RAKE_COMPLETE");
    Serial.println("DEBUG:Rastrillos completados, volviendo a navegación");
  }
}

void handleLaserPositioningState() {
  static bool positioning = false;
  
  if (!positioning) {
    positioning = true;
    stateTimer = millis();
    Serial.println("DEBUG:Posicionando láser en (" + String(laserX) + "," + String(laserY) + ")");
  }
  
  // Simular tiempo de posicionamiento (1 segundo)
  if (millis() - stateTimer >= 1000) {
    positioning = false;
    changeState(LASER_OPERATION);
    Serial.println("STATUS:LASER_POSITIONED");
  }
}

void handleLaserOperationState() {
  static bool laserStarted = false;
  
  if (!laserStarted) {
    laserStarted = true;
    stateTimer = millis();
    Serial.println("STATUS:LASER_ON");
    Serial.println("DEBUG:Láser disparando por " + String(LASER_DURATION/1000) + " segundos");
  }
  
  // Verificar timeout
  if (millis() - stateTimer >= LASER_DURATION) {
    laserStarted = false;
    changeState(LASER_RETURNING);
    Serial.println("STATUS:LASER_OFF");
  }
}

void handleLaserReturningState() {
  static bool returning = false;
  
  if (!returning) {
    returning = true;
    stateTimer = millis();
    Serial.println("DEBUG:Retornando láser a posición home");
  }
  
  // Simular tiempo de retorno (1 segundo)
  if (millis() - stateTimer >= 1000) {
    returning = false;
    changeState(NAVIGATING);
    Serial.println("STATUS:LASER_HOME");
  }
}

void handleEmergencyStopState() {
  static bool emergencyReported = false;
  
  if (!emergencyReported) {
    Serial.println("DEBUG:PARADA DE EMERGENCIA - Obstáculo a " + String(simulatedDistance) + "cm");
    emergencyReported = true;
  }
  
  // Solo salir si no hay obstáculo
  if (!obstacleDetected) {
    emergencyReported = false;
    changeState(previousState);
    Serial.println("STATUS:EMERGENCY_CLEARED");
    Serial.println("DEBUG:Emergencia despejada, volviendo a estado anterior");
  }
}

void processSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        handleSerialCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
    }
  }
}

void handleSerialCommand(String command) {
  Serial.println("DEBUG:Comando recibido: " + command);
  
  if (command.startsWith("START_NAV:")) {
    navigationSpeed = command.substring(10).toInt();
    changeState(NAVIGATING);
    Serial.println("STATUS:NAV_STARTED");
    Serial.println("DEBUG:Navegación iniciada a velocidad " + String(navigationSpeed));
    
  } else if (command == "DEPLOY_RAKES") {
    if (currentState == NAVIGATING) {
      changeState(RAKE_OPERATION);
      Serial.println("DEBUG:Iniciando operación de rastrillos");
    } else {
      Serial.println("DEBUG:No se puede desplegar rastrillos - no navegando");
    }
    
  } else if (command.startsWith("LASER_POS:")) {
    int commaIndex = command.indexOf(',');
    if (commaIndex > 0) {
      laserX = command.substring(10, commaIndex).toInt();
      laserY = command.substring(commaIndex + 1).toInt();
      if (currentState == NAVIGATING) {
        changeState(LASER_POSITIONING);
        Serial.println("DEBUG:Iniciando posicionamiento láser");
      } else {
        Serial.println("DEBUG:No se puede posicionar láser - no navegando");
      }
    }
    
  } else if (command == "EMERGENCY_STOP") {
    changeState(EMERGENCY_STOP);
    Serial.println("DEBUG:Parada de emergencia manual activada");
    
  } else if (command == "GET_SENSORS") {
    reportSensorStatus();
    
  } else if (command == "STOP") {
    changeState(IDLE);
    Serial.println("STATUS:STOPPED");
    Serial.println("DEBUG:Sistema detenido - Estado IDLE");
    
  } else {
    Serial.println("DEBUG:Comando no reconocido: " + command);
  }
}

void checkSimulatedUltrasonics() {
  unsigned long currentTime = millis();
  
  // Simular cambio de distancia cada 2 segundos
  if (currentTime - lastObstacleSimulation >= 2000) {
    // 10% probabilidad de obstáculo
    if (random(100) < 10) {
      simulatedDistance = random(5, 20);  // Obstáculo cerca (5-19cm)
    } else {
      simulatedDistance = random(30, 100); // Sin obstáculo (30-100cm)
    }
    lastObstacleSimulation = currentTime;
  }
  
  // Detectar obstáculo
  if (simulatedDistance < 20 && simulatedDistance > 0) {
    if (!obstacleDetected) {
      obstacleDetected = true;
      if (currentState != EMERGENCY_STOP) {
        previousState = currentState;
        changeState(EMERGENCY_STOP);
        Serial.println("OBSTACLE:distance_" + String(simulatedDistance) + "cm");
      }
    }
  } else {
    obstacleDetected = false;
  }
}

void changeState(RobotState newState) {
  if (newState != currentState) {
    Serial.println("DEBUG:Cambio de estado: " + String(currentState) + " -> " + String(newState));
    previousState = currentState;
    currentState = newState;
    stateTimer = millis();
  }
}

void reportSensorStatus() {
  String stateNames[] = {"IDLE", "NAVIGATING", "RAKE_OPERATION", 
                        "LASER_POSITIONING", "LASER_OPERATION", 
                        "LASER_RETURNING", "EMERGENCY_STOP"};
  
  Serial.println("SENSORS:US_" + String(simulatedDistance) + "cm,STATE_" + stateNames[currentState]);
  Serial.println("DEBUG:Velocidad nav: " + String(navigationSpeed) + 
                 ", Láser pos: (" + String(laserX) + "," + String(laserY) + ")");
}