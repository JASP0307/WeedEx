/*
 * main.cpp
 * Programa principal para prueba de control PID de motores
 * Robot Quitamaleza - Sistema de Movilidad
 */
#include <Arduino.h>
#include "MotorControl.h"

// Instancias de motores
Motor leftMotor(MotorConfigs::LEFT_MOTOR_CONFIG);
Motor rightMotor(MotorConfigs::RIGHT_MOTOR_CONFIG);

// Sistema de control dual
DualMotorControl robotControl(&leftMotor, &rightMotor);

// Variables de control
unsigned long lastStatusPrint = 0;
unsigned long lastDetailedPrint = 0;
const unsigned long STATUS_INTERVAL = 1000;    // Cada 1 segundo
const unsigned long DETAILED_INTERVAL = 5000;  // Cada 5 segundos

void setup() {
  Serial.begin(115200);
  Serial.println("=== ROBOT QUITAMALEZA - SISTEMA DE MOVILIDAD ===");
  Serial.println("Inicializando sistema de control PID...");
  
  // Inicializar sistema
  robotControl.begin();
  
  // Esperar un momento para estabilizar
  delay(1000);
  
  // Iniciar prueba automática
  robotControl.startTest();
  
  Serial.println("Sistema listo. Iniciando prueba PID automática...");
  Serial.println("Comandos disponibles:");
  Serial.println("  's' - Detener/Iniciar prueba");
  Serial.println("  'p' - Imprimir estado detallado");
  Serial.println("  'r' - Resetear encoders");
  Serial.println("  'h' - Ayuda");
  Serial.println();
}

void loop() {
  // Actualizar sistema de control
  robotControl.update();
  
  // Manejar comandos serie
  handleSerialCommands();
  
  // Imprimir estado periódicamente
  printPeriodicStatus();
  
  // Pequeña pausa para no saturar
  delay(10);
}

void handleSerialCommands() {
  if (Serial.available()) {
    char command = Serial.read();
    
    switch (command) {
      case 's':
      case 'S':
        if (robotControl.isTestRunning()) {
          robotControl.stopTest();
          Serial.println("Prueba detenida");
        } else {
          robotControl.startTest();
          Serial.println("Prueba reiniciada");
        }
        break;
        
      case 'p':
      case 'P':
        printDetailedStatus();
        break;
        
      case 'r':
      case 'R':
        leftMotor.resetEncoder();
        rightMotor.resetEncoder();
        Serial.println("Encoders reseteados");
        break;
        
      case 'h':
      case 'H':
        printHelp();
        break;
        
      default:
        // Ignorar otros caracteres
        break;
    }
  }
}

void printPeriodicStatus() {
  unsigned long currentTime = millis();
  
  // Estado básico cada segundo
  if (currentTime - lastStatusPrint >= STATUS_INTERVAL) {
    if (robotControl.isTestRunning()) {
      Serial.println("Tiempo: " + String(currentTime / 1000.0, 1) + "s | " +
                    "Izq: " + String(leftMotor.getCurrentRPM(), 1) + " RPM | " +
                    "Der: " + String(rightMotor.getCurrentRPM(), 1) + " RPM | " +
                    "Setpoint: " + String(leftMotor.getSetpoint(), 1) + " RPM");
    }
    lastStatusPrint = currentTime;
  }
  
  // Estado detallado cada 5 segundos
  if (currentTime - lastDetailedPrint >= DETAILED_INTERVAL) {
    if (robotControl.isTestRunning()) {
      printDetailedStatus();
    }
    lastDetailedPrint = currentTime;
  }
}

void printDetailedStatus() {
  Serial.println();
  Serial.println("############################################");
  robotControl.printSystemStatus();
  robotControl.printMotorData();
  Serial.println("############################################");
  Serial.println();
}

void printHelp() {
  Serial.println();
  Serial.println("=== COMANDOS DISPONIBLES ===");
  Serial.println("s/S - Detener/Iniciar prueba automática");
  Serial.println("p/P - Imprimir estado detallado de motores");
  Serial.println("r/R - Resetear contadores de encoders");
  Serial.println("h/H - Mostrar esta ayuda");
  Serial.println("=============================");
  Serial.println();
}

// Función para ajustar parámetros PID dinámicamente (opcional)
void adjustPIDParameters() {
  // Esta función se puede expandir para ajustar PID en tiempo real
  // Por ejemplo, leyendo valores desde Serial
  
  // Ejemplo de uso:
  // leftMotor.setPIDParams(2.5, 0.8, 0.15);
  // rightMotor.setPIDParams(2.5, 0.8, 0.15);
}

// Función de diagnóstico adicional
void systemDiagnostics() {
  Serial.println("=== DIAGNÓSTICO DEL SISTEMA ===");
  Serial.println("Memoria libre: " + String(freeMemory()) + " bytes");
  Serial.println("Uptime: " + String(millis() / 1000) + " segundos");
  Serial.println("Estado interrupciones: " + String(SREG & 0x80 ? "Habilitadas" : "Deshabilitadas"));
  Serial.println("===============================");
}

// Función para obtener memoria libre (opcional)
int freeMemory() {
  char top;
  extern char *__heap_start, *__brkval;
  int v;
  return (int)&top - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}