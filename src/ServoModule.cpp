#include "ServoModule.h"

ServoModule::ServoModule(uint8_t pin, int delayPaso) : 
  pin(pin), 
  anguloActual(90), 
  anguloDestino(90),
  delayPorPaso(delayPaso),
  ultimoUpdate(0),
  enMovimiento(false),
  inicializado(false) {
  
  // Validar pin
  if (pin < 2 || pin > 53) {
    Serial.println("Error: Pin inválido para servo");
    return;
  }
  
  // Validar delay
  if (delayPaso < 5) {
    delayPorPaso = 15; // Valor por defecto seguro
  }
  
  // Inicializar servo
  if (servo.attach(pin)) {
    servo.write(anguloActual);
    inicializado = true;
    
    // Dar tiempo inicial sin bloquear FreeRTOS
    ultimoUpdate = millis();
    delay(500); // Solo en constructor es aceptable
    
    Serial.print("Servo inicializado en pin ");
    Serial.println(pin);
  } else {
    Serial.println("Error: No se pudo inicializar el servo");
  }
}

bool ServoModule::moverASuavemente(int anguloObjetivo) {
  if (!inicializado) {
    return false;
  }
  
  // Validar y ajustar rango
  anguloObjetivo = constrain(anguloObjetivo, 0, 180);
  
  // Si es un nuevo objetivo, actualizar destino
  if (anguloDestino != anguloObjetivo) {
    anguloDestino = anguloObjetivo;
    enMovimiento = true;
  }
  
  // Si ya está en la posición objetivo
  if (anguloActual == anguloDestino) {
    enMovimiento = false;
    return true; // Movimiento completado
  }
  
  // Verificar si es tiempo de dar el siguiente paso
  unsigned long tiempoActual = millis();
  if (tiempoActual - ultimoUpdate >= delayPorPaso) {
    // Calcular dirección del movimiento
    if (anguloActual < anguloDestino) {
      anguloActual++;
    } else if (anguloActual > anguloDestino) {
      anguloActual--;
    }
    
    // Actualizar posición del servo
    servo.write(anguloActual);
    ultimoUpdate = tiempoActual;
    
    // Verificar si llegó al destino
    if (anguloActual == anguloDestino) {
      enMovimiento = false;
      return true; // Movimiento completado
    }
  }
  
  return false; // Aún en movimiento
}

void ServoModule::setAnguloActual(int angulo) {
  if (!inicializado) {
    return;
  }
  
  angulo = constrain(angulo, 0, 180);
  anguloActual = angulo;
  anguloDestino = angulo;
  enMovimiento = false;
  servo.write(angulo);
  ultimoUpdate = millis();
}

int ServoModule::getAnguloActual() {
  return anguloActual;
}

bool ServoModule::estaEnMovimiento() {
  return enMovimiento;
}

void ServoModule::detener() {
  // Detener en la posición actual
  anguloDestino = anguloActual;
  enMovimiento = false;
}