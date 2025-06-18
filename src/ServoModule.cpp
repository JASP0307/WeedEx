#include "ServoModule.h"

// --- Implementación de los Métodos de la Clase ServoModule ---

// Implementación del constructor.
// Usa la lista de inicialización para establecer los valores por defecto.
ServoModule::ServoModule(uint8_t pin) : _pin(pin), _currentAngle(90), _targetAngle(90) {
  // El cuerpo del constructor puede estar vacío si todo se hace en la lista de inicialización.
}

// Implementación de la función begin.
void ServoModule::begin(int initialAngle) {
  _currentAngle = constrain(initialAngle, 0, 180);
  _targetAngle = _currentAngle;
  _servo.attach(_pin);
  _servo.write(_currentAngle); // Mueve el servo físicamente a la posición inicial.
}

// Implementación de la función setTarget.
void ServoModule::setTarget(int angle) {
  // Se usa constrain para asegurar que el ángulo objetivo esté dentro de los límites físicos del servo.
  _targetAngle = constrain(angle, 0, 180);
}

// Implementación de la función update (el corazón del movimiento suave).
bool ServoModule::update() {
  if (_currentAngle < _targetAngle) {
    _currentAngle++; // Mueve un grado hacia el objetivo.
    _servo.write(_currentAngle);
    return true; // Indica que el movimiento no ha terminado.
  } 
  else if (_currentAngle > _targetAngle) {
    _currentAngle--; // Mueve un grado hacia el objetivo.
    _servo.write(_currentAngle);
    return true; // Indica que el movimiento no ha terminado.
  }
  
  // Si no entra en los 'if' anteriores, significa que _currentAngle == _targetAngle.
  return false; // Indica que el movimiento ha finalizado.
}

// Implementación de la función getCurrentAngle.
int ServoModule::getCurrentAngle() {
  return _currentAngle;
}