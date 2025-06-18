#ifndef SERVOMODULE_H
#define SERVOMODULE_H

#include <Arduino.h>
#include <Servo.h>

// --- Declaración de la Clase ServoModule ---
// Contiene las firmas de las funciones y las variables miembro.

class ServoModule {
public:
  // Constructor: Se ejecuta al crear un objeto ServoModule.
  ServoModule(uint8_t pin);

  // Inicializa y adjunta el servo al pin especificado.
  // Se puede proporcionar un ángulo inicial, por defecto es 90.
  void begin(int initialAngle = 90);

  // Establece el ángulo objetivo al que se moverá el servo.
  void setTarget(int angle);

  // Actualiza la posición del servo, moviéndolo un paso hacia el objetivo.
  // Devuelve 'true' si el servo todavía está en movimiento.
  // Debe ser llamado repetidamente desde una tarea RTOS.
  bool update();

  // Devuelve la posición angular actual del servo.
  int getCurrentAngle();

private:
  Servo _servo;           // La instancia del objeto de la librería Servo.
  uint8_t _pin;           // El pin al que está conectado el servo.
  
  // 'volatile' es importante en un entorno RTOS, ya que estas variables
  // pueden ser modificadas por una tarea y leídas por otra.
  volatile int _targetAngle;  // Posición a la que queremos que se mueva.
  volatile int _currentAngle; // Posición actual registrada del servo.
};

#endif // SERVOMODULE_H