#ifndef SERVOMODULE_H
#define SERVOMODULE_H

#include <Arduino.h>
#include <Servo.h>

class ServoModule {
public:
    // --- Miembros Públicos ---
    ServoModule(uint8_t pin);            // Constructor
    void begin(int initialAngle = 170);   // Inicializador del servo
    void setTarget(int angle);           // Establece el ángulo objetivo
    bool update();                       // Actualiza la posición del servo (debe llamarse en loop)
    int getCurrentAngle();               // Devuelve el ángulo actual

private:
    // --- Miembros Privados ---
    Servo _servo;        // La instancia del objeto Servo
    uint8_t _pin;        // El pin al que está conectado el servo
    int _currentAngle;   // El ángulo actual del servo
    int _targetAngle;    // El ángulo al que queremos que se mueva el servo
    bool _isAttached;    //
};

#endif // SERVOMODULE_H