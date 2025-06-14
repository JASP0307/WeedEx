#ifndef SERVOMODULE_H
#define SERVOMODULE_H

#include <Arduino.h>
#include <Servo.h>

class ServoModule {
  public:
    ServoModule(uint8_t pin, int delayPaso = 15);
    bool moverASuavemente(int anguloObjetivo);
    void setAnguloActual(int angulo);
    int getAnguloActual();
    bool estaEnMovimiento();
    void detener();

  private:
    Servo servo;
    uint8_t pin;
    int anguloActual;
    int anguloDestino;
    int delayPorPaso;
    unsigned long ultimoUpdate;
    bool enMovimiento;
    bool inicializado;
};

#endif