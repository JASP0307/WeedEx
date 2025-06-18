#include "Ultrasonico.h"
#include <Arduino.h>

Ultrasonico::Ultrasonico(int trig, int echo)
    : TRIG(trig), ECHO(echo) {
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
}

int Ultrasonico::distancia() {
    // Asegura que el TRIG está en LOW antes de iniciar
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);

    // Pulso de 10 microsegundos en HIGH para activar el sensor
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    // Lee la duración del eco en microsegundos
    duration_us = pulseIn(ECHO, HIGH);

    // Calcula la distancia en cm
    distance_cm = (duration_us / 2.0) * 0.0343;
    
    return distance_cm;
}
