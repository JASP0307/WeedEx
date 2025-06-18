#ifndef ULTRASONICO_H
#define ULTRASONICO_H

#include "Arduino.h"

class Ultrasonico{
    public:
        Ultrasonico(int trig, int echo);
        int distancia();

    private:  
        int TRIG,ECHO;
        int duration_us;
        int distance_cm;
        float vel = 0.017;
};
#endif