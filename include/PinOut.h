#pragma once

#include <Arduino.h>

namespace Pinout {

    //================================================================
    // SUBSISTEMA DE LOCOMOCIÓN
    //----------------------------------------------------------------
    namespace Locomocion {

        // --- MOTOR IZQUIERDO ---
        // Nota: Los nombres son genéricos. Te recomiendo renombrarlos según la función
        // exacta de cada pin en tu driver (ej: PWM, DIR1, DIR2, ENABLE).
        namespace MotorIzquierdo {
            constexpr uint8_t rPWM = 10;
            constexpr uint8_t lPWM = 11;
            constexpr uint8_t rEN = 22;
            constexpr uint8_t lEN = 23;
            constexpr uint8_t ENC_A = 2; // Pin de interrupción
            constexpr uint8_t ENC_B = 4;
        }

        // --- MOTOR DERECHO ---
        namespace MotorDerecho {
            constexpr uint8_t rPWM = 12;
            constexpr uint8_t lPWM = 13;
            constexpr uint8_t rEN = 24;
            constexpr uint8_t lEN = 25;
            constexpr uint8_t ENC_A = 3; // Pin de interrupción
            constexpr uint8_t ENC_B = 5;
        }
    }

    //================================================================
    // SUBSISTEMA DE ACTUACIÓN (BRAZO DELTA)
    //----------------------------------------------------------------
    namespace BrazoDelta {
        constexpr uint8_t SERVO_1 = 6;
        constexpr uint8_t SERVO_2 = 7;
        constexpr uint8_t SERVO_3 = 8;
    }

    //================================================================
    // SUBSISTEMA DE ACTUACIÓN (RASTRILLOS)
    //----------------------------------------------------------------
    namespace Rastrillos {
        constexpr uint8_t SERVO_4 = 9;
    }

    //================================================================
    // SUBSISTEMA DE ACTUACIÓN (LASER])
    //----------------------------------------------------------------
    namespace Laser {
        constexpr uint8_t Laser_1 = 38;
    }

    //================================================================
    // SUBSISTEMA DE SENSORES
    //----------------------------------------------------------------
    namespace Sensores {
    
        // --- SENSOR DE RUMBO (GIROSCOPIO/IMU) ---
        namespace Giroscopio {
            constexpr uint8_t SDA = 20;
            constexpr uint8_t SCL = 21;
        }

        // --- SENSOR ULTRASÓNICO FRONTAL ---
        namespace Ultra_Der {
            constexpr uint8_t TRIG = 44;
            constexpr uint8_t ECHO = 42;
        }

        // --- SENSOR ULTRASÓNICO DERECHA ---
        namespace Ultra_Front {
            constexpr uint8_t TRIG = 48;
            constexpr uint8_t ECHO = 46;
        }

        // --- SENSOR ULTRASÓNICO IZQUIERDA ---
        namespace Ultra_Izq {
            constexpr uint8_t TRIG = 52;
            constexpr uint8_t ECHO = 50;
        }
    }

} // Fin del namespace Pinout