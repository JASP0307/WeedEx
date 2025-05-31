// MotorModule.cpp
#include "MotorModule.h"

// Variables estáticas para múltiples instancias
MotorModule* MotorModule::instancias[MAX_MOTORES] = {nullptr, nullptr, nullptr, nullptr};
int MotorModule::numInstancias = 0;

MotorModule::MotorModule(int rpwm, int lpwm, int r_en, int l_en, 
                         int encA, int encB, int pulsosRev, int muestreo) {
    // Configurar pines
    pinRPWM = rpwm;
    pinLPWM = lpwm;
    pinR_EN = r_en;
    pinL_EN = l_en;
    pinEncoderA = encA;
    pinEncoderB = encB;
    
    // Configurar parámetros
    pulsosPorRevolucion = pulsosRev;
    tiempoMuestreo = muestreo;
    
    // Inicializar variables
    contadorPulsos = 0;
    velocidadActual_RPM = 0;
    velocidadObjetivo_RPM = 0;
    tiempoAnterior = 0;
    
    // Inicializar PID con valores por defecto
    kp = 1.0;
    ki = 3.0;
    kd = 0.01;
    error = 0;
    errorAnterior = 0;
    integral = 0;
    derivada = 0;
    salidaPID = 0;
    limitePWM = 255.0;
    
    // Estado inicial
    motorHabilitado = false;
    pidActivo = false;
    indiceInstancia = -1;
    
    // Registrar esta instancia
    registrarInstancia();
}

MotorModule::~MotorModule() {
    desregistrarInstancia();
}

void MotorModule::registrarInstancia() {
    if (numInstancias < MAX_MOTORES) {
        indiceInstancia = numInstancias;
        instancias[numInstancias] = this;
        numInstancias++;
    } else {
        Serial.println("Error: Máximo número de motores alcanzado");
        indiceInstancia = -1;
    }
}

void MotorModule::desregistrarInstancia() {
    if (indiceInstancia >= 0 && indiceInstancia < numInstancias) {
        // Desplazar instancias hacia abajo
        for (int i = indiceInstancia; i < numInstancias - 1; i++) {
            instancias[i] = instancias[i + 1];
            if (instancias[i] != nullptr) {
                instancias[i]->indiceInstancia = i;
            }
        }
        instancias[numInstancias - 1] = nullptr;
        numInstancias--;
        indiceInstancia = -1;
    }
}

void MotorModule::inicializar() {
    // Configurar pines del encoder
    pinMode(pinEncoderA, INPUT_PULLUP);
    pinMode(pinEncoderB, INPUT_PULLUP);
    
    // Configurar ISR según el pin
    configurarISR();
    
    // Configurar pines del motor
    pinMode(pinRPWM, OUTPUT);
    pinMode(pinLPWM, OUTPUT);
    pinMode(pinR_EN, OUTPUT);
    pinMode(pinL_EN, OUTPUT);
    
    // Habilitar motor por defecto
    habilitarMotor(true);
    
    Serial.print("Motor ");
    Serial.print(indiceInstancia);
    Serial.print(" inicializado en pin ");
    Serial.println(pinEncoderA);
}

void MotorModule::configurarISR() {
    switch (pinEncoderA) {
        case 2:
            attachInterrupt(digitalPinToInterrupt(2), ISR_encoder_pin2, RISING);
            break;
        case 3:
            attachInterrupt(digitalPinToInterrupt(3), ISR_encoder_pin3, RISING);
            break;
        case 18:
            attachInterrupt(digitalPinToInterrupt(18), ISR_encoder_pin18, RISING);
            break;
        case 19:
            attachInterrupt(digitalPinToInterrupt(19), ISR_encoder_pin19, RISING);
            break;
        case 20:
            attachInterrupt(digitalPinToInterrupt(20), ISR_encoder_pin20, RISING);
            break;
        case 21:
            attachInterrupt(digitalPinToInterrupt(21), ISR_encoder_pin21, RISING);
            break;
        default:
            Serial.print("Error: Pin ");
            Serial.print(pinEncoderA);
            Serial.println(" no soporta interrupciones");
            break;
    }
}

void MotorModule::habilitarMotor(bool estado) {
    motorHabilitado = estado;
    digitalWrite(pinR_EN, estado ? HIGH : LOW);
    digitalWrite(pinL_EN, estado ? HIGH : LOW);
    
    if (!estado) {
        controlarMotorManual(0);
        resetearPID();
    }
}

void MotorModule::configurarPID(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
}

void MotorModule::establecerSetpoint(float rpm) {
    velocidadObjetivo_RPM = rpm;
}

void MotorModule::activarPID(bool estado) {
    pidActivo = estado;
    if (estado && !motorHabilitado) {
        habilitarMotor(true);
    }
    if (!estado) {
        resetearPID();
        controlarMotorManual(0);
    }
}

void MotorModule::resetearPID() {
    error = 0;
    errorAnterior = 0;
    integral = 0;
    derivada = 0;
    salidaPID = 0;
}

void MotorModule::limitarSalidaPID(float limite) {
    limitePWM = abs(limite);
}

bool MotorModule::actualizar() {
    unsigned long tiempoActual = millis();
    if (tiempoActual - tiempoAnterior >= tiempoMuestreo) {
        float deltaT = (tiempoActual - tiempoAnterior) / 1000.0;
        tiempoAnterior = tiempoActual;
        
        // Actualizar velocidad
        actualizarVelocidad(deltaT);
        
        // Calcular PID si está activo
        if (pidActivo && motorHabilitado) {
            calcularPID(deltaT);
            aplicarControlMotor();
        }
        
        return true;
    }
    return false;
}

void MotorModule::controlarMotorManual(int pwmValue) {
    if (!motorHabilitado) {
        return;
    }
    
    pwmValue = constrain(pwmValue, -255, 255);
    
    if (pwmValue > 0) {
        analogWrite(pinRPWM, pwmValue);
        analogWrite(pinLPWM, 0);
    } else if (pwmValue < 0) {
        analogWrite(pinRPWM, 0);
        analogWrite(pinLPWM, -pwmValue);
    } else {
        analogWrite(pinRPWM, 0);
        analogWrite(pinLPWM, 0);
    }
}

void MotorModule::actualizarVelocidad(float deltaT) {
    // Leer encoder de forma atómica
    noInterrupts();
    long pulsos = contadorPulsos;
    contadorPulsos = 0;
    interrupts();
    
    // Calcular RPM
    float revoluciones = (float)pulsos / pulsosPorRevolucion;
    velocidadActual_RPM = (revoluciones * 60.0) / deltaT;
}

void MotorModule::calcularPID(float deltaT) {
    // Calcular error
    error = velocidadObjetivo_RPM - velocidadActual_RPM;
    
    // Término integral
    integral += error * deltaT;
    
    // Término derivativo
    derivada = (error - errorAnterior) / deltaT;
    
    // Calcular salida PID
    salidaPID = kp * error + ki * integral + kd * derivada;
    
    // Limitar salida
    salidaPID = constrain(salidaPID, -limitePWM, limitePWM);
    
    // Guardar error para próxima iteración
    errorAnterior = error;
}

void MotorModule::aplicarControlMotor() {
    controlarMotorManual((int)salidaPID);
}

// Getters
float MotorModule::obtenerVelocidadActual() const {
    return velocidadActual_RPM;
}

float MotorModule::obtenerVelocidadObjetivo() const {
    return velocidadObjetivo_RPM;
}

float MotorModule::obtenerSalidaPID() const {
    return salidaPID;
}

float MotorModule::obtenerError() const {
    return error;
}

long MotorModule::obtenerPulsos() const {
    noInterrupts();
    long pulsos = contadorPulsos;
    interrupts();
    return pulsos;
}

bool MotorModule::estaHabilitado() const {
    return motorHabilitado;
}

bool MotorModule::pidEstaActivo() const {
    return pidActivo;
}

// Setters
void MotorModule::establecerTiempoMuestreo(int tiempo) {
    tiempoMuestreo = tiempo;
}

void MotorModule::establecerPulsosPorRevolucion(int pulsos) {
    pulsosPorRevolucion = pulsos;
}

void MotorModule::establecerKp(float valor) {
    kp = valor;
}

void MotorModule::establecerKi(float valor) {
    ki = valor;
}

void MotorModule::establecerKd(float valor) {
    kd = valor;
}

// Métodos de depuración
void MotorModule::imprimirDatos(bool incluirEncabezado) {
    if (incluirEncabezado) {
        Serial.println("Motor,Setpoint,RPM,PWM,Error");
    }
    Serial.print(indiceInstancia);
    Serial.print(",");
    Serial.print(velocidadObjetivo_RPM);
    Serial.print(",");
    Serial.print(velocidadActual_RPM);
    Serial.print(",");
    Serial.print((int)salidaPID);
    Serial.print(",");
    Serial.println(error);
}

void MotorModule::imprimirCSV() {
    Serial.print("M");
    Serial.println(indiceInstancia);
    Serial.print(">Setpoint:");
    Serial.println(velocidadObjetivo_RPM);

    Serial.print(">ActualVal:");
    Serial.println(velocidadActual_RPM);

    Serial.print(",");
    Serial.println((int)salidaPID);
}

void MotorModule::leerEncoder() {
    bool estadoB = digitalRead(pinEncoderB);
    if (estadoB) {
        contadorPulsos++;
    } else {
        contadorPulsos--;
    }
}

// ISRs estáticos para diferentes pines
void MotorModule::ISR_encoder_pin2() {
    for (int i = 0; i < numInstancias; i++) {
        if (instancias[i] != nullptr && instancias[i]->pinEncoderA == 2) {
            instancias[i]->leerEncoder();
            break;
        }
    }
}

void MotorModule::ISR_encoder_pin3() {
    for (int i = 0; i < numInstancias; i++) {
        if (instancias[i] != nullptr && instancias[i]->pinEncoderA == 3) {
            instancias[i]->leerEncoder();
            break;
        }
    }
}

void MotorModule::ISR_encoder_pin18() {
    for (int i = 0; i < numInstancias; i++) {
        if (instancias[i] != nullptr && instancias[i]->pinEncoderA == 18) {
            instancias[i]->leerEncoder();
            break;
        }
    }
}

void MotorModule::ISR_encoder_pin19() {
    for (int i = 0; i < numInstancias; i++) {
        if (instancias[i] != nullptr && instancias[i]->pinEncoderA == 19) {
            instancias[i]->leerEncoder();
            break;
        }
    }
}

void MotorModule::ISR_encoder_pin20() {
    for (int i = 0; i < numInstancias; i++) {
        if (instancias[i] != nullptr && instancias[i]->pinEncoderA == 20) {
            instancias[i]->leerEncoder();
            break;
        }
    }
}

void MotorModule::ISR_encoder_pin21() {
    for (int i = 0; i < numInstancias; i++) {
        if (instancias[i] != nullptr && instancias[i]->pinEncoderA == 21) {
            instancias[i]->leerEncoder();
            break;
        }
    }
}