#ifndef MOTOR_MODULE_H
#define MOTOR_MODULE_H

#include <Arduino.h>

class MotorModule {
private:
    // Pines del motor BTS7960
    int pinRPWM;
    int pinLPWM;
    int pinR_EN;
    int pinL_EN;
    
    // Pines del encoder
    int pinEncoderA;
    int pinEncoderB;
    
    // Parámetros del encoder
    int pulsosPorRevolucion;
    volatile long contadorPulsos;
    
    // Variables PID
    float kp, ki, kd;
    float error, errorAnterior;
    float integral, derivada;
    float salidaPID;
    float limitePWM;
    
    // Variables de velocidad
    float velocidadObjetivo_RPM;
    float velocidadActual_RPM;
    
    // Variables de control de tiempo
    unsigned long tiempoAnterior;
    int tiempoMuestreo;
    
    // Estado del motor
    bool motorHabilitado;
    bool pidActivo;
    
    // Gestión de múltiples instancias para ISR
    static const int MAX_MOTORES = 4;
    static MotorModule* instancias[MAX_MOTORES];
    static int numInstancias;
    int indiceInstancia;
    
    // ISRs estáticos para diferentes pines
    static void ISR_encoder_pin2();
    static void ISR_encoder_pin3();
    static void ISR_encoder_pin18();
    static void ISR_encoder_pin19();
    static void ISR_encoder_pin20();
    static void ISR_encoder_pin21();
    
public:
    // Constructor
    MotorModule(int rpwm, int lpwm, int r_en, int l_en, 
                    int encA, int encB, int pulsosRev = 360, int muestreo = 100);
    
    // Destructor
    ~MotorModule();
    
    // Métodos de inicialización
    void inicializar();
    void habilitarMotor(bool estado = true);
    
    // Métodos de control PID
    void configurarPID(float kp, float ki, float kd);
    void establecerSetpoint(float rpm);
    void activarPID(bool estado = true);
    void resetearPID();
    void limitarSalidaPID(float limite = 255.0);
    
    // Método principal de actualización
    bool actualizar();
    
    // Control manual del motor (sin PID)
    void controlarMotorManual(int pwmValue);
    
    // Getters
    float obtenerVelocidadActual() const;
    float obtenerVelocidadObjetivo() const;
    float obtenerSalidaPID() const;
    float obtenerError() const;
    long obtenerPulsos() const;
    bool estaHabilitado() const;
    bool pidEstaActivo() const;
    
    // Setters
    void establecerTiempoMuestreo(int tiempo);
    void establecerPulsosPorRevolucion(int pulsos);
    void establecerKp(float valor);
    void establecerKi(float valor);
    void establecerKd(float valor);
    
    // Métodos de depuración
    void imprimirDatos(bool incluirEncabezado = false);
    void imprimirCSV();
    
    // Método para leer encoder (usado por ISR)
    void leerEncoder();
    
private:
    // Métodos internos
    void actualizarVelocidad(float deltaT);
    void calcularPID(float deltaT);
    void aplicarControlMotor();
    void registrarInstancia();
    void desregistrarInstancia();
    void configurarISR();
};

#endif