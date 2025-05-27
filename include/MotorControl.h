/*
 * MotorControl.h
 * Sistema de control PID para motores con drivers BTS7960 y encoders incrementales
 * Robot Quitamaleza - Control de Movilidad
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// Constantes del sistema
#define ENCODER_CPR 1440              // Pulsos por revolución
#define PID_SAMPLE_TIME_MS 50         // Tiempo de muestreo PID (50ms)
#define SETPOINT_CHANGE_INTERVAL 5000 // Cambio de setpoint cada 5 segundos

// Estructura para parámetros PID
struct PIDParams {
  double kp;
  double ki;
  double kd;
  double output_min;
  double output_max;
};

// Estructura para configuración del motor
struct MotorConfig {
  // Pines BTS7960
  uint8_t pwm_pin;        // PWM para velocidad
  uint8_t dir1_pin;       // Dirección 1 (L_EN)
  uint8_t dir2_pin;       // Dirección 2 (R_EN)
  uint8_t enable_pin;     // Enable (opcional)
  
  // Pines encoder
  uint8_t encoder_a_pin;
  uint8_t encoder_b_pin;
  
  // Parámetros PID
  PIDParams pid_params;
  
  // Identificación
  String motor_name;
};

// Clase para control individual de motor
class Motor {
private:
  // Configuración
  MotorConfig config;
  
  // Variables PID
  double setpoint_rpm;
  double current_rpm;
  double output_pwm;
  double pid_error;
  double pid_integral;
  double pid_derivative;
  double pid_last_error;
  unsigned long last_pid_time;
  
  // Variables encoder
  volatile long encoder_count;
  volatile long last_encoder_count;
  unsigned long last_rpm_time;
  
  // Control de tiempo
  unsigned long last_sample_time;
  
  // Estado del motor
  bool motor_enabled;
  int8_t motor_direction; // -1, 0, 1
  
public:
  Motor(const MotorConfig& cfg);
  
  // Inicialización
  void begin();
  void attachEncoderInterrupts();
  
  // Control de velocidad
  void setSetpoint(double rpm);
  double getSetpoint() const { return setpoint_rpm; }
  double getCurrentRPM() const { return current_rpm; }
  
  // PID
  void updatePID();
  void setPIDParams(double kp, double ki, double kd);
  void resetPID();
  
  // Control de motor
  void enable();
  void disable();
  void stop();
  void setDirection(int8_t dir); // -1: reversa, 0: stop, 1: adelante
  
  // Encoder
  void encoderISR(); // Llamar desde ISR
  void updateRPM();
  long getEncoderCount() const { return encoder_count; }
  void resetEncoder();
  
  // Utilidades
  void printStatus() const;
  bool isEnabled() const { return motor_enabled; }
  double getOutput() const { return output_pwm; }
  
private:
  void applyPWM();
  void updateDirection();
};

// Clase para control dual de motores (robot diferencial)
class DualMotorControl {
private:
  Motor* left_motor;
  Motor* right_motor;
  
  // Setpoints de prueba
  double test_setpoints[6] = {0, 50, 100, 75, -50, 25}; // RPM
  uint8_t current_setpoint_index;
  unsigned long last_setpoint_change;
  
  // Variables de sistema
  bool system_enabled;
  unsigned long system_start_time;
  
public:
  DualMotorControl(Motor* left, Motor* right);
  
  // Inicialización
  void begin();
  
  // Control principal
  void update(); // Llamar en loop principal
  
  // Control de velocidad
  void setSpeed(double left_rpm, double right_rpm);
  void setLinearSpeed(double linear_rpm, double angular_rpm = 0);
  void stop();
  
  // Test automático
  void startTest();
  void stopTest();
  bool isTestRunning() const;
  
  // Estado del sistema
  void enable();
  void disable();
  bool isEnabled() const { return system_enabled; }
  
  // Utilidades
  void printSystemStatus() const;
  void printMotorData() const;
  
private:
  void updateTestSetpoints();
};

// Variables globales para ISR (necesarias para attachInterrupt)
extern Motor* g_left_motor_ptr;
extern Motor* g_right_motor_ptr;

// Funciones ISR globales
void leftEncoderISR();
void rightEncoderISR();

// Configuraciones predefinidas
namespace MotorConfigs {
  extern const MotorConfig LEFT_MOTOR_CONFIG;
  extern const MotorConfig RIGHT_MOTOR_CONFIG;
}

#endif // MOTOR_CONTROL_H