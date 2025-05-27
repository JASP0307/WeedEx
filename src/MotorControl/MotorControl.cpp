/*
 * MotorControl.cpp
 * Implementación del sistema de control PID para motores BTS7960
 */

#include "MotorControl.h"

// Variables globales para ISR
Motor* g_left_motor_ptr = nullptr;
Motor* g_right_motor_ptr = nullptr;

// Funciones ISR globales
void leftEncoderISR() {
  if (g_left_motor_ptr) {
    g_left_motor_ptr->encoderISR();
  }
}

void rightEncoderISR() {
  if (g_right_motor_ptr) {
    g_right_motor_ptr->encoderISR();
  }
}

// Configuraciones predefinidas
namespace MotorConfigs {
  const MotorConfig LEFT_MOTOR_CONFIG = {
    // Pines BTS7960 - Motor Izquierdo
    .pwm_pin = 5,
    .dir1_pin = 6,      // L_EN
    .dir2_pin = 7,      // R_EN
    .enable_pin = 8,    // Enable (opcional)
    
    // Pines encoder
    .encoder_a_pin = 2, // Pin de interrupción
    .encoder_b_pin = 4,
    
    // Parámetros PID iniciales
    .pid_params = {
      .kp = 2.0,
      .ki = 0.5,
      .kd = 0.1,
      .output_min = -255.0,
      .output_max = 255.0
    },
    
    .motor_name = "Motor_Izquierdo"
  };
  
  const MotorConfig RIGHT_MOTOR_CONFIG = {
    // Pines BTS7960 - Motor Derecho
    .pwm_pin = 9,
    .dir1_pin = 10,     // L_EN
    .dir2_pin = 11,     // R_EN
    .enable_pin = 12,   // Enable (opcional)
    
    // Pines encoder
    .encoder_a_pin = 3, // Pin de interrupción
    .encoder_b_pin = 13,
    
    // Parámetros PID iniciales
    .pid_params = {
      .kp = 2.0,
      .ki = 0.5,
      .kd = 0.1,
      .output_min = -255.0,
      .output_max = 255.0
    },
    
    .motor_name = "Motor_Derecho"
  };
}

// ========== IMPLEMENTACIÓN CLASE MOTOR ==========

Motor::Motor(const MotorConfig& cfg) : config(cfg) {
  // Inicializar variables
  setpoint_rpm = 0.0;
  current_rpm = 0.0;
  output_pwm = 0.0;
  pid_error = 0.0;
  pid_integral = 0.0;
  pid_derivative = 0.0;
  pid_last_error = 0.0;
  
  encoder_count = 0;
  last_encoder_count = 0;
  
  motor_enabled = false;
  motor_direction = 0;
  
  last_pid_time = 0;
  last_rpm_time = 0;
  last_sample_time = 0;
}

void Motor::begin() {
  // Configurar pines BTS7960
  pinMode(config.pwm_pin, OUTPUT);
  pinMode(config.dir1_pin, OUTPUT);
  pinMode(config.dir2_pin, OUTPUT);
  
  if (config.enable_pin != 255) {
    pinMode(config.enable_pin, OUTPUT);
  }
  
  // Configurar pines encoder
  pinMode(config.encoder_a_pin, INPUT_PULLUP);
  pinMode(config.encoder_b_pin, INPUT_PULLUP);
  
  // Estado inicial seguro
  stop();
  
  Serial.println("Motor " + config.motor_name + " inicializado");
}

void Motor::attachEncoderInterrupts() {
  // Las interrupciones se configuran externamente debido a limitaciones
  // de attachInterrupt con métodos de clase
  Serial.println("Configurar interrupciones externamente para " + config.motor_name);
}

void Motor::setSetpoint(double rpm) {
  setpoint_rpm = rpm;
  
  // Resetear PID para evitar saltos
  if (abs(rpm - setpoint_rpm) > 10) {
    resetPID();
  }
}

void Motor::updatePID() {
  unsigned long current_time = millis();
  
  // Verificar tiempo de muestreo
  if (current_time - last_sample_time < PID_SAMPLE_TIME_MS) {
    return;
  }
  
  // Actualizar RPM actual
  updateRPM();
  
  // Calcular error
  pid_error = setpoint_rpm - current_rpm;
  
  // Calcular tiempo transcurrido
  double dt = (current_time - last_pid_time) / 1000.0; // Segundos
  
  if (dt > 0) {
    // Término proporcional
    double p_term = config.pid_params.kp * pid_error;
    
    // Término integral
    pid_integral += pid_error * dt;
    
    // Anti-windup para término integral
    double max_integral = config.pid_params.output_max / config.pid_params.ki;
    pid_integral = constrain(pid_integral, -max_integral, max_integral);
    double i_term = config.pid_params.ki * pid_integral;
    
    // Término derivativo
    pid_derivative = (pid_error - pid_last_error) / dt;
    double d_term = config.pid_params.kd * pid_derivative;
    
    // Salida PID
    output_pwm = p_term + i_term + d_term;
    
    // Limitar salida
    output_pwm = constrain(output_pwm, 
                          config.pid_params.output_min, 
                          config.pid_params.output_max);
    
    // Aplicar salida si está habilitado
    if (motor_enabled) {
      applyPWM();
    }
    
    // Actualizar variables para siguiente iteración
    pid_last_error = pid_error;
  }
  
  last_pid_time = current_time;
  last_sample_time = current_time;
}

void Motor::applyPWM() {
  // Determinar dirección basada en signo de output_pwm
  if (output_pwm > 0) {
    setDirection(1);  // Adelante
    analogWrite(config.pwm_pin, abs(output_pwm));
  } else if (output_pwm < 0) {
    setDirection(-1); // Reversa
    analogWrite(config.pwm_pin, abs(output_pwm));
  } else {
    setDirection(0);  // Stop
    analogWrite(config.pwm_pin, 0);
  }
}

void Motor::setDirection(int8_t dir) {
  motor_direction = dir;
  updateDirection();
}

void Motor::updateDirection() {
  switch (motor_direction) {
    case 1:  // Adelante
      digitalWrite(config.dir1_pin, HIGH);
      digitalWrite(config.dir2_pin, LOW);
      break;
      
    case -1: // Reversa
      digitalWrite(config.dir1_pin, LOW);
      digitalWrite(config.dir2_pin, HIGH);
      break;
      
    default: // Stop (0)
      digitalWrite(config.dir1_pin, LOW);
      digitalWrite(config.dir2_pin, LOW);
      break;
  }
}

void Motor::enable() {
  motor_enabled = true;
  
  if (config.enable_pin != 255) {
    digitalWrite(config.enable_pin, HIGH);
  }
  
  Serial.println(config.motor_name + " habilitado");
}

void Motor::disable() {
  motor_enabled = false;
  stop();
  
  if (config.enable_pin != 255) {
    digitalWrite(config.enable_pin, LOW);
  }
  
  Serial.println(config.motor_name + " deshabilitado");
}

void Motor::stop() {
  output_pwm = 0;
  setpoint_rpm = 0;
  setDirection(0);
  analogWrite(config.pwm_pin, 0);
  resetPID();
}

void Motor::encoderISR() {
  // Leer estado de encoder A y B
  bool a_state = digitalRead(config.encoder_a_pin);
  bool b_state = digitalRead(config.encoder_b_pin);
  
  // Determinar dirección basada en cuadratura
  if (a_state == b_state) {
    encoder_count++;
  } else {
    encoder_count--;
  }
}

void Motor::updateRPM() {
  unsigned long current_time = millis();
  unsigned long time_diff = current_time - last_rpm_time;
  
  if (time_diff >= 100) { // Actualizar cada 100ms mínimo
    long count_diff = encoder_count - last_encoder_count;
    
    // Calcular RPM
    // RPM = (pulsos * 60 * 1000) / (CPR * tiempo_ms)
    current_rpm = (count_diff * 60.0 * 1000.0) / (ENCODER_CPR * time_diff);
    
    // Actualizar variables para siguiente cálculo
    last_encoder_count = encoder_count;
    last_rpm_time = current_time;
  }
}

void Motor::setPIDParams(double kp, double ki, double kd) {
  config.pid_params.kp = kp;
  config.pid_params.ki = ki;
  config.pid_params.kd = kd;
  resetPID();
}

void Motor::resetPID() {
  pid_integral = 0.0;
  pid_derivative = 0.0;
  pid_last_error = 0.0;
  last_pid_time = millis();
}

void Motor::resetEncoder() {
  noInterrupts();
  encoder_count = 0;
  last_encoder_count = 0;
  interrupts();
}

void Motor::printStatus() const {
  Serial.println("=== " + config.motor_name + " ===");
  Serial.println("Setpoint: " + String(setpoint_rpm, 2) + " RPM");
  Serial.println("Actual: " + String(current_rpm, 2) + " RPM");
  Serial.println("Error: " + String(pid_error, 2) + " RPM");
  Serial.println("Output: " + String(output_pwm, 2) + " PWM");
  Serial.println("Encoder: " + String(encoder_count) + " pulsos");
  Serial.println("Habilitado: " + String(motor_enabled ? "SI" : "NO"));
  Serial.println("Dirección: " + String(motor_direction));
  Serial.println();
}

// ========== IMPLEMENTACIÓN CLASE DUAL MOTOR CONTROL ==========

DualMotorControl::DualMotorControl(Motor* left, Motor* right) {
  left_motor = left;
  right_motor = right;
  current_setpoint_index = 0;
  last_setpoint_change = 0;
  system_enabled = false;
  system_start_time = 0;
}

void DualMotorControl::begin() {
  if (left_motor) left_motor->begin();
  if (right_motor) right_motor->begin();
  
  // Configurar punteros globales para ISR
  g_left_motor_ptr = left_motor;
  g_right_motor_ptr = right_motor;
  
  // Configurar interrupciones de encoder
  attachInterrupt(digitalPinToInterrupt(2), leftEncoderISR, CHANGE);  // Motor izquierdo
  attachInterrupt(digitalPinToInterrupt(3), rightEncoderISR, CHANGE); // Motor derecho
  
  Serial.println("Sistema de control dual inicializado");
}

void DualMotorControl::update() {
  if (!system_enabled) return;
  
  // Actualizar PID de ambos motores
  if (left_motor) left_motor->updatePID();
  if (right_motor) right_motor->updatePID();
  
  // Actualizar setpoints de prueba
  updateTestSetpoints();
}

void DualMotorControl::updateTestSetpoints() {
  unsigned long current_time = millis();
  
  if (current_time - last_setpoint_change >= SETPOINT_CHANGE_INTERVAL) {
    // Cambiar al siguiente setpoint
    double new_setpoint = test_setpoints[current_setpoint_index];
    
    Serial.println("=== CAMBIO DE SETPOINT ===");
    Serial.println("Nuevo setpoint: " + String(new_setpoint) + " RPM");
    Serial.println("Tiempo: " + String(current_time / 1000.0, 2) + " s");
    
    setSpeed(new_setpoint, new_setpoint);
    
    // Avanzar al siguiente setpoint
    current_setpoint_index = (current_setpoint_index + 1) % 6;
    last_setpoint_change = current_time;
  }
}

void DualMotorControl::setSpeed(double left_rpm, double right_rpm) {
  if (left_motor) left_motor->setSetpoint(left_rpm);
  if (right_motor) right_motor->setSetpoint(right_rpm);
}

void DualMotorControl::setLinearSpeed(double linear_rpm, double angular_rpm) {
  // Para robot diferencial: 
  // left_rpm = linear_rpm + angular_rpm
  // right_rpm = linear_rpm - angular_rpm
  double left_rpm = linear_rpm + angular_rpm;
  double right_rpm = linear_rpm - angular_rpm;
  
  setSpeed(left_rpm, right_rpm);
}

void DualMotorControl::stop() {
  if (left_motor) left_motor->stop();
  if (right_motor) right_motor->stop();
  Serial.println("Sistema detenido");
}

void DualMotorControl::enable() {
  system_enabled = true;
  if (left_motor) left_motor->enable();
  if (right_motor) right_motor->enable();
  Serial.println("Sistema habilitado");
}

void DualMotorControl::disable() {
  system_enabled = false;
  stop();
  if (left_motor) left_motor->disable();
  if (right_motor) right_motor->disable();
  Serial.println("Sistema deshabilitado");
}

void DualMotorControl::startTest() {
  current_setpoint_index = 0;
  last_setpoint_change = millis();
  system_start_time = millis();
  enable();
  
  Serial.println("=== INICIO DE PRUEBA PID ===");
  Serial.println("Secuencia de setpoints (RPM): 0, 50, 100, 75, -50, 25");
  Serial.println("Cambio cada " + String(SETPOINT_CHANGE_INTERVAL / 1000) + " segundos");
  Serial.println("=============================");
}

void DualMotorControl::stopTest() {
  disable();
  Serial.println("=== FIN DE PRUEBA PID ===");
}

bool DualMotorControl::isTestRunning() const {
  return system_enabled;
}

void DualMotorControl::printSystemStatus() const {
  Serial.println("=== ESTADO DEL SISTEMA ===");
  Serial.println("Tiempo funcionamiento: " + String((millis() - system_start_time) / 1000.0, 2) + " s");
  Serial.println("Sistema habilitado: " + String(system_enabled ? "SI" : "NO"));
  Serial.println("Setpoint actual: " + String(current_setpoint_index + 1) + "/6");
  Serial.println("===========================");
}

void DualMotorControl::printMotorData() const {
  if (left_motor) left_motor->printStatus();
  if (right_motor) right_motor->printStatus();
}