#ifndef ROBOT_DEFINITIONS_H
#define ROBOT_DEFINITIONS_H

// Este archivo centraliza todas las definiciones lógicas, constantes de comportamiento,
// y enumeraciones para la Máquina de Estados Finitos (FSM) del robot.

// =================================================================
// 1. ESTADOS DE LA MÁQUINA DE ESTADOS (FSM)
// =================================================================
enum RobotState {
  IDLE,
  NAVIGATING,
  MOVING_TO_WEED,
  LASERING,
  RETURNING_HOME,
  ERROR_STATE,
  LOW_BATTERY,
  OBSTACLE,
  ROW_CHANGE
};

// =================================================================
// 2. EVENTOS DE LA MÁQUINA DE ESTADOS (FSM)
// =================================================================
typedef enum {
  EVENT_NONE,
  EVENT_NAVIGATE,
  EVENT_STOP,
  EVENT_OBSTACLE,
  EVENT_RAKE_WEED_FOUND,
  EVENT_WEED_FOUND,   
  EVENT_ARM_AT_TARGET,
  EVENT_LASER_COMPLETE,
  EVENT_ARM_AT_HOME,
  EVENT_LOW_BATTERY,
  EVENT_ERROR,
  EVENT_RESUME,
  EVENT_ATTACK_COMPLETE,
  EVENT_TIMEOUT_OBSTACLE,
  EVENT_IR_SIGNAL_DETECTED,
  EVENT_ROW_CHANGED
} FSMEvent;

enum ArmCommand {
  CMD_IDLE,
  CMD_MOVE_TO_TARGET,
  CMD_RETURN_HOME
};

// =================================================================
// 3. CONSTANTES DE COMPORTAMIENTO
// =================================================================

// Estado protegido por mutex
volatile RobotState currentState = IDLE;
float g_batteryVoltage = 0.0;

// --- Detección de Obstáculos ---
const int DISTANCIA_MINIMA = 20; // Distancia en cm para detectar un obstáculo
const int SIMULATED_MOVE_TIME_MS = 3000;

// --- Tiempos de Espera (en milisegundos) ---
const int OBSTACLE_WAIT_TIME_MS = 5000;
const int LASER_ON_TIME_MS = 2500;
const int RAKE_ACTION_TIME_MS = 5000;

// --- Parámetros de Servos ---

const int POS_INICIAL_RASTRILLO = 170; // Posición de reposo
const int POS_TRABAJO_RASTRILLO = 150; // Posición para rastrillar

// --- Parámetros de Divisor de Voltaje ---

const float R1 = 21800.0; // 20k ohms
const float R2 = 10000.0; // 10k ohms
const float VOLTAJE_BATERIA_BAJA = 9;

// Puedes añadir más constantes aquí (velocidades de motor, etc.)
volatile ArmCommand g_armCommand = CMD_IDLE;

bool g_isRaking = false;
TickType_t rakeStartTime;

float g_targetYaw = 0.0;      // Rumbo deseado (ej. 0.0 para ir al Norte)
float g_baseSpeedRPM = 0.1;  // Velocidad base de los motores en RPM
SemaphoreHandle_t locomotionMutex;


// --- CONSTANTES DE AJUSTE DEL PID (TUNING) ---
const float Kp_heading = 0.5;
const float Ki_heading = 0.0;
const float Kd_heading = 0.0;

float VELOCIDAD_CRUCERO = 10.0;

// --- LÓGICA DEL BRAZO DELTA ---
// Parámetros físicos y de compensación

const int SERVO1_HORIZONTAL = 172; 
const int SERVO2_HORIZONTAL = 168;
const int SERVO3_HORIZONTAL = 178;
const double ROTATION_ANGLE_DEG = -30.0;
double cos_theta, sin_theta;

// Posiciones
const double HOME_X = 0, HOME_Y = 0, HOME_Z = -110;
const double Z_ATAQUE = -100;

// Estructura y array para el grid de ataque
struct GridPoint { double x; double y; double z;};
const int NUM_GRID_POINTS = 20;
GridPoint grid[NUM_GRID_POINTS];


// --- DECLARACIONES PARA LA TAREA PID ---
float integralError = 0.0;
float previousError = 0.0;

#endif // ROBOT_DEFINITIONS_H