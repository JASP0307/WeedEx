#!/usr/bin/env python3
"""
Simulador completo del Robot Quitamaleza
Simula tanto Arduino como MiniPC para probar comunicación serial
"""

import time
import threading
import queue
import random
import json
import logging
from enum import Enum
from dataclasses import dataclass
from typing import Optional, List
import serial
import serial.tools.list_ports

# Configurar logging con colores para distinguir componentes
class ColoredFormatter(logging.Formatter):
    COLORS = {
        'ARDUINO': '\033[94m',    # Azul
        'MINIPC': '\033[92m',     # Verde
        'SYSTEM': '\033[93m',     # Amarillo
        'ERROR': '\033[91m',      # Rojo
        'ENDC': '\033[0m'         # Reset
    }
    
    def format(self, record):
        # Add default component if it doesn't exist
        if not hasattr(record, 'component'):
            record.component = 'SYSTEM'  # or whatever default you prefer
            
        color = self.COLORS.get(record.component, '')
        record.name = f"{color}{{{record.component}}}{self.COLORS['ENDC']}"
        return super().format(record)

# Configurar logging
handler = logging.StreamHandler()
handler.setFormatter(ColoredFormatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
logger = logging.getLogger('SIMULATOR')
logger.addHandler(handler)
logger.setLevel(logging.INFO)

class ArduinoState(Enum):
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    RAKE_OPERATION = "RAKE_OPERATION"
    LASER_POSITIONING = "LASER_POSITIONING"
    LASER_OPERATION = "LASER_OPERATION"
    LASER_RETURNING = "LASER_RETURNING"
    EMERGENCY_STOP = "EMERGENCY_STOP"

class MiniPCState(Enum):
    BOOT_UP = "boot_up"
    WAITING_CONNECTION = "waiting_connection"
    READY_TO_START = "ready_to_start"
    NAVIGATING = "navigating"
    MECHANICAL_WEEDING = "mechanical_weeding"
    LASER_POSITIONING = "laser_positioning"
    LASER_TARGETING = "laser_targeting"
    LASER_RETURNING = "laser_returning"
    TURNING_SEQUENCE = "turning_sequence"
    MISSION_COMPLETE = "mission_complete"
    EMERGENCY_STOP = "emergency_stop"

@dataclass
class WeedInfo:
    x: int
    y: int
    weed_type: str  # 'mechanical' or 'laser'
    confidence: float
    timestamp: float

class ArduinoSimulator:
    """Simulador del Arduino Mega"""
    
    def __init__(self, serial_port):
        self.serial_port = serial_port
        self.current_state = ArduinoState.IDLE
        self.previous_state = ArduinoState.IDLE
        self.state_timer = time.time()
        
        # Variables de operación
        self.navigation_speed = 0
        self.laser_x = 0
        self.laser_y = 0
        self.obstacle_detected = False
        self.distance = 50  # Distancia simulada del ultrasonido
        
        # Timers para operaciones
        self.rake_duration = 2.0    # 2 segundos
        self.laser_duration = 3.0   # 3 segundos
        
        self.running = True
        
        # Logger específico para Arduino
        self.logger = logging.getLogger('ARDUINO_SIM')
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)
        
        # Simular inicialización
        time.sleep(1)
        self.send_response("STATUS:ARDUINO_READY")
    
    def send_response(self, message):
        """Enviar respuesta por serial"""
        try:
            self.serial_port.write(f"{message}\n".encode())
            self.logger.info(f"→ {message}", extra={'component': 'ARDUINO'})
        except Exception as e:
            self.logger.error(f"Error enviando: {e}", extra={'component': 'ARDUINO'})
    
    def simulate_ultrasonics(self):
        """Simular lecturas de ultrasonido"""
        # Simular obstáculo ocasionalmente
        if random.random() < 0.05:  # 5% probabilidad
            self.distance = random.randint(5, 19)  # Obstáculo detectado
            if not self.obstacle_detected:
                self.obstacle_detected = True
                self.previous_state = self.current_state
                self.current_state = ArduinoState.EMERGENCY_STOP
                self.send_response(f"OBSTACLE:distance_{self.distance}cm")
        else:
            self.distance = random.randint(30, 100)  # Sin obstáculo
            if self.obstacle_detected and self.distance > 25:
                self.obstacle_detected = False
    
    def execute_state_machine(self):
        """Ejecutar máquina de estados del Arduino"""
        current_time = time.time()
        
        if self.current_state == ArduinoState.IDLE:
            # Solo esperar comandos
            pass
            
        elif self.current_state == ArduinoState.NAVIGATING:
            # Mantener navegación
            pass
            
        elif self.current_state == ArduinoState.RAKE_OPERATION:
            # Simular operación de rastrillos
            if current_time - self.state_timer >= self.rake_duration:
                self.current_state = ArduinoState.NAVIGATING
                self.send_response("STATUS:RAKE_COMPLETE")
                
        elif self.current_state == ArduinoState.LASER_POSITIONING:
            # Simular posicionamiento del láser (1 segundo)
            if current_time - self.state_timer >= 1.0:
                self.current_state = ArduinoState.LASER_OPERATION
                self.send_response("STATUS:LASER_POSITIONED")
                self.state_timer = current_time
                
        elif self.current_state == ArduinoState.LASER_OPERATION:
            # Simular operación del láser
            if current_time - self.state_timer >= self.laser_duration:
                self.current_state = ArduinoState.LASER_RETURNING
                self.send_response("STATUS:LASER_OFF")
                self.state_timer = current_time
                
        elif self.current_state == ArduinoState.LASER_RETURNING:
            # Simular retorno del láser (1 segundo)
            if current_time - self.state_timer >= 1.0:
                self.current_state = ArduinoState.NAVIGATING
                self.send_response("STATUS:LASER_HOME")
                
        elif self.current_state == ArduinoState.EMERGENCY_STOP:
            # Solo salir si no hay obstáculo
            if not self.obstacle_detected:
                self.current_state = self.previous_state
                self.send_response("STATUS:EMERGENCY_CLEARED")
    
    def process_command(self, command):
        """Procesar comando recibido"""
        self.logger.info(f"← {command.strip()}", extra={'component': 'ARDUINO'})
        
        if command.startswith("START_NAV:"):
            self.navigation_speed = int(command.split(":")[1])
            self.current_state = ArduinoState.NAVIGATING
            self.send_response("STATUS:NAV_STARTED")
            
        elif command.startswith("DEPLOY_RAKES"):
            if self.current_state == ArduinoState.NAVIGATING:
                self.current_state = ArduinoState.RAKE_OPERATION
                self.state_timer = time.time()
                self.send_response("STATUS:RAKES_DEPLOYED")
                
        elif command.startswith("LASER_POS:"):
            coords = command.split(":")[1].split(",")
            self.laser_x = int(coords[0])
            self.laser_y = int(coords[1])
            if self.current_state == ArduinoState.NAVIGATING:
                self.current_state = ArduinoState.LASER_POSITIONING
                self.state_timer = time.time()
                
        elif command == "EMERGENCY_STOP":
            self.previous_state = self.current_state
            self.current_state = ArduinoState.EMERGENCY_STOP
            
        elif command == "GET_SENSORS":
            self.send_response(f"SENSORS:US_{self.distance}cm,STATE_{self.current_state.value}")
            
        elif command == "STOP":
            self.current_state = ArduinoState.IDLE
            self.send_response("STATUS:STOPPED")
    
    def run(self):
        """Loop principal del Arduino simulado"""
        last_ultrasonic_time = time.time()
        
        while self.running:
            try:
                # Simular ultrasonidos cada 100ms
                if time.time() - last_ultrasonic_time >= 0.1:
                    self.simulate_ultrasonics()
                    last_ultrasonic_time = time.time()
                
                # Ejecutar máquina de estados
                self.execute_state_machine()
                
                # Procesar comandos recibidos
                if self.serial_port.in_waiting > 0:
                    command = self.serial_port.readline().decode().strip()
                    if command:
                        self.process_command(command)
                
                time.sleep(0.01)  # 10ms loop
                
            except Exception as e:
                self.logger.error(f"Error en loop: {e}", extra={'component': 'ARDUINO'})

class MiniPCSimulator:
    """Simulador de la MiniPC"""
    
    def __init__(self, serial_port):
        self.serial_port = serial_port
        self.current_state = MiniPCState.BOOT_UP
        self.previous_state = None
        self.state_start_time = time.time()
        self.mission_start_time = None
        
        # Configuración
        self.navigation_speed = 100
        self.max_surcos = 2
        self.current_surco = 1
        self.mission_stats = {'weeds_mechanical': 0, 'weeds_laser': 0}
        
        # Estado del sistema
        self.arduino_connected = False
        self.emergency_active = False
        self.position_x = 0
        self.surco_end_x = 50  # Longitud del surco en metros
        
        # Cola de comandos internos
        self.command_queue = queue.Queue()
        
        self.running = True
        
        # Logger específico para MiniPC
        self.logger = logging.getLogger('MINIPC_SIM')
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)
        
        # Inicializar
        self.arduino_connected = True
        self.change_state(MiniPCState.READY_TO_START)
    
    def send_arduino_command(self, command):
        """Enviar comando al Arduino"""
        try:
            self.serial_port.write(f"{command}\n".encode())
            self.logger.info(f"→ {command}", extra={'component': 'MINIPC'})
        except Exception as e:
            self.logger.error(f"Error enviando: {e}", extra={'component': 'MINIPC'})
    
    def change_state(self, new_state):
        """Cambiar estado del sistema"""
        if new_state != self.current_state:
            self.logger.info(f"Estado: {self.current_state.value} -> {new_state.value}", 
                           extra={'component': 'MINIPC'})
            self.previous_state = self.current_state
            self.current_state = new_state
            self.state_start_time = time.time()
    
    def simulate_weed_detection(self):
        """Simular detección de maleza"""
        if self.current_state == MiniPCState.NAVIGATING:
            # 15% probabilidad de detectar maleza cada segundo
            if random.random() < 0.15:
                weed_type = 'mechanical' if random.random() < 0.7 else 'laser'
                x = random.randint(50, 150)
                y = random.randint(50, 150)
                
                weed = WeedInfo(
                    x=x, y=y, 
                    weed_type=weed_type, 
                    confidence=random.uniform(0.7, 0.95),
                    timestamp=time.time()
                )
                
                self.command_queue.put({
                    'action': 'weed_detected',
                    'weed_info': weed
                })
                
                self.logger.info(f"Maleza detectada: {weed_type} en ({x}, {y})", 
                               extra={'component': 'MINIPC'})
    
    def simulate_position_update(self):
        """Simular actualización de posición GPS"""
        if self.current_state == MiniPCState.NAVIGATING:
            self.position_x += 0.5  # Avanzar 0.5m por segundo
    
    def execute_state_machine(self):
        """Ejecutar máquina de estados de la MiniPC"""
        current_time = time.time()
        
        if self.current_state == MiniPCState.BOOT_UP:
            if self.arduino_connected:
                self.change_state(MiniPCState.READY_TO_START)
                
        elif self.current_state == MiniPCState.READY_TO_START:
            # Auto-iniciar misión después de 3 segundos
            if current_time - self.state_start_time >= 3.0:
                self.mission_start_time = time.time()
                self.current_surco = 1
                self.position_x = 0
                self.send_arduino_command(f"START_NAV:{self.navigation_speed}")
                self.change_state(MiniPCState.NAVIGATING)
                
        elif self.current_state == MiniPCState.NAVIGATING:
            # Verificar si llegamos al final del surco
            if self.position_x >= self.surco_end_x:
                if self.current_surco < self.max_surcos:
                    self.change_state(MiniPCState.TURNING_SEQUENCE)
                else:
                    self.change_state(MiniPCState.MISSION_COMPLETE)
                return
            
            # Procesar comandos de weeding
            if not self.command_queue.empty():
                command = self.command_queue.get()
                if command.get('action') == 'weed_detected':
                    weed_info = command.get('weed_info')
                    if weed_info.weed_type == 'mechanical':
                        self.deploy_mechanical_weeding()
                    elif weed_info.weed_type == 'laser':
                        self.start_laser_weeding(weed_info.x, weed_info.y)
                        
        elif self.current_state == MiniPCState.TURNING_SEQUENCE:
            # Simular giro (2 segundos)
            if current_time - self.state_start_time >= 2.0:
                self.current_surco += 1
                self.position_x = 0  # Reset posición
                self.logger.info(f"Iniciando surco {self.current_surco}", 
                               extra={'component': 'MINIPC'})
                self.send_arduino_command(f"START_NAV:{self.navigation_speed}")
                self.change_state(MiniPCState.NAVIGATING)
                
        elif self.current_state == MiniPCState.MISSION_COMPLETE:
            self.send_arduino_command("STOP")
            self.send_mission_report()
            # Reiniciar después de 5 segundos
            if current_time - self.state_start_time >= 5.0:
                self.change_state(MiniPCState.READY_TO_START)
    
    def deploy_mechanical_weeding(self):
        """Desplegar desmalezado mecánico"""
        self.send_arduino_command("DEPLOY_RAKES")
        self.mission_stats['weeds_mechanical'] += 1
        self.change_state(MiniPCState.MECHANICAL_WEEDING)
    
    def start_laser_weeding(self, x, y):
        """Iniciar desmalezado con láser"""
        self.send_arduino_command(f"LASER_POS:{x},{y}")
        self.mission_stats['weeds_laser'] += 1
        self.change_state(MiniPCState.LASER_POSITIONING)
    
    def send_mission_report(self):
        """Enviar reporte de misión"""
        if self.mission_start_time:
            duration = time.time() - self.mission_start_time
            report = {
                'stats': {
                    **self.mission_stats,
                    'duration_minutes': round(duration / 60, 2),
                    'surcos_completed': self.current_surco
                }
            }
            self.logger.info(f"Misión completada: {report['stats']}", 
                           extra={'component': 'MINIPC'})
    
    def process_arduino_response(self, response):
        """Procesar respuesta del Arduino"""
        self.logger.info(f"← {response.strip()}", extra={'component': 'MINIPC'})
        
        if response.startswith("STATUS:"):
            status = response.split(":")[1]
            
            if status == "ARDUINO_READY":
                self.arduino_connected = True
            elif status == "RAKE_COMPLETE":
                self.change_state(MiniPCState.NAVIGATING)
            elif status == "LASER_POSITIONED":
                self.change_state(MiniPCState.LASER_TARGETING)
            elif status == "LASER_OFF":
                self.change_state(MiniPCState.LASER_RETURNING)
            elif status == "LASER_HOME":
                self.change_state(MiniPCState.NAVIGATING)
            elif status == "EMERGENCY_CLEARED":
                self.emergency_active = False
                
        elif response.startswith("OBSTACLE:"):
            self.emergency_active = True
            if self.current_state != MiniPCState.EMERGENCY_STOP:
                self.change_state(MiniPCState.EMERGENCY_STOP)
    
    def run(self):
        """Loop principal de la MiniPC simulada"""
        last_detection_time = time.time()
        last_position_time = time.time()
        
        while self.running:
            try:
                current_time = time.time()
                
                # Simular detección de maleza cada segundo
                if current_time - last_detection_time >= 1.0:
                    self.simulate_weed_detection()
                    last_detection_time = current_time
                
                # Actualizar posición cada segundo
                if current_time - last_position_time >= 1.0:
                    self.simulate_position_update()
                    last_position_time = current_time
                
                # Ejecutar máquina de estados
                self.execute_state_machine()
                
                # Procesar respuestas del Arduino
                if self.serial_port.in_waiting > 0:
                    response = self.serial_port.readline().decode().strip()
                    if response:
                        self.process_arduino_response(response)
                
                time.sleep(0.1)  # 100ms loop
                
            except Exception as e:
                self.logger.error(f"Error en loop: {e}", extra={'component': 'MINIPC'})

class VirtualSerialPort:
    """Puerto serie virtual para comunicación bidireccional"""
    
    def __init__(self):
        self.buffer_a_to_b = queue.Queue()
        self.buffer_b_to_a = queue.Queue()
        
    def get_port_a(self):
        return VirtualPort(self.buffer_a_to_b, self.buffer_b_to_a)
    
    def get_port_b(self):
        return VirtualPort(self.buffer_b_to_a, self.buffer_a_to_b)

class VirtualPort:
    """Puerto virtual individual"""
    
    def __init__(self, send_buffer, receive_buffer):
        self.send_buffer = send_buffer
        self.receive_buffer = receive_buffer
        
    def write(self, data):
        self.send_buffer.put(data)
        
    def readline(self):
        try:
            return self.receive_buffer.get_nowait()
        except queue.Empty:
            return b""
    
    @property
    def in_waiting(self):
        return self.receive_buffer.qsize()

def main():
    """Función principal del simulador"""
    logger.info("=== INICIANDO SIMULADOR ROBOT QUITAMALEZA ===", extra={'component': 'SYSTEM'})
    
    # Crear puerto serie virtual
    virtual_serial = VirtualSerialPort()
    arduino_port = virtual_serial.get_port_a()
    minipc_port = virtual_serial.get_port_b()
    
    # Crear simuladores
    arduino_sim = ArduinoSimulator(arduino_port)
    minipc_sim = MiniPCSimulator(minipc_port)
    
    # Crear threads
    arduino_thread = threading.Thread(target=arduino_sim.run, daemon=True)
    minipc_thread = threading.Thread(target=minipc_sim.run, daemon=True)
    
    # Iniciar threads
    arduino_thread.start()
    minipc_thread.start()
    
    logger.info("Simuladores iniciados. Presiona Ctrl+C para detener.", extra={'component': 'SYSTEM'})
    
    try:
        # Mantener programa corriendo
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        logger.info("Deteniendo simuladores...", extra={'component': 'SYSTEM'})
        arduino_sim.running = False
        minipc_sim.running = False
        
    logger.info("=== SIMULADOR TERMINADO ===", extra={'component': 'SYSTEM'})

if __name__ == "__main__":
    main()