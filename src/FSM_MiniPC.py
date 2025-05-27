#!/usr/bin/env python3
"""
MiniPC Real del Robot Quitamaleza
Se conecta a Arduino Mega real via puerto serie
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

# Configurar logging con colores
class ColoredFormatter(logging.Formatter):
    COLORS = {
        'MINIPC': '\033[92m',     # Verde
        'ARDUINO': '\033[94m',    # Azul
        'SYSTEM': '\033[93m',     # Amarillo
        'ERROR': '\033[91m',      # Rojo
        'ENDC': '\033[0m'         # Reset
    }
    
    def format(self, record):
        if not hasattr(record, 'component'):
            record.component = 'SYSTEM'
            
        color = self.COLORS.get(record.component, '')
        record.name = f"{color}{{{record.component}}}{self.COLORS['ENDC']}"
        return super().format(record)

# Configurar logging
handler = logging.StreamHandler()
handler.setFormatter(ColoredFormatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
logger = logging.getLogger('ROBOT_SYSTEM')
logger.addHandler(handler)
logger.setLevel(logging.INFO)

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

class ArduinoConnection:
    """Maneja la comunicación con el Arduino Mega real"""
    
    def __init__(self, port=None, baudrate=115200, timeout=1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.connected = False
        self.response_queue = queue.Queue()
        self.running = False
        
        # Logger específico para comunicación Arduino
        self.logger = logging.getLogger('ARDUINO_COMM')
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)
    
    def find_arduino_port(self):
        """Buscar automáticamente el puerto del Arduino"""
        self.logger.info("Buscando Arduino Mega...", extra={'component': 'SYSTEM'})
        
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.logger.info(f"Puerto encontrado: {port.device} - {port.description}", 
                           extra={'component': 'SYSTEM'})
            
            # Buscar puertos que podrían ser Arduino
            if any(keyword in port.description.lower() for keyword in 
                   ['arduino', 'mega', 'ch340', 'usb-serial', 'cp210x']):
                return port.device
        
        # Si no encuentra uno específico, probar puertos comunes
        common_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1',
                       'COM3', 'COM4', 'COM5', 'COM6']
        
        for port in common_ports:
            try:
                test_conn = serial.Serial(port, self.baudrate, timeout=0.5)
                test_conn.close()
                self.logger.info(f"Puerto disponible: {port}", extra={'component': 'SYSTEM'})
                return port
            except:
                continue
                
        return None
    
    def connect(self):
        """Conectar al Arduino"""
        if not self.port:
            self.port = self.find_arduino_port()
            
        if not self.port:
            self.logger.error("No se encontró puerto Arduino", extra={'component': 'SYSTEM'})
            return False
        
        try:
            self.logger.info(f"Conectando a Arduino en {self.port}...", 
                           extra={'component': 'SYSTEM'})
            
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
            
            # Esperar a que Arduino se inicialice
            time.sleep(2)
            
            # Limpiar buffer inicial
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            self.connected = True
            self.running = True
            
            # Iniciar hilo de lectura
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            
            self.logger.info("Conectado exitosamente al Arduino", 
                           extra={'component': 'SYSTEM'})
            return True
            
        except Exception as e:
            self.logger.error(f"Error conectando: {e}", extra={'component': 'SYSTEM'})
            return False
    
    def disconnect(self):
        """Desconectar del Arduino"""
        self.running = False
        self.connected = False
        
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.logger.info("Desconectado del Arduino", extra={'component': 'SYSTEM'})
    
    def send_command(self, command):
        """Enviar comando al Arduino"""
        if not self.connected or not self.serial_conn:
            self.logger.error("Arduino no conectado", extra={'component': 'SYSTEM'})
            return False
        
        try:
            message = f"{command}\n"
            self.serial_conn.write(message.encode())
            self.logger.info(f"→ {command}", extra={'component': 'ARDUINO'})
            return True
            
        except Exception as e:
            self.logger.error(f"Error enviando comando: {e}", extra={'component': 'ARDUINO'})
            return False
    
    def get_response(self, timeout=1.0):
        """Obtener respuesta del Arduino"""
        try:
            return self.response_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def _read_loop(self):
        """Hilo de lectura continua del Arduino"""
        while self.running and self.connected:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode().strip()
                    if line:
                        self.logger.info(f"← {line}", extra={'component': 'ARDUINO'})
                        self.response_queue.put(line)
                
                time.sleep(0.01)  # 10ms delay
                
            except Exception as e:
                if self.running:  # Solo log si no estamos cerrando
                    self.logger.error(f"Error leyendo: {e}", extra={'component': 'ARDUINO'})
                break
        
        self.connected = False

class WeedDetector:
    """Simulador de detección de maleza (reemplazar con IA real)"""
    
    def __init__(self):
        self.logger = logging.getLogger('WEED_DETECTOR')
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)
        
        # En implementación real, aquí inicializarías cámara y modelo de IA
        self.detection_active = False
    
    def start_detection(self):
        """Iniciar detección de maleza"""
        self.detection_active = True
        self.logger.info("Detección de maleza activada", extra={'component': 'MINIPC'})
    
    def stop_detection(self):
        """Detener detección de maleza"""
        self.detection_active = False
        self.logger.info("Detección de maleza desactivada", extra={'component': 'MINIPC'})
    
    def detect_weeds(self):
        """
        Detectar maleza en tiempo real
        En implementación real, procesar frames de cámara con IA
        """
        if not self.detection_active:
            return []
        
        weeds = []
        
        # SIMULACIÓN - Reemplazar con procesamiento real
        if random.random() < 0.15:  # 15% probabilidad por segundo
            weed_type = 'mechanical' if random.random() < 0.7 else 'laser'
            x = random.randint(50, 150)
            y = random.randint(50, 150)
            
            weed = WeedInfo(
                x=x, y=y,
                weed_type=weed_type,
                confidence=random.uniform(0.7, 0.95),
                timestamp=time.time()
            )
            
            weeds.append(weed)
            self.logger.info(f"Maleza detectada: {weed_type} en ({x}, {y}) - Conf: {weed.confidence:.2f}", 
                           extra={'component': 'MINIPC'})
        
        return weeds

class GPSSimulator:
    """Simulador de GPS (reemplazar con GPS real)"""
    
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.speed = 0.5  # m/s
        self.logger = logging.getLogger('GPS')
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)
    
    def update_position(self, dt):
        """Actualizar posición GPS"""
        self.x += self.speed * dt
        # En implementación real, leer GPS actual
    
    def get_position(self):
        """Obtener posición actual"""
        return self.x, self.y

class MiniPCController:
    """Controlador principal de la MiniPC"""
    
    def __init__(self, arduino_port=None):
        self.arduino = ArduinoConnection(arduino_port)
        self.weed_detector = WeedDetector()
        self.gps = GPSSimulator()
        
        # Estado del sistema
        self.current_state = MiniPCState.BOOT_UP
        self.previous_state = None
        self.state_start_time = time.time()
        self.mission_start_time = None
        
        # Configuración de misión
        self.navigation_speed = 100
        self.max_surcos = 2
        self.current_surco = 1
        self.surco_length = 50  # metros
        self.mission_stats = {'weeds_mechanical': 0, 'weeds_laser': 0}
        
        # Control de sistema
        self.emergency_active = False
        self.running = True
        
        # Logger principal
        self.logger = logging.getLogger('MINIPC_CTRL')
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)
    
    def initialize(self):
        """Inicializar sistema"""
        self.logger.info("=== INICIANDO MINIPC ROBOT QUITAMALEZA ===", 
                        extra={'component': 'SYSTEM'})
        
        # Conectar al Arduino
        if not self.arduino.connect():
            self.logger.error("Fallo conectando al Arduino", extra={'component': 'SYSTEM'})
            return False
        
        # Esperar confirmación del Arduino
        timeout = time.time() + 10  # 10 segundos timeout
        arduino_ready = False
        
        while time.time() < timeout and not arduino_ready:
            response = self.arduino.get_response(timeout=1.0)
            if response and "STATUS:ARDUINO_READY" in response:
                arduino_ready = True
                break
            time.sleep(0.1)
        
        if not arduino_ready:
            self.logger.error("Arduino no responde", extra={'component': 'SYSTEM'})
            return False
        
        self.change_state(MiniPCState.READY_TO_START)
        self.logger.info("Sistema inicializado correctamente", extra={'component': 'SYSTEM'})
        return True
    
    def shutdown(self):
        """Cerrar sistema limpiamente"""
        self.logger.info("Cerrando sistema...", extra={'component': 'SYSTEM'})
        self.running = False
        self.arduino.send_command("STOP")
        time.sleep(0.5)
        self.arduino.disconnect()
        self.weed_detector.stop_detection()
    
    def change_state(self, new_state):
        """Cambiar estado del sistema"""
        if new_state != self.current_state:
            self.logger.info(f"Estado: {self.current_state.value} -> {new_state.value}", 
                           extra={'component': 'MINIPC'})
            self.previous_state = self.current_state
            self.current_state = new_state
            self.state_start_time = time.time()
    
    def start_mission(self):
        """Iniciar misión de desmalezado"""
        self.logger.info("=== INICIANDO MISIÓN DE DESMALEZADO ===", 
                        extra={'component': 'MINIPC'})
        
        self.mission_start_time = time.time()
        self.current_surco = 1
        self.gps.x = 0
        self.mission_stats = {'weeds_mechanical': 0, 'weeds_laser': 0}
        
        # Iniciar navegación
        self.arduino.send_command(f"START_NAV:{self.navigation_speed}")
        self.weed_detector.start_detection()
        self.change_state(MiniPCState.NAVIGATING)
    
    def process_weeds(self, weeds):
        """Procesar maleza detectada"""
        for weed in weeds:
            if weed.weed_type == 'mechanical':
                self.deploy_mechanical_weeding()
            elif weed.weed_type == 'laser':
                self.start_laser_weeding(weed.x, weed.y)
            break  # Procesar una maleza a la vez
    
    def deploy_mechanical_weeding(self):
        """Desplegar desmalezado mecánico"""
        if self.current_state == MiniPCState.NAVIGATING:
            self.arduino.send_command("DEPLOY_RAKES")
            self.mission_stats['weeds_mechanical'] += 1
            self.change_state(MiniPCState.MECHANICAL_WEEDING)
    
    def start_laser_weeding(self, x, y):
        """Iniciar desmalezado con láser"""
        if self.current_state == MiniPCState.NAVIGATING:
            self.arduino.send_command(f"LASER_POS:{x},{y}")
            self.mission_stats['weeds_laser'] += 1
            self.change_state(MiniPCState.LASER_POSITIONING)
    
    def process_arduino_responses(self):
        """Procesar respuestas del Arduino"""
        response = self.arduino.get_response(timeout=0.1)
        if not response:
            return
        
        if response.startswith("STATUS:"):
            status = response.split(":")[1]
            
            if status == "RAKE_COMPLETE":
                self.change_state(MiniPCState.NAVIGATING)
            elif status == "LASER_POSITIONED":
                self.change_state(MiniPCState.LASER_TARGETING)
            elif status == "LASER_OFF":
                self.change_state(MiniPCState.LASER_RETURNING)
            elif status == "LASER_HOME":
                self.change_state(MiniPCState.NAVIGATING)
            elif status == "EMERGENCY_CLEARED":
                self.emergency_active = False
                self.change_state(self.previous_state or MiniPCState.NAVIGATING)
                
        elif response.startswith("OBSTACLE:"):
            self.emergency_active = True
            if self.current_state != MiniPCState.EMERGENCY_STOP:
                self.change_state(MiniPCState.EMERGENCY_STOP)
                
        elif response.startswith("SENSORS:"):
            # Procesar datos de sensores si es necesario
            pass
    
    def execute_state_machine(self):
        """Ejecutar máquina de estados principal"""
        current_time = time.time()
        
        if self.current_state == MiniPCState.BOOT_UP:
            # Estado inicial, esperar conexión
            pass
            
        elif self.current_state == MiniPCState.READY_TO_START:
            # Auto-iniciar misión después de 3 segundos
            if current_time - self.state_start_time >= 3.0:
                self.start_mission()
                
        elif self.current_state == MiniPCState.NAVIGATING:
            # Actualizar posición
            dt = current_time - getattr(self, '_last_position_update', current_time)
            self.gps.update_position(dt)
            self._last_position_update = current_time
            
            # Verificar fin de surco
            if self.gps.x >= self.surco_length:
                if self.current_surco < self.max_surcos:
                    self.change_state(MiniPCState.TURNING_SEQUENCE)
                else:
                    self.change_state(MiniPCState.MISSION_COMPLETE)
                return
            
            # Procesar detección de maleza
            weeds = self.weed_detector.detect_weeds()
            if weeds:
                self.process_weeds(weeds)
                
        elif self.current_state == MiniPCState.TURNING_SEQUENCE:
            # Simular giro entre surcos (2 segundos)
            if current_time - self.state_start_time >= 2.0:
                self.current_surco += 1
                self.gps.x = 0  # Reset posición
                self.logger.info(f"Iniciando surco {self.current_surco}/{self.max_surcos}", 
                               extra={'component': 'MINIPC'})
                self.arduino.send_command(f"START_NAV:{self.navigation_speed}")
                self.change_state(MiniPCState.NAVIGATING)
                
        elif self.current_state == MiniPCState.MISSION_COMPLETE:
            self.arduino.send_command("STOP")
            self.weed_detector.stop_detection()
            self.send_mission_report()
            
            # Reiniciar después de 10 segundos
            if current_time - self.state_start_time >= 10.0:
                self.change_state(MiniPCState.READY_TO_START)
                
        elif self.current_state == MiniPCState.EMERGENCY_STOP:
            # Esperar que se despeje la emergencia
            pass
    
    def send_mission_report(self):
        """Enviar reporte de misión"""
        if self.mission_start_time:
            duration = time.time() - self.mission_start_time
            report = {
                'surcos_completed': self.current_surco,
                'weeds_mechanical': self.mission_stats['weeds_mechanical'],
                'weeds_laser': self.mission_stats['weeds_laser'],
                'duration_minutes': round(duration / 60, 2),
                'distance_meters': round(self.gps.x, 1)
            }
            
            self.logger.info(f"=== MISIÓN COMPLETADA ===", extra={'component': 'MINIPC'})
            self.logger.info(f"Surcos: {report['surcos_completed']}", extra={'component': 'MINIPC'})
            self.logger.info(f"Maleza mecánica: {report['weeds_mechanical']}", extra={'component': 'MINIPC'})
            self.logger.info(f"Maleza láser: {report['weeds_laser']}", extra={'component': 'MINIPC'})
            self.logger.info(f"Duración: {report['duration_minutes']} min", extra={'component': 'MINIPC'})
            self.logger.info(f"Distancia: {report['distance_meters']} m", extra={'component': 'MINIPC'})
    
    def run(self):
        """Loop principal del sistema"""
        if not self.initialize():
            return
        
        self.logger.info("Sistema funcionando. Presiona Ctrl+C para detener.", 
                        extra={'component': 'SYSTEM'})
        
        try:
            while self.running:
                # Ejecutar máquina de estados
                self.execute_state_machine()
                
                # Procesar respuestas del Arduino
                self.process_arduino_responses()
                
                time.sleep(0.1)  # 100ms loop
                
        except KeyboardInterrupt:
            self.logger.info("Interrupción del usuario", extra={'component': 'SYSTEM'})
        except Exception as e:
            self.logger.error(f"Error en loop principal: {e}", extra={'component': 'SYSTEM'})
        finally:
            self.shutdown()

def main():
    """Función principal"""
    import argparse
    
    parser = argparse.ArgumentParser(description='MiniPC Robot Quitamaleza')
    parser.add_argument('--port', '-p', help='Puerto serie del Arduino (ej: /dev/ttyUSB0 o COM3)')
    parser.add_argument('--baudrate', '-b', type=int, default=115200, help='Baudrate (default: 115200)')
    
    args = parser.parse_args()
    
    # Crear y ejecutar controlador
    controller = MiniPCController(arduino_port=args.port)
    controller.run()

if __name__ == "__main__":
    main()
