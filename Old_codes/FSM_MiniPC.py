#!/usr/bin/env python3
"""
Máquina de Estados para MiniPC
Comunicación Serial con Arduino Mega
"""

import serial
import time
import sys
from enum import Enum

class EstadoMiniPC(Enum):
    DESCONECTADO = 1
    CONECTANDO = 2
    CONECTADO = 3
    ENVIANDO_COMANDO = 4
    ESPERANDO_RESPUESTA = 5
    PROCESANDO_RESPUESTA = 6
    ERROR = 7

class ComunicacionArduino:
    def __init__(self, puerto='/dev/ttyUSB0', baudrate=115200):
        self.puerto = puerto
        self.baudrate = baudrate
        self.serial_conn = None
        self.estado_actual = EstadoMiniPC.DESCONECTADO
        self.ultimo_comando = ""
        self.respuesta_recibida = ""
        self.timeout = 5.0  # 5 segundos
        self.tiempo_ultimo_comando = 0
        
    def conectar(self):
        """Intenta conectar con el Arduino"""
        try:
            self.serial_conn = serial.Serial(
                port=self.puerto,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            time.sleep(2)  # Esperar estabilización
            return True
        except Exception as e:
            print(f"Error al conectar: {e}")
            return False
    
    def desconectar(self):
        """Cierra la conexión serial"""
        if self.serial_conn and self.serial_conn.is_open:
            self.enviar_mensaje("DISCONNECT")
            time.sleep(0.5)
            self.serial_conn.close()
            print("Conexión cerrada")
    
    def enviar_mensaje(self, mensaje):
        """Envía un mensaje al Arduino"""
        if self.serial_conn and self.serial_conn.is_open:
            mensaje_completo = mensaje + '\n'
            self.serial_conn.write(mensaje_completo.encode())
            self.serial_conn.flush()
            return True
        return False
    
    def leer_respuesta(self):
        """Lee respuesta del Arduino"""
        try:
            if self.serial_conn and self.serial_conn.in_waiting > 0:
                respuesta = self.serial_conn.readline().decode().strip()
                return respuesta
        except Exception as e:
            print(f"Error leyendo respuesta: {e}")
        return None
    
    def maquina_estados(self):
        """Ejecuta la máquina de estados principal"""
        
        if self.estado_actual == EstadoMiniPC.DESCONECTADO:
            print("Estado: DESCONECTADO - Intentando conectar...")
            if self.conectar():
                self.estado_actual = EstadoMiniPC.CONECTANDO
                print("Estado: CONECTANDO")
            else:
                time.sleep(2)
        
        elif self.estado_actual == EstadoMiniPC.CONECTANDO:
            if self.enviar_mensaje("CONNECT"):
                self.estado_actual = EstadoMiniPC.ESPERANDO_RESPUESTA
                self.tiempo_ultimo_comando = time.time()
                print("Enviado: CONNECT")
        
        elif self.estado_actual == EstadoMiniPC.CONECTADO:
            # En este estado esperamos comandos del usuario o ejecutamos rutinas
            self.mostrar_menu()
            
        elif self.estado_actual == EstadoMiniPC.ENVIANDO_COMANDO:
            if self.enviar_mensaje(self.ultimo_comando):
                self.estado_actual = EstadoMiniPC.ESPERANDO_RESPUESTA
                self.tiempo_ultimo_comando = time.time()
                print(f"Enviado: {self.ultimo_comando}")
            else:
                self.estado_actual = EstadoMiniPC.ERROR
        
        elif self.estado_actual == EstadoMiniPC.ESPERANDO_RESPUESTA:
            respuesta = self.leer_respuesta()
            if respuesta:
                self.respuesta_recibida = respuesta
                self.estado_actual = EstadoMiniPC.PROCESANDO_RESPUESTA
                print(f"Recibido: {respuesta}")
            elif time.time() - self.tiempo_ultimo_comando > self.timeout:
                print("TIMEOUT - No se recibió respuesta")
                self.estado_actual = EstadoMiniPC.ERROR
        
        elif self.estado_actual == EstadoMiniPC.PROCESANDO_RESPUESTA:
            if self.procesar_respuesta():
                if self.ultimo_comando == "CONNECT":
                    self.estado_actual = EstadoMiniPC.CONECTADO
                    print("Estado: CONECTADO")
                else:
                    self.estado_actual = EstadoMiniPC.CONECTADO
            else:
                self.estado_actual = EstadoMiniPC.ERROR
        
        elif self.estado_actual == EstadoMiniPC.ERROR:
            print("Estado: ERROR - Intentando reconectar...")
            self.desconectar()
            time.sleep(2)
            self.estado_actual = EstadoMiniPC.DESCONECTADO
    
    def procesar_respuesta(self):
        """Procesa las respuestas del Arduino"""
        if self.ultimo_comando == "CONNECT" and self.respuesta_recibida == "ACK_CONNECT":
            print("✓ Conexión establecida con Arduino")
            return True
        elif self.ultimo_comando == "PING" and self.respuesta_recibida == "PONG":
            print("✓ Ping exitoso")
            return True
        elif self.ultimo_comando == "STATUS" and "ARDUINO_OK" in self.respuesta_recibida:
            print("✓ Arduino funcionando correctamente")
            return True
        elif "LED_" in self.respuesta_recibida:
            print(f"✓ {self.respuesta_recibida}")
            return True
        elif self.respuesta_recibida == "REINICIANDO":
            print("✓ Arduino reiniciando...")
            return True
        elif "Estado:" in self.respuesta_recibida:
            print(f"Arduino - {self.respuesta_recibida}")
            return True
        else:
            print(f"Respuesta no esperada: {self.respuesta_recibida}")
            return False
    
    def mostrar_menu(self):
        """Muestra el menú de opciones"""
        print("\n=== COMUNICACIÓN ARDUINO ===")
        print("1. PING")
        print("2. STATUS") 
        print("3. LED_ON")
        print("4. LED_OFF")
        print("5. RESET")
        print("6. Salir")
        print("Selecciona una opción (1-6): ", end="", flush=True)
        
        # Leer entrada del usuario (no bloqueante en implementación real)
        try:
            opcion = input()
            self.procesar_menu(opcion)
        except KeyboardInterrupt:
            print("\nSaliendo...")
            self.desconectar()
            sys.exit(0)
    
    def procesar_menu(self, opcion):
        """Procesa la opción seleccionada del menú"""
        comandos = {
            '1': 'PING',
            '2': 'STATUS', 
            '3': 'LED_ON',
            '4': 'LED_OFF',
            '5': 'RESET'
        }
        
        if opcion in comandos:
            self.ultimo_comando = comandos[opcion]
            self.estado_actual = EstadoMiniPC.ENVIANDO_COMANDO
        elif opcion == '6':
            print("Desconectando...")
            self.desconectar()
            sys.exit(0)
        else:
            print("Opción no válida")
    
    def ejecutar(self):
        """Bucle principal de la aplicación"""
        print("=== INICIANDO COMUNICACIÓN CON ARDUINO ===")
        print(f"Puerto: {self.puerto}")
        print(f"Baudrate: {self.baudrate}")
        
        try:
            while True:
                self.maquina_estados()
                
                # Leer mensajes adicionales del Arduino
                respuesta_extra = self.leer_respuesta()
                if respuesta_extra and self.estado_actual != EstadoMiniPC.ESPERANDO_RESPUESTA:
                    print(f"Arduino: {respuesta_extra}")
                
                time.sleep(0.1)  # Pequeña pausa
                
        except KeyboardInterrupt:
            print("\nInterrumpido por usuario")
        finally:
            self.desconectar()

def main():
    # Configurar puerto según tu sistema
    # Linux/Mac: '/dev/ttyUSB0' o '/dev/ttyACM0'
    # Windows: 'COM3', 'COM4', etc.
    puerto = 'COM7'  # Cambia según tu configuración
    
    comunicador = ComunicacionArduino(puerto=puerto)
    comunicador.ejecutar()

if __name__ == "__main__":
    main()