import serial
import serial.tools.list_ports
import threading
import time
from datetime import datetime

# Script Gateway para Robot en Python (Versión COM a COM)
# Actúa como un puente entre dos puertos serie: uno para el Arduino y otro
# para el puerto COM virtual creado por Windows para una conexión Bluetooth.
# Requiere la librería: pyserial

# --- Variables Globales ---
arduino_port = None
bluetooth_port = None # Ahora es un puerto serie, no un socket
is_running = True

def log(message):
    """Función para imprimir mensajes con timestamp."""
    timestamp = datetime.now().strftime('%H:%M:%S')
    print(f"[{timestamp}] {message}")

def select_serial_port(prompt_message):
    """Función genérica para seleccionar un puerto COM."""
    log("Detectando puertos COM disponibles...")
    ports = list(serial.tools.list_ports.comports())
    
    if not ports:
        log("ERROR: No se encontraron puertos COM.")
        return None

    log("Puertos encontrados:")
    for i, p in enumerate(ports):
        print(f"  {i}: {p.device} - {p.description}")

    try:
        choice = int(input(f"{prompt_message} (escribe el número y presiona Enter): "))
        if 0 <= choice < len(ports):
            selected_port_name = ports[choice].device
            log(f"Puerto seleccionado: {selected_port_name}")
            return selected_port_name
        else:
            log("ERROR: Selección inválida.")
            return None
    except (ValueError, IndexError):
        log("ERROR: Entrada no válida.")
        return None

def bridge_traffic(source_port, destination_port, source_name, dest_name):
    """Lee desde un puerto y escribe en el otro."""
    global is_running
    while is_running:
        try:
            if source_port and source_port.is_open and source_port.in_waiting > 0:
                message = source_port.readline().decode('utf-8').strip()
                if message:
                    log(f"{source_name} -> PC: {message}")
                    if destination_port and destination_port.is_open:
                        destination_port.write((message + '\n').encode('utf-8'))
                        log(f"PC -> {dest_name}: {message}")
        except serial.SerialException as e:
            log(f"ERROR en {source_name}: {e}. El hilo se cerrará.")
            is_running = False
            break
        time.sleep(0.05)

if __name__ == "__main__":
    log("--- Gateway del Robot en Python Iniciado (Modo Puente COM-a-COM) ---")

    # 1. Seleccionar puerto para el Arduino
    arduino_port_name = select_serial_port("Selecciona el puerto del ARDUINO")
    if not arduino_port_name:
        log("La aplicación se cerrará.")
    else:
        # 2. Seleccionar puerto para el Bluetooth
        bluetooth_port_name = select_serial_port("Selecciona el puerto COM BLUETOOTH 'Saliente'")
        if not bluetooth_port_name:
            log("La aplicación se cerrará.")
        elif arduino_port_name == bluetooth_port_name:
            log("ERROR: El puerto del Arduino y del Bluetooth no pueden ser el mismo. La aplicación se cerrará.")
        else:
            try:
                # 3. Abrir ambos puertos
                arduino_port = serial.Serial(arduino_port_name, 115200, timeout=1)
                log(f"Puerto Arduino ({arduino_port_name}) abierto.")
                
                bluetooth_port = serial.Serial(bluetooth_port_name, 9600, timeout=1) # Los puertos COM de BT suelen usar 9600
                log(f"Puerto Bluetooth ({bluetooth_port_name}) abierto.")

                log("\n--- El Gateway está activo ---")
                log("Conéctate desde la App del celular al dispositivo Bluetooth de la PC.")
                log("El puente de datos está funcionando. Presiona Ctrl+C para detener.")

                # 4. Iniciar hilos para el puente bidireccional
                thread_arduino_to_bt = threading.Thread(
                    target=bridge_traffic, 
                    args=(arduino_port, bluetooth_port, "ARDUINO", "BLUETOOTH")
                )
                
                thread_bt_to_arduino = threading.Thread(
                    target=bridge_traffic, 
                    args=(bluetooth_port, arduino_port, "BLUETOOTH", "ARDUINO")
                )

                thread_arduino_to_bt.daemon = True
                thread_bt_to_arduino.daemon = True
                
                thread_arduino_to_bt.start()
                thread_bt_to_arduino.start()

                # 5. Mantener la aplicación corriendo
                while is_running:
                    time.sleep(1)

            except serial.SerialException as e:
                log(f"ERROR FATAL: No se pudo abrir uno de los puertos. {e}")
            except KeyboardInterrupt:
                log("\nCerrando Gateway...")
            finally:
                is_running = False
                if arduino_port and arduino_port.is_open:
                    arduino_port.close()
                if bluetooth_port and bluetooth_port.is_open:
                    bluetooth_port.close()
                log("Puertos cerrados. Aplicación finalizada.")
