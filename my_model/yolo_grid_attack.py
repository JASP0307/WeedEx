# yolo_grid_attack_precalibrado.py
import os
import sys
import argparse
import time
import json
import serial 

import cv2
import numpy as np
from ultralytics import YOLO

# --- CONFIGURACIÓN DE LA GRILLA Y ARDUINO ---
NOMBRE_ARCHIVO_CONFIG = "config_grilla.json"
GRILLA_FILAS = 2
GRILLA_COLUMNAS = 10
GRILLA_POSICION_Y = 300  

# Ajusta el puerto a tu Arduino (ej. 'COM3' en Windows, '/dev/ttyACM0' en Linux)
ARDUINO_PORT = 'COM6' 
ARDUINO_BAUD = 9600
arduino = None 

# --- SECCIÓN DE FUNCIONES ---

def inicializar_serial():
    """Intenta conectar con el Arduino por el puerto serial."""
    global arduino
    try:
        arduino = serial.Serial(port=ARDUINO_PORT, baudrate=ARDUINO_BAUD, timeout=1)
        time.sleep(2) 
        print(f"✅ Conectado al Arduino en el puerto {ARDUINO_PORT}")
    except serial.SerialException:
        print(f"⚠️  ADVERTENCIA: No se pudo conectar al Arduino en {ARDUINO_PORT}.")
        arduino = None

def enviar_comando_arduino(comando):
    """Envía un string al Arduino si está conectado."""
    if arduino and arduino.is_open:
        arduino.write(comando.encode('utf-8'))
        print(f"🚀 Comando enviado: {comando.strip()}")

def cargar_configuracion():
    """Carga los datos de calibración de la grilla. Termina si no es válido."""
    if not os.path.exists(NOMBRE_ARCHIVO_CONFIG):
        print(f"❌ ERROR: El archivo '{NOMBRE_ARCHIVO_CONFIG}' no se encontró.")
        print("Asegúrate de que el archivo de configuración exista o ejecuta el script de calibración primero.")
        sys.exit()
    
    with open(NOMBRE_ARCHIVO_CONFIG, 'r') as f:
        datos = json.load(f)
    
    pixels_por_cm = datos.get("pixels_por_cm")
    if not pixels_por_cm:
        print(f"❌ ERROR: El archivo '{NOMBRE_ARCHIVO_CONFIG}' no contiene la clave 'pixels_por_cm'.")
        sys.exit()
        
    print("✅ Configuración de grilla cargada exitosamente.")
    return pixels_por_cm

def dibujar_grilla(frame, origen, pixels_por_cm):
    """Dibuja la grilla en el frame."""
    ox, oy = origen
    ancho_celda_px = int(1.0 * pixels_por_cm)
    alto_celda_px = ancho_celda_px
    
    ancho_total_grilla = GRILLA_COLUMNAS * ancho_celda_px
    alto_total_grilla = GRILLA_FILAS * alto_celda_px

    # Dibuja la grilla y numera las celdas
    for i in range(GRILLA_FILAS):
        for j in range(GRILLA_COLUMNAS):
            x1 = ox + j * ancho_celda_px
            y1 = oy + i * alto_celda_px
            cv2.rectangle(frame, (x1, y1), (x1 + ancho_celda_px, y1 + alto_celda_px), (255, 0, 0), 1)
            numero_posicion = i * GRILLA_COLUMNAS + j + 1
            cv2.putText(frame, str(numero_posicion), (x1 + 5, y1 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            
    return ancho_total_grilla, alto_total_grilla, ancho_celda_px

# --- PROGRAMA PRINCIPAL ---

if __name__ == "__main__":
    # --- PARSEO DE ARGUMENTOS DE YOLO ---
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True, help='Path to YOLO model file')
    parser.add_argument('--source', required=True, help='Image source (file, folder, video, "usb0", etc.)')
    parser.add_argument('--thresh', type=float, default=0.5, help='Confidence threshold')
    args = parser.parse_args()

    # --- INICIALIZACIÓN ---
    inicializar_serial()
    pixels_por_cm = cargar_configuracion()
    model = YOLO(args.model)
    
    # Configuración de la fuente de video
    if 'usb' in args.source:
        cap = cv2.VideoCapture(int(args.source[3:]))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    else:
        cap = cv2.VideoCapture(args.source)

    window_name = 'YOLO Grid Attack System'
    cv2.namedWindow(window_name)
    
    print("\n--- Sistema iniciado ---")
    print("Presiona 'q' para salir.")
    print("---------------------\n")

    posiciones_atacadas_en_frame = set()

    # --- BUCLE PRINCIPAL DE DETECCIÓN ---
    while True:
        ret, frame = cap.read()
        if not ret: break

        posiciones_atacadas_en_frame.clear()

        # --- DIBUJAR GRILLA ---
        ancho_total_grilla_px = int(GRILLA_COLUMNAS * pixels_por_cm)
        ancho_ventana = frame.shape[1]
        origen_x = (ancho_ventana - ancho_total_grilla_px) // 2
        origen_y = GRILLA_POSICION_Y
        origen_grilla = (origen_x, origen_y)
        
        ancho_total_grilla, alto_total_grilla, ancho_celda_px = dibujar_grilla(frame, origen_grilla, pixels_por_cm)

        # --- LÓGICA DE DETECCIÓN YOLO ---
        results = model(frame, verbose=False)
        for detection in results[0].boxes:
            if detection.conf[0] > args.thresh:
                xyxy = detection.xyxy[0].cpu().numpy().astype(int)
                xmin, ymin, xmax, ymax = xyxy
                
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                
                # --- NÚCLEO DE LA INTEGRACIÓN ---
                centerX = int((xmin + xmax) / 2)
                centerY = int((ymin + ymax) / 2)
                cv2.circle(frame, (centerX, centerY), 5, (0, 0, 255), -1)

                ox, oy = origen_grilla
                if (ox <= centerX < ox + ancho_total_grilla) and (oy <= centerY < oy + alto_total_grilla):
                    col = (centerX - ox) // ancho_celda_px
                    row = (centerY - oy) // alto_celda_px
                    posicion = row * GRILLA_COLUMNAS + col + 1

                    if posicion not in posiciones_atacadas_en_frame:
                        comando = f"ATTACK,{posicion}\n"
                        enviar_comando_arduino(comando)
                        posiciones_atacadas_en_frame.add(posicion)

                        x1 = ox + col * ancho_celda_px
                        y1 = oy + row * ancho_celda_px
                        cv2.rectangle(frame, (x1, y1), (x1 + ancho_celda_px, y1 + alto_celda_px), (0,0,255), -1)

        cv2.imshow(window_name, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # --- LIMPIEZA FINAL ---
    cap.release()
    cv2.destroyAllWindows()
    if arduino and arduino.is_open:
        arduino.close()
        print("Puerto serial cerrado.")