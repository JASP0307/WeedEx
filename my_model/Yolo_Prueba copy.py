import os
import sys
import argparse
import glob
import time
import serial # <--- NUEVA IMPORTACIÓN
import json
import threading # <--- NUEVA IMPORTACIÓN
from flask import Response, Flask # <--- NUEVAS IMPORTACIONES

import cv2
import numpy as np
from ultralytics import YOLO

# --- NUEVO: Configuración del servidor de streaming ---
output_frame = None
lock = threading.Lock()
# Inicializar la aplicación Flask
app = Flask(__name__)

# Define and parse user input arguments
# (El código de argparse se mantiene igual)
parser = argparse.ArgumentParser()
parser.add_argument('--model', help='Path to YOLO model file (example: "runs/detect/train/weights/best.pt")',
                    required=True)
parser.add_argument('--source', help='Image source, can be image file ("test.jpg"), \
                    image folder ("test_dir"), video file ("testvid.mp4"), index of USB camera ("usb0"), or index of Picamera ("picamera0")', 
                    required=True)
parser.add_argument('--thresh', help='Minimum confidence threshold for displaying detected objects (example: "0.4")',
                    default=0.5)
parser.add_argument('--resolution', help='Resolution in WxH to display inference results at (example: "640x480"), \
                    otherwise, match source resolution',
                    default=None)
parser.add_argument('--record', help='Record results from video or webcam and save it as "demo1.avi". Must specify --resolution argument to record.',
                    action='store_true')

args = parser.parse_args()


# Parse user inputs
model_path = args.model
img_source = args.source
min_thresh = float(args.thresh) # Asegurarse de que sea float
user_res = args.resolution
record = args.record

# ######################################################
# ## CONFIGURACIÓN DEL ROBOT Y GRID ##
# ######################################################
# Ajusta estos parámetros según tu configuración
SERIAL_PORT = 'COM7'  # <--- CAMBIA ESTO (ej. 'COM3' en Windows, '/dev/ttyACM0' en Linux)
BAUD_RATE = 9600
TARGET_CLASS = 'Rude' # Nombre de la clase de maleza en tu modelo YOLO
SERIAL_TIMEOUT = 10

# Configuración del Grid
# --- NUEVO: Configuración de dimensiones físicas y archivo ---
CONFIG_FILE = 'config_grilla.json'
GRID_ROWS_CM = 2  # Altura del grid en cm
GRID_COLS_CM = 10 # Ancho del grid en cm

LOG_FILE = 'log_ataques.json'


def resetear_log():
    """Borra el archivo de log y limpia el diccionario en memoria."""
    global posiciones_atacadas_log
    try:
        if os.path.exists(LOG_FILE):
            os.remove(LOG_FILE)
            print("✅ Archivo de log borrado exitosamente.")
        else:
            print("INFO: El archivo de log no existía, no hay nada que borrar.")
        
        posiciones_atacadas_log.clear() # Limpia el log en memoria
        print(" memoria del robot reiniciada.")

    except OSError as e:
        print(f"⚠️ ERROR: No se pudo borrar el archivo de log. Detalle: {e}")

# ######################################################
# ## NUEVO: FUNCIONES PARA MANEJAR EL LOG DE ATAQUES ##
# ######################################################

def cargar_log_ataques(archivo_log):
    """Carga las posiciones atacadas desde un archivo JSON."""
    try:
        with open(archivo_log, 'r') as f:
            log = json.load(f)
            # JSON guarda claves como string, las convertimos a int si es necesario
            return {int(k): v for k, v in log.items()}
    except FileNotFoundError:
        print(f"INFO: No se encontró el archivo '{archivo_log}'. Se creará uno nuevo.")
        return {} # Devuelve un diccionario vacío si el archivo no existe
    except json.JSONDecodeError:
        print(f"ADVERTENCIA: El archivo '{archivo_log}' está corrupto. Se iniciará un nuevo log.")
        return {}


def guardar_log_ataques(archivo_log, datos_log):
    """Guarda el diccionario de posiciones atacadas en un archivo JSON."""
    with open(archivo_log, 'w') as f:
        json.dump(datos_log, f, indent=4)

# ######################################################
# ## NUEVO: LÓGICA DEL SERVIDOR DE STREAMING DE VIDEO ##
# ######################################################
def generate_frames():
    global output_frame, lock
    # Bucle infinito para generar fotogramas para el stream
    while True:
        with lock:
            # Si no hay un fotograma disponible, no hacer nada
            if output_frame is None:
                continue
            # Codificar el fotograma a formato JPEG
            (flag, encoded_image) = cv2.imencode(".jpg", output_frame)
            if not flag:
                continue
        # Entregar el fotograma en formato de bytes
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
              bytearray(encoded_image) + b'\r\n')

@app.route("/video_feed")
def video_feed():
    # Devuelve la respuesta generada por la función generate_frames
    return Response(generate_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

# ######################################################
# ## CARGA DE CONFIGURACIÓN DESDE JSON ##
# ######################################################
try:
    with open(CONFIG_FILE, 'r') as f:
        config = json.load(f)
    PX_PER_CM = config['pixels_por_cm']
    ORIGEN_X, ORIGEN_Y = config['origen_grilla_px']
    print(f"✅ Configuración '{CONFIG_FILE}' cargada exitosamente.")
    print(f"   - Origen del Grid (px): ({ORIGEN_X}, {ORIGEN_Y})")
    print(f"   - Escala (px/cm): {PX_PER_CM:.2f}")
except FileNotFoundError:
    print(f"⚠️ ERROR: No se encontró el archivo de configuración '{CONFIG_FILE}'. Abortando.")
    sys.exit(1)
except KeyError as e:
    print(f"⚠️ ERROR: La clave {e} no se encontró en el archivo de configuración. Abortando.")
    sys.exit(1)

ATTACK_COOLDOWN = 5 # Segundos de espera antes de poder atacar la misma celda de nuevo
TARGET_FPS = 25

# ######################################################
# ## INICIALIZACIÓN DE LA COMUNICACIÓN SERIAL ##
# ######################################################
arduino = None
try:
    arduino = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=SERIAL_TIMEOUT)
    print(f"✅ Conectado al Arduino en el puerto {SERIAL_PORT}")
    time.sleep(2) # Espera a que la conexión se establezca
except serial.SerialException as e:
    print(f"⚠️ ERROR: No se pudo conectar al Arduino en {SERIAL_PORT}. El script correrá en modo de solo detección.")
    print(f"   Detalle del error: {e}")

# Check if model file exists and is valid
# (El resto de la validación inicial se mantiene igual)
if (not os.path.exists(model_path)):
    print('ERROR: Model path is invalid or model was not found. Make sure the model filename was entered correctly.')
    sys.exit(0)

model = YOLO(model_path, task='detect')
labels = model.names

img_ext_list = ['.jpg','.JPG','.jpeg','.JPEG','.png','.PNG','.bmp','.BMP']
vid_ext_list = ['.avi','.mov','.mp4','.mkv','.wmv']

if os.path.isdir(img_source):
    source_type = 'folder'
elif os.path.isfile(img_source):
    _, ext = os.path.splitext(img_source)
    if ext in img_ext_list:
        source_type = 'image'
    elif ext in vid_ext_list:
        source_type = 'video'
    else:
        print(f'File extension {ext} is not supported.')
        sys.exit(0)
elif 'usb' in img_source:
    source_type = 'usb'
    usb_idx = int(img_source[3:])
elif 'picamera' in img_source:
    source_type = 'picamera'
    picam_idx = int(img_source[8:])
else:
    print(f'Input {img_source} is invalid. Please try again.')
    sys.exit(0)

resize = False
if user_res:
    resize = True
    resW, resH = int(user_res.split('x')[0]), int(user_res.split('x')[1])

if record:
    if source_type not in ['video','usb']:
        print('Recording only works for video and camera sources. Please try again.')
        sys.exit(0)
    if not user_res:
        print('Please specify resolution to record video at.')
        sys.exit(0)
    
    record_name = 'demo1.avi'
    record_fps = 30
    recorder = cv2.VideoWriter(record_name, cv2.VideoWriter_fourcc(*'MJPG'), record_fps, (resW,resH))

# Load or initialize image source
if source_type == 'image':
    imgs_list = [img_source]
elif source_type == 'folder':
    imgs_list = []
    filelist = glob.glob(img_source + '/*')
    for file in filelist:
        _, file_ext = os.path.splitext(file)
        if file_ext in img_ext_list:
            imgs_list.append(file)
elif source_type == 'video' or source_type == 'usb':
    if source_type == 'video': cap_arg = img_source
    elif source_type == 'usb': cap_arg = usb_idx
    cap = cv2.VideoCapture(cap_arg)

    # Si el usuario no especificó resolución, obtenerla de la cámara
    if not user_res:
        resW = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        resH = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    else:
        ret = cap.set(3, resW)
        ret = cap.set(4, resH)

elif source_type == 'picamera':
    from picamera2 import Picamera2
    cap = Picamera2()
    cap.configure(cap.create_video_configuration(main={"format": 'RGB888', "size": (resW, resH)}))
    cap.start()

# ######################################################
# ## INICIALIZACIÓN Y CARGA DEL LOG ##
# ######################################################
posiciones_atacadas_log = cargar_log_ataques(LOG_FILE)
print(f"✅ Log de ataques cargado. {len(posiciones_atacadas_log)} posiciones ya han sido atacadas.")


# ######################################################
# ## NUEVO: BUCLE DE ESPERA DEL COMANDO DE INICIO ##
# ######################################################
if arduino:
    print("\n⏳ Esperando comando 'START_DETECTION' del Arduino para iniciar...")
    while True:
        if arduino.in_waiting > 0:
            comando_inicio = arduino.readline().decode('utf-8').strip()
            if comando_inicio == "START_DETECTION":
                print("Comando recibido. ¡Iniciando detección!")
                break # Sale del bucle de espera y continúa con el script
        
        time.sleep(0.1) # Pequeña pausa para no saturar el CPU

# ######################################################
# ## PRE-CÁLCULO DEL GRID Y VARIABLES DE CONTROL ##
# ######################################################
# --- Cálculos basados en la configuración JSON y medidas en cm ---
GRID_WIDTH_PX = GRID_COLS_CM * PX_PER_CM
GRID_HEIGHT_PX = GRID_ROWS_CM * PX_PER_CM

# Asumimos que cada celda del grid mide 1x1 cm
CELL_WIDTH_PX = 1 * PX_PER_CM
CELL_HEIGHT_PX = 1 * PX_PER_CM

bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106), 
               (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]

avg_frame_rate = 0
frame_rate_buffer = []
fps_avg_len = 200
img_count = 0

if __name__ == '__main__':
    # --- NUEVO: Iniciar el servidor Flask en un hilo separado ---
    print("🚀 Iniciando el servidor de streaming en segundo plano...")
    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=8000, debug=False))
    flask_thread.daemon = True
    flask_thread.start()
    
    # Begin inference loop
    while True:

        # ######################################################
        # ## NUEVO: OYENTE DE COMANDOS DEL ARDUINO ##
        # ######################################################
        if arduino and arduino.in_waiting > 0:
            # Lee el comando enviado desde Arduino sin bloquear el script
            comando_arduino = arduino.readline().decode('utf-8').strip()
            
            if comando_arduino == "NAVIGATING":
                print("\nINFO: Recibido comando para entrar en modo navegación.")
                resetear_log()

        t_start = time.perf_counter()

        # (El código para cargar el frame se mantiene igual)
        if source_type == 'image' or source_type == 'folder':
            if img_count >= len(imgs_list):
                print('All images have been processed. Exiting program.')
                sys.exit(0)
            img_filename = imgs_list[img_count]
            frame = cv2.imread(img_filename)
            img_count = img_count + 1
        elif source_type == 'video':
            ret, frame = cap.read()
            if not ret:
                print('Reached end of the video file. Exiting program.')
                break
        elif source_type == 'usb':
            ret, frame = cap.read()
            if (frame is None) or (not ret):
                print('Unable to read frames from the camera. This indicates the camera is disconnected or not working. Exiting program.')
                break
        elif source_type == 'picamera':
            frame = cap.capture_array()
            if (frame is None):
                print('Unable to read frames from the Picamera. This indicates the camera is disconnected or not working. Exiting program.')
                break

        if resize:
            frame = cv2.resize(frame, (resW, resH))

        # En tu bucle principal
        t_inferencia_inicio = time.perf_counter()
        # Run inference on frame
        results = model(frame, verbose=False)
        t_inferencia_fin = time.perf_counter()
        print(f"Tiempo de inferencia: {t_inferencia_fin - t_inferencia_inicio:.4f} segundos")

        detections = results[0].boxes
        object_count = 0

        # ######################################################
        # ## DIBUJAR EL GRID EN LA IMAGEN ##
        # ######################################################
        # Dibuja el borde exterior del grid
        pt1 = (ORIGEN_X, ORIGEN_Y)
        pt2 = (int(ORIGEN_X + GRID_WIDTH_PX), int(ORIGEN_Y + GRID_HEIGHT_PX))
        cv2.rectangle(frame, pt1, pt2, (0, 255, 0), 2)

        #Dibuja las líneas internas (opcional, puede ser pesado visualmente)
        for row in range(1, GRID_ROWS_CM):
            y = int(ORIGEN_Y + row * CELL_HEIGHT_PX)
            cv2.line(frame, (ORIGEN_X, y), (int(ORIGEN_X + GRID_WIDTH_PX), y), (0, 255, 0), 1)
        for col in range(1, GRID_COLS_CM):
            x = int(ORIGEN_X + col * CELL_WIDTH_PX)
            cv2.line(frame, (x, ORIGEN_Y), (x, int(ORIGEN_Y + GRID_HEIGHT_PX)), (0, 255, 0), 1)

        # Go through each detection
        for i in range(len(detections)):
            xyxy_tensor = detections[i].xyxy.cpu()
            xyxy = xyxy_tensor.numpy().squeeze()
            xmin, ymin, xmax, ymax = xyxy.astype(int)

            centerX = int((xmin + xmax) / 2)
            centerY = int((ymin + ymax) / 2)

            cv2.circle(frame, (centerX, centerY), 5, (0, 0, 255), -1)

            classidx = int(detections[i].cls.item())
            classname = labels[classidx]
            conf = detections[i].conf.item()

            if conf > min_thresh:
                object_count += 1
                color = bbox_colors[classidx % 10]
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
                label = f'{classname}: {int(conf*100)}%'
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                label_ymin = max(ymin, labelSize[1] + 10)
                cv2.rectangle(frame, (xmin, label_ymin - labelSize[1] - 10), (xmin + labelSize[0], label_ymin + baseLine - 10), color, cv2.FILLED)
                cv2.putText(frame, label, (xmin, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

                # ######################################################
                # ## LÓGICA DE DETECCIÓN Y ATAQUE ##
                # ######################################################
                if classname == TARGET_CLASS and arduino is not None:
                    # --- NUEVO: Verificar si la detección está DENTRO del grid ---
                    is_inside_grid = (ORIGEN_X <= centerX < ORIGEN_X + GRID_WIDTH_PX) and \
                                    (ORIGEN_Y <= centerY < ORIGEN_Y + GRID_HEIGHT_PX)

                    if is_inside_grid:
                        # Calcular coordenadas relativas al origen del grid
                        relative_x = centerX - ORIGEN_X
                        relative_y = centerY - ORIGEN_Y

                        # Calcular en qué celda está el objeto
                        grid_col = int(relative_x // CELL_WIDTH_PX)
                        grid_row = int(relative_y // CELL_HEIGHT_PX)
                        grid_position = grid_row * GRID_COLS_CM + grid_col

                        if grid_position not in posiciones_atacadas_log:
                    
                            # (Envío del comando "ATTACK" al Arduino)
                            command = f"ATTACK,{grid_position}\n"
                            arduino.write(command.encode('utf-8'))
                            print(f"¡{TARGET_CLASS} en celda {grid_position}! Enviando comando...")
                            cv2.putText(frame, f"ATTACK: {grid_position}", (centerX, centerY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                        # ######################################################
                        # ## NUEVO: BUCLE DE ESPERA DE CONFIRMACIÓN ##
                        # ######################################################
                        print("⏳ Esperando confirmación del Arduino...")
                        
                        # Pausar el video temporalmente mostrando un mensaje
                        overlay = frame.copy()
                        cv2.putText(overlay, "ATACANDO...", (resW // 2 - 100, resH // 2), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                        cv2.imshow('YOLO detection results', overlay)
                        cv2.waitKey(1) # Esencial para que el overlay se muestre
                        
                        t_espera_inicio = time.perf_counter()
                        response = arduino.readline().decode('utf-8').strip()
                        t_espera_fin = time.perf_counter()
                        print(f"Tiempo de espera del Arduino: {t_espera_fin - t_espera_inicio:.4f} segundos")

                        if response == "DONE":
                            print("Confirmación recibida. Guardando en el log permanente.")
                            posiciones_atacadas_log[grid_position] = time.strftime("%Y-%m-%d %H:%M:%S")
                            guardar_log_ataques(LOG_FILE, posiciones_atacadas_log)
                        else:
                            print(f"Timeout o respuesta inesperada: '{response}'.")
                        break 

        if source_type in ['video', 'usb', 'picamera']:
            cv2.putText(frame, f'FPS: {avg_frame_rate:0.2f}', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 255), 2)
        
        cv2.putText(frame, f'Number of objects: {object_count}', (10, 40), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 255), 2)
        
        # --- NUEVO: Actualizar el fotograma para el streaming ---
        # Después de dibujar todo en el 'frame', lo compartimos con el servidor
        with lock:
            output_frame = frame.copy() 

        cv2.imshow('YOLO detection results', frame)
        if record: recorder.write(frame)

        # --- INICIO DEL BLOQUE PARA LIMITAR FPS ---
        target_frame_duration = 1 / TARGET_FPS
        elapsed_time = time.perf_counter() - t_start
        
        # Calcula el tiempo que falta para completar el segundo y haz una pausa
        sleep_time = target_frame_duration - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)

        # El cálculo de FPS ahora debería ser cercano a 1
        t_stop = time.perf_counter()
        frame_rate_calc = 1 / (t_stop - t_start)
        # --- FIN DEL BLOQUE PARA LIMITAR FPS ---

        # Cambiamos waitKey a 1ms para que la ventana no se congele
        key = cv2.waitKey(1) 

        if key == ord('q') or key == ord('Q'): # Press 'q' to quit
            break
        elif key == ord('s') or key == ord('S'): # Press 's' to pause inference
            cv2.waitKey()
        elif key == ord('p') or key == ord('P'): # Press 'p' to save a picture of results on this frame
            cv2.imwrite('capture.png',frame)
        
        # Append FPS result to frame_rate_buffer (for finding average FPS over multiple frames)
        if len(frame_rate_buffer) >= fps_avg_len:
            temp = frame_rate_buffer.pop(0)
            frame_rate_buffer.append(frame_rate_calc)
        else:
            frame_rate_buffer.append(frame_rate_calc)

        # Calculate average FPS for past frames
        avg_frame_rate = np.mean(frame_rate_buffer)

# ######################################################
# ## LIMPIEZA DE RECURSOS ##
# ######################################################
print(f'Average pipeline FPS: {avg_frame_rate:.2f}')
if arduino is not None:
    arduino.close() # <-- CERRAR CONEXIÓN SERIAL
    print("🔌 Conexión serial con Arduino cerrada.")

if source_type in ['video', 'usb']:
    cap.release()
elif source_type == 'picamera':
    cap.stop()
if record: recorder.release()
cv2.destroyAllWindows()