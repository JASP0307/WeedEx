import cv2
import numpy as np
import json
import os

# --- CONFIGURACIÓN ---
NOMBRE_ARCHIVO_CONFIG = "config_grilla.json"
GRILLA_FILAS = 2
GRILLA_COLUMNAS = 19
ANCHO_HILERA_REAL_CM = 19.0 # Ancho real del objeto de referencia

# --- VARIABLES GLOBALES ---
puntos_calibracion = []
# 0: Inactivo, 1: Esperando puntos de escala, 2: Esperando punto de origen Y
modo_calibracion = 0
datos_calibracion = {
    "pixels_por_cm": None,
    "posicion_y": 300  # Valor por defecto si no hay archivo de config
}

# --- FUNCIÓN DE CALLBACK PARA EL MOUSE ---
def manejador_clic(event, x, y, flags, param):
    global puntos_calibracion, modo_calibracion, datos_calibracion

    if event == cv2.EVENT_LBUTTONDOWN:
        if modo_calibracion == 1:
            # Captura los dos puntos para medir la escala
            if len(puntos_calibracion) < 2:
                puntos_calibracion.append((x, y))
                print(f"Punto {len(puntos_calibracion)}/2 capturado en ({x}, {y})")
            if len(puntos_calibracion) == 2:
                print("¡Puntos de escala capturados! Presiona 'c' de nuevo para definir la altura Y.")
                modo_calibracion = 2 # Pasa al siguiente paso

        elif modo_calibracion == 2:
            # Captura el punto de origen para la grilla
            datos_calibracion["posicion_y"] = y
            print(f"Altura Y de la grilla definida en: {y}")
            print("¡Calibración completada! Presiona 's' para guardar.")
            modo_calibracion = 0 # Finaliza el modo de calibración

# --- FUNCIONES AUXILIARES ---
def cargar_configuracion():
    """Carga los datos de calibración desde el archivo JSON."""
    global datos_calibracion
    if os.path.exists(NOMBRE_ARCHIVO_CONFIG):
        try:
            with open(NOMBRE_ARCHIVO_CONFIG, 'r') as f:
                datos_cargados = json.load(f)
                # Asegurarse de que las claves necesarias existen
                if "pixels_por_cm" in datos_cargados and "posicion_y" in datos_cargados:
                    datos_calibracion = datos_cargados
                    print("Configuración de grilla cargada exitosamente.")
                    return True
        except (json.JSONDecodeError, KeyError) as e:
            print(f"Error al leer el archivo de configuración: {e}. Se usarán valores por defecto.")
            return False
            
    print("No se encontró archivo de configuración. Inicia la calibración con la tecla 'c'.")
    return False

def guardar_configuracion():
    """Guarda los datos de calibración actuales en el archivo JSON."""
    if datos_calibracion.get("pixels_por_cm") and datos_calibracion.get("posicion_y") is not None:
        with open(NOMBRE_ARCHIVO_CONFIG, 'w') as f:
            json.dump(datos_calibracion, f, indent=4)
            print(f"Configuración guardada en {NOMBRE_ARCHIVO_CONFIG}")
    else:
        print("Error: No hay datos de calibración para guardar. Completa la calibración primero.")

def dibujar_grilla(frame, origen, pixels_por_cm):
    """Dibuja la grilla en el frame de la cámara."""
    if not origen or not pixels_por_cm:
        return
    
    ox, oy = origen
    ancho_celda_px = int(1.0 * pixels_por_cm)
    alto_celda_px = ancho_celda_px

    # Dibuja las líneas de la grilla
    for i in range(GRILLA_FILAS + 1):
        y = oy + i * alto_celda_px
        cv2.line(frame, (ox, y), (ox + GRILLA_COLUMNAS * ancho_celda_px, y), (255, 0, 0), 1)
    
    for j in range(GRILLA_COLUMNAS + 1):
        x = ox + j * ancho_celda_px
        cv2.line(frame, (x, oy), (x, oy + GRILLA_FILAS * alto_celda_px), (255, 0, 0), 1)

# --- PROGRAMA PRINCIPAL ---
cap = cv2.VideoCapture(0)
cv2.namedWindow("Calibrador de Grilla")
cv2.setMouseCallback("Calibrador de Grilla", manejador_clic)

calibrado = cargar_configuracion()

print("\n--- INSTRUCCIONES ---")
print(" 'c' -> Iniciar/Proseguir calibración")
print(" 's' -> Guardar configuración tras calibrar")
print(" 'r' -> Resetear calibración")
print(" 'q' -> Salir")
print("---------------------\n")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    if datos_calibracion.get("pixels_por_cm"):
        pixels_por_cm = datos_calibracion["pixels_por_cm"]
        ancho_total_grilla_px = int(GRILLA_COLUMNAS * pixels_por_cm)
        ancho_ventana = frame.shape[1]

        origen_x = (ancho_ventana - ancho_total_grilla_px) // 2
        origen_y = datos_calibracion["posicion_y"]
        
        dibujar_grilla(frame, (origen_x, origen_y), pixels_por_cm)

    if modo_calibracion == 1:
        texto = f"PASO 1: Haz clic en los 2 extremos de un objeto de {ANCHO_HILERA_REAL_CM} cm"
        cv2.putText(frame, texto, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        for punto in puntos_calibracion:
            cv2.circle(frame, punto, 5, (0, 0, 255), -1)

    elif modo_calibracion == 2:
        texto = "PASO 2: Haz clic en cualquier lugar para definir la altura (eje Y) de la grilla"
        cv2.putText(frame, texto, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    cv2.imshow("Calibrador de Grilla", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break
    
    elif key == ord('c'):
        if modo_calibracion == 0:
            puntos_calibracion = []
            modo_calibracion = 1
            print("\nMODO CALIBRACION: PASO 1 (Escala). Esperando 2 clics...")
        
        elif modo_calibracion == 2:
             # Este bloque no hace nada, se espera un clic que es manejado por el callback
            print("\nMODO CALIBRACION: PASO 2 (Altura Y). Esperando 1 clic...")

    elif key == ord('s'):
        if len(puntos_calibracion) == 2:
            distancia_pixeles = np.sqrt((puntos_calibracion[1][0] - puntos_calibracion[0][0])**2 + 
                                        (puntos_calibracion[1][1] - puntos_calibracion[0][1])**2)
            datos_calibracion["pixels_por_cm"] = distancia_pixeles / ANCHO_HILERA_REAL_CM
            print(f"-> Escala calculada: {datos_calibracion['pixels_por_cm']:.2f} pixeles por cm.")
            
            guardar_configuracion()
            calibrado = True
        else:
            print("No se puede guardar. Debes completar la calibración primero.")

    elif key == ord('r'):
        modo_calibracion = 0
        puntos_calibracion = []
        # Eliminar el archivo de configuración para un reseteo completo
        if os.path.exists(NOMBRE_ARCHIVO_CONFIG):
            os.remove(NOMBRE_ARCHIVO_CONFIG)
            print("Archivo de configuración eliminado.")
        datos_calibracion = {"pixels_por_cm": None, "posicion_y": 300}
        print("Calibración reseteada. Presiona 'c' para iniciar de nuevo.")

cap.release()
cv2.destroyAllWindows()