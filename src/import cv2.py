import cv2
import numpy as np
import json
import os

# --- CONFIGURACIÓN ---
NOMBRE_ARCHIVO_CONFIG = "config_grilla.json"
GRILLA_FILAS = 2
GRILLA_COLUMNAS = 10
ANCHO_HILERA_REAL_CM = 10.0 # Ancho real del objeto de referencia

# NUEVO PARÁMETRO: Define la altura (en píxeles desde arriba) donde se dibujará la grilla.
GRILLA_POSICION_Y = 20

# --- VARIABLES GLOBALES PARA CALIBRACIÓN ---
puntos_calibracion = []
modo_calibracion_activo = False
datos_calibracion = {
    "pixels_por_cm": None
}

# --- FUNCIÓN DE CALLBACK PARA EL MOUSE ---
def manejador_clic(event, x, y, flags, param):
    global puntos_calibracion, modo_calibracion_activo
    
    if event == cv2.EVENT_LBUTTONDOWN and modo_calibracion_activo:
        # Captura los dos puntos para medir la escala
        if len(puntos_calibracion) < 2:
            puntos_calibracion.append((x, y))
            print(f"Punto {len(puntos_calibracion)}/2 capturado en ({x}, {y})")
        if len(puntos_calibracion) == 2:
            print("¡Puntos de escala capturados! Presiona 's' para calcular y guardar.")
            modo_calibracion_activo = False # Finaliza el modo de calibración

# --- FUNCIONES AUXILIARES ---
def cargar_configuracion():
    """Carga los datos de calibración desde el archivo JSON."""
    global datos_calibracion
    if os.path.exists(NOMBRE_ARCHIVO_CONFIG):
        with open(NOMBRE_ARCHIVO_CONFIG, 'r') as f:
            datos_calibracion = json.load(f)
            print("Configuración de grilla cargada exitosamente.")
            return True
    print("No se encontró archivo de configuración. Inicia la calibración con la tecla 'c'.")
    return False

def guardar_configuracion():
    """Guarda los datos de calibración actuales en el archivo JSON."""
    if datos_calibracion["pixels_por_cm"]:
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
    # El ancho y alto de la celda ahora es 1cm real
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
cap = cv2.VideoCapture(1) # O el índice de tu cámara
# Configura tu resolución si es necesario
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

cv2.namedWindow("Calibrador de Grilla Centrada")
cv2.setMouseCallback("Calibrador de Grilla Centrada", manejador_clic)

# Intenta cargar la configuración al inicio
calibrado = cargar_configuracion()

print("\n--- INSTRUCCIONES ---")
print(" 'c' -> Iniciar calibración (2 clics)")
print(" 's' -> Guardar configuración tras calibrar")
print(" 'r' -> Resetear calibración")
print(" 'q' -> Salir")
print("---------------------\n")


while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Dibuja la grilla si el sistema ya está calibrado
    if calibrado:
        # --- CÁLCULO DINÁMICO DEL ORIGEN ---
        pixels_por_cm = datos_calibracion["pixels_por_cm"]
        ancho_total_grilla_px = int(GRILLA_COLUMNAS * pixels_por_cm)
        ancho_ventana = frame.shape[1]

        # Calcula el origen X para centrar la grilla
        origen_x = (ancho_ventana - ancho_total_grilla_px) // 2
        
        # El origen Y se toma del parámetro de configuración
        origen_y = GRILLA_POSICION_Y
        
        dibujar_grilla(frame, (origen_x, origen_y), pixels_por_cm)

    # Lógica de visualización para el modo de calibración
    if modo_calibracion_activo:
        texto = f"MODO CALIBRACION: Haz clic en los 2 extremos de un objeto de {ANCHO_HILERA_REAL_CM} cm"
        cv2.putText(frame, texto, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        for punto in puntos_calibracion:
            cv2.circle(frame, punto, 5, (0, 0, 255), -1)


    cv2.imshow("Calibrador de Grilla Centrada", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break
    
    elif key == ord('c'):
        # Inicia el proceso de calibración
        puntos_calibracion = []
        modo_calibracion_activo = True
        calibrado = False
        print("\nMODO CALIBRACION: Esperando 2 clics del mouse...")
    
    elif key == ord('s'):
        if len(puntos_calibracion) == 2:
            # Procesa los puntos para calcular la escala
            distancia_pixeles = np.sqrt((puntos_calibracion[1][0] - puntos_calibracion[0][0])**2 + 
                                        (puntos_calibracion[1][1] - puntos_calibracion[0][1])**2)
            
            datos_calibracion["pixels_por_cm"] = distancia_pixeles / ANCHO_HILERA_REAL_CM
            print(f"-> Escala calculada: {datos_calibracion['pixels_por_cm']:.2f} pixeles por cm.")
            
            guardar_configuracion()
            calibrado = True # Marca el sistema como calibrado
        else:
            print("No se puede guardar. Debes hacer 2 clics en modo calibración primero.")

    elif key == ord('r'):
        # Resetea la calibración
        modo_calibracion_activo = False
        calibrado = False
        datos_calibracion = {"pixels_por_cm": None}
        puntos_calibracion = []
        print("Calibración reseteada. Presiona 'c' para iniciar de nuevo.")


cap.release()
cv2.destroyAllWindows()