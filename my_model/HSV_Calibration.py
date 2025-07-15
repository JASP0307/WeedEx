import cv2
import numpy as np

def nothing(x):
    pass

# --- CONFIGURACIÓN ---
CAMERA_INDEX = 0  # Cambia esto al índice de tu cámara si es diferente (ej: 1 para la segunda cámara)
WINDOW_NAME = "HSV Calibrator"
IMAGE_NAME = "captured_image.png"

# --- FUNCIONES ---
def capture_image(camera_index, output_name):
    """Captura una imagen desde la cámara y la guarda."""
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Error: No se pudo acceder a la cámara con índice {camera_index}")
        return False
    
    ret, frame = cap.read()
    cap.release()
    
    if ret:
        cv2.imwrite(output_name, frame)
        print(f"Imagen guardada como {output_name}")
        return True
    else:
        print("Error al capturar la imagen.")
        return False

def calibrate_hsv(image_path):
    """Permite al usuario calibrar los rangos HSV para un color específico en una imagen."""
    img = cv2.imread(image_path)
    if img is None:
        print(f"Error: No se pudo cargar la imagen desde {image_path}")
        return
    
    cv2.namedWindow(WINDOW_NAME)
    
    # Crear barras de seguimiento para los rangos HSV
    cv2.createTrackbar("H Min", WINDOW_NAME, 0, 179, nothing)
    cv2.createTrackbar("H Max", WINDOW_NAME, 179, 179, nothing)
    cv2.createTrackbar("S Min", WINDOW_NAME, 0, 255, nothing)
    cv2.createTrackbar("S Max", WINDOW_NAME, 255, 255, nothing)
    cv2.createTrackbar("V Min", WINDOW_NAME, 0, 255, nothing)
    cv2.createTrackbar("V Max", WINDOW_NAME, 255, 255, nothing)
    
    while(True):
        h_min = cv2.getTrackbarPos("H Min", WINDOW_NAME)
        h_max = cv2.getTrackbarPos("H Max", WINDOW_NAME)
        s_min = cv2.getTrackbarPos("S Min", WINDOW_NAME)
        s_max = cv2.getTrackbarPos("S Max", WINDOW_NAME)
        v_min = cv2.getTrackbarPos("V Min", WINDOW_NAME)
        v_max = cv2.getTrackbarPos("V Max", WINDOW_NAME)
        
        lower_bound = np.array([h_min, s_min, v_min])
        upper_bound = np.array([h_max, s_max, v_max])
        
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        result = cv2.bitwise_and(img, img, mask=mask)
        
        cv2.imshow(WINDOW_NAME, result)
        
        k = cv2.waitKey(1) & 0xFF
        if k == 27: # Presiona ESC para salir
            break
        elif k == ord('s'): # Presiona 's' para guardar los valores
            print("Valores HSV encontrados:")
            print(f"RANGO_AMARILLO_HSV = (")
            print(f"    np.array([{h_min}, {s_min}, {v_min}]),  # Límite inferior (H, S, V)")
            print(f"    np.array([{h_max}, {s_max}, {v_max}])   # Límite superior (H, S, V)")
            print(f")")
            break
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    if capture_image(CAMERA_INDEX, IMAGE_NAME):
        calibrate_hsv(IMAGE_NAME)
    else:
        print("No se pudo capturar la imagen para calibración.")