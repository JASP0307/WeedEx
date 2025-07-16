import cv2
import numpy as np

# ##################################################################
# ## ⚙️ CONFIGURACIÓN: ¡MODIFICA ESTOS VALORES! ##
# ##################################################################

# 1. Especifica la ruta a la nueva imagen que quieres probar.
#    Asegúrate de que la imagen esté en la misma carpeta que el script,
#    o proporciona la ruta completa.
IMAGEN_A_PROBAR = 'captured_image.png' # <--- CAMBIA ESTO

# 2. Pega aquí los valores que obtuviste del script de calibración.
#    Estos son solo un ejemplo, ¡debes usar los tuyos!
RANGO_AMARILLO_HSV = (
    np.array([112, 87, 186]),  # Límite inferior (H, S, V)
    np.array([128, 122, 206])   # Límite superior (H, S, V)
)

# 3. Define el área mínima en píxeles para considerar una detección válida.
#    Este valor debe ser el mismo que usas en tu script principal.
MIN_AREA_IMPACTO = 120

# ##################################################################
# ## CÓDIGO DE PRUEBA (No necesitas modificar debajo de esta línea) ##
# ##################################################################

# Cargar la imagen
img_original = cv2.imread(IMAGEN_A_PROBAR)

if img_original is None:
    print(f"❌ ERROR: No se pudo cargar la imagen desde '{IMAGEN_A_PROBAR}'.")
    print("Asegúrate de que el nombre del archivo es correcto y está en la carpeta adecuada.")
else:
    # Convertir la imagen al espacio de color HSV
    hsv = cv2.cvtColor(img_original, cv2.COLOR_BGR2HSV)

    # Crear la máscara usando los rangos HSV definidos
    lower_bound, upper_bound = RANGO_AMARILLO_HSV
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # Encontrar contornos en la máscara
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    impacto_detectado = False
    # Dibujar los contornos encontrados que superen el área mínima
    for cnt in contours:
        if cv2.contourArea(cnt) > MIN_AREA_IMPACTO:
            impacto_detectado = True
            # Dibuja el contorno en verde sobre la imagen original
            cv2.drawContours(img_original, [cnt], -1, (0, 255, 0), 3)

    # Mostrar mensaje de resultado en la imagen
    if impacto_detectado:
        print("✅ ¡Impacto detectado con los valores HSV proporcionados!")
        cv2.putText(img_original, "IMPACTO DETECTADO", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    else:
        print("❌ No se detectó ningún impacto con los valores HSV actuales.")
        cv2.putText(img_original, "NO SE DETECTO IMPACTO", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # Crear una versión de la máscara en 3 canales para poder apilarla con la original
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    # Apilar la imagen original (con el resultado) y la máscara para verlas juntas
    resultado_combinado = np.hstack([img_original, mask_bgr])
    
    # Redimensionar si la imagen es muy grande para la pantalla
    h, w, _ = resultado_combinado.shape
    if w > 1900: # Si el ancho combinado es mayor a 1900px
        escala = 1900 / w
        nuevo_ancho = int(w * escala)
        nuevo_alto = int(h * escala)
        resultado_combinado = cv2.resize(resultado_combinado, (nuevo_ancho, nuevo_alto))


    # Mostrar la imagen
    cv2.imshow("Resultados de la Prueba: Original (Izquierda) vs Mascara HSV (Derecha)", resultado_combinado)
    print("\nPresiona cualquier tecla para cerrar la ventana.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()