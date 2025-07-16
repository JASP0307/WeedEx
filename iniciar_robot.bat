@echo off
REM Archivo para iniciar el script de detección del robot.

REM 1. Cambiar al directorio del proyecto.
REM Reemplaza la ruta con la tuya. Usa la ruta absoluta.
cd C:\Users\User\Documents\WeedEx-1

REM 2. Activar el entorno virtual de Python.
REM Asegúrate de que esta ruta a 'activate.bat' es correcta.
call C:\Users\User\Documents\WeedEx-1\.venv\Scripts\activate.bat

REM 3. Ejecutar el script de Python con sus argumentos.
REM Modifica esta línea con los argumentos que necesitas.
echo Lanzando el script de deteccion...
cd my_model
python Yolo_Prueba_v2.py --model my_modelV11_2.pt --source usb0 --resolution 1080x720

REM 4. (Opcional) Pausa para ver errores.
REM Si la ventana se cierra muy rápido, quita el 'REM' de la línea de abajo para depurar.
