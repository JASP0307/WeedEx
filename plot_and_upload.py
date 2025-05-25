import subprocess
import os

# 1. Subir el firmware
upload_result = subprocess.run(["pio", "run", "-t", "upload"])

# 2. Si la carga fue exitosa, lanza el script de graficado
if upload_result.returncode == 0:
    plotter_path = os.path.join("tools", "plot_serial.py")
    subprocess.Popen(["python", plotter_path])
else:
    print("Error en la carga del firmware. No se ejecutará el graficador.")
