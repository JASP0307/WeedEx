import subprocess
import os

# Subir el firmware del entorno de prueba
upload_result = subprocess.run(["pio", "run", "-e", "pid_test", "-t", "upload"])

# Si fue exitoso, lanza el graficador
if upload_result.returncode == 0:
    plot_script = os.path.join("tools", "plot_serial.py")
    subprocess.Popen(["python", plot_script])
else:
    print("Error al subir el firmware de prueba. No se ejecutará el graficador.")
