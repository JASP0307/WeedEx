import serial
import matplotlib.pyplot as plt
from collections import deque
import time

# Configura el puerto serial y velocidad
PORT = 'COM3'  # Cambia esto al puerto de tu Arduino
BAUD = 9600

# Tamaño del buffer (número de muestras mostradas en el gráfico)
BUFFER_SIZE = 100

# Inicializa buffers
rpm_vals = deque(maxlen=BUFFER_SIZE)
sp_vals = deque(maxlen=BUFFER_SIZE)
timestamps = deque(maxlen=BUFFER_SIZE)

# Configura la gráfica
plt.ion()
fig, ax = plt.subplots()
line1, = ax.plot([], [], label='RPM actual')
line2, = ax.plot([], [], label='Setpoint')
ax.set_ylim(0, 120)  # Ajusta si tu motor tiene otros límites
ax.set_xlabel("Tiempo (s)")
ax.set_ylabel("RPM")
ax.set_title("Respuesta del control PID")
ax.legend()
ax.grid(True)

with serial.Serial(PORT, BAUD, timeout=1) as ser:
    time.sleep(2)  # Espera a que el Arduino reinicie
    start_time = time.time()

    while True:
        try:
            line = ser.readline().decode().strip()
            if not line or "," not in line:
                continue

            sp_str, rpm_str = line.split(",")
            sp = float(sp_str)
            rpm = float(rpm_str)
            now = time.time() - start_time

            sp_vals.append(sp)
            rpm_vals.append(rpm)
            timestamps.append(now)

            line1.set_data(timestamps, rpm_vals)
            line2.set_data(timestamps, sp_vals)
            ax.relim()
            ax.autoscale_view()
            plt.pause(0.01)

        except KeyboardInterrupt:
            print("Finalizado por el usuario.")
            break
        except Exception as e:
            print(f"Error: {e}")
