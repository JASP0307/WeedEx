import torch_directml

# Listar dispositivos disponibles
num_devices = torch_directml.device_count()
print(f"🧠 GPUs disponibles con DirectML: {num_devices}")

# Mostrar información del primer dispositivo
for i in range(num_devices):
    device = torch_directml.device(i)
    print(f"📦 Dispositivo {i}: {device}")
