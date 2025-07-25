import time
import smbus2
import json
import paho.mqtt.client as mqtt
from collections import deque

# === Configuración MQTT ===
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC = "sensores/datos"

def publicar_mqtt(payload):
    try:
        client = mqtt.Client()
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.publish(MQTT_TOPIC, json.dumps(payload))
        client.disconnect()
        print("✅ Publicado a MQTT:", payload)
    except Exception as e:
        print(f"❌ Error al publicar MQTT: {e}")

# === I2C / SMBus ===
smbus = smbus2.SMBus(1)

# Direcciones
BME280_ADDR = 0x76
MPU6050_ADDR = 0x68
MLX90614_ADDR = 0x5A

# === Activar MPU6050 ===
smbus.write_byte_data(MPU6050_ADDR, 0x6B, 0)

# === Inicializar BME280 ===
def write_bme280(addr, reg, data):
    smbus.write_byte_data(addr, reg, data)

write_bme280(BME280_ADDR, 0xF2, 0x01)
write_bme280(BME280_ADDR, 0xF4, 0x27)
write_bme280(BME280_ADDR, 0xF5, 0xA0)
time.sleep(0.5)

# === Leer coeficientes calibración ===
def read_calibration():
    def u16(reg):
        return smbus.read_byte_data(BME280_ADDR, reg) | (smbus.read_byte_data(BME280_ADDR, reg + 1) << 8)
    def s16(reg):
        result = u16(reg)
        if result > 32767:
            result -= 65536
        return result

    return {
        'T1': u16(0x88), 'T2': s16(0x8A), 'T3': s16(0x8C),
        'P1': u16(0x8E), 'P2': s16(0x90), 'P3': s16(0x92),
        'P4': s16(0x94), 'P5': s16(0x96), 'P6': s16(0x98),
        'P7': s16(0x9A), 'P8': s16(0x9C), 'P9': s16(0x9E),
        'H1': smbus.read_byte_data(BME280_ADDR, 0xA1),
        'H2': s16(0xE1), 'H3': smbus.read_byte_data(BME280_ADDR, 0xE3),
        'H4': (smbus.read_byte_data(BME280_ADDR, 0xE4) << 4) | (smbus.read_byte_data(BME280_ADDR, 0xE5) & 0x0F),
        'H5': (smbus.read_byte_data(BME280_ADDR, 0xE6) << 4) | (smbus.read_byte_data(BME280_ADDR, 0xE5) >> 4),
        'H6': smbus.read_byte_data(BME280_ADDR, 0xE7)
    }

# === Lectura y compensación BME280 ===
def read_bme280_raw():
    data = smbus.read_i2c_block_data(BME280_ADDR, 0xF7, 8)
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
    hum_raw  = (data[6] << 8) | data[7]
    return temp_raw, pres_raw, hum_raw

def compensate(temp_raw, pres_raw, hum_raw, calib):
    var1 = (((temp_raw >> 3) - (calib['T1'] << 1)) * calib['T2']) >> 11
    var2 = (((((temp_raw >> 4) - calib['T1']) * ((temp_raw >> 4) - calib['T1'])) >> 12) * calib['T3']) >> 14
    t_fine = var1 + var2

    # Presión
    var1 = t_fine - 128000
    var2 = var1 * var1 * calib['P6']
    var2 += ((var1 * calib['P5']) << 17)
    var2 += (calib['P4'] << 35)
    var1 = ((var1 * var1 * calib['P3']) >> 8) + ((var1 * calib['P2']) << 12)
    var1 = (((1 << 47) + var1) * calib['P1']) >> 33
    if var1 == 0:
        pressure = 0
    else:
        p = 1048576 - pres_raw
        p = (((p << 31) - var2) * 3125) // var1
        var1 = (calib['P9'] * (p >> 13) * (p >> 13)) >> 25
        var2 = (calib['P8'] * p) >> 19
        pressure = ((p + var1 + var2) >> 8) + (calib['P7'] << 4)
        pressure = pressure / 256.0 / 100.0

    # Humedad corregida
    h = t_fine - 76800
    h = (((((hum_raw << 14) - (calib['H4'] << 20) - (calib['H5'] * h)) + 16384) >> 15) *
         (((((((h * calib['H6']) >> 10) * (((h * calib['H3']) >> 11) + 32768)) >> 10) + 2097152) *
           calib['H2'] + 8192) >> 14)) >> 10
    h = h - (((((h >> 15) * (h >> 15)) >> 7) * calib['H1']) >> 4)
    h = max(0, min(h, 419430400)) >> 12
    humidity = h / 1024.0

    return pressure, humidity

# === MPU6050 (aceleración + giroscopio) ===
def read_mpu6050():
    def read_word(reg):
        high = smbus.read_byte_data(MPU6050_ADDR, reg)
        low = smbus.read_byte_data(MPU6050_ADDR, reg + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value

    ax = read_word(0x3B) / 16384.0
    ay = read_word(0x3D) / 16384.0
    az = read_word(0x3F) / 16384.0
    gx = read_word(0x43) / 131.0
    gy = read_word(0x45) / 131.0
    gz = read_word(0x47) / 131.0
    return ax, ay, az, gx, gy, gz

# === MLX90614 ===
def read_mlx90614_temp():
    raw = smbus.read_word_data(MLX90614_ADDR, 0x07)
    return (raw * 0.02) - 273.15

# === Paso avanzado ===
calib = read_calibration()
step_count = 0
last_step_time = 0
acc_window = deque(maxlen=5)

print("✅ Sistema con filtro avanzado iniciado")

while True:
    # Leer sensores
    temp_raw, pres_raw, hum_raw = read_bme280_raw()
    pressure, humidity = compensate(temp_raw, pres_raw, hum_raw, calib)
    ax, ay, az, gx, gy, gz = read_mpu6050()
    temp_obj = read_mlx90614_temp()

    # Magnitud aceleración y filtro
    acc_mag = (ax**2 + ay**2 + az**2) ** 0.5
    acc_window.append(acc_mag)
    acc_filtered = sum(acc_window) / len(acc_window)

    # Detección paso con antirrebote + giro
    current_time = time.time()
    dynamic_threshold = 1.1  # Ajustable
    if acc_filtered > dynamic_threshold and (abs(gx) + abs(gy)) > 15:
        if current_time - last_step_time > 0.3:  # 300 ms mínimo
            step_count += 1
            last_step_time = current_time

    # Mostrar y publicar
    print(f"Presión: {pressure:.2f} hPa | Humedad: {humidity:.2f}% | Temp.Obj: {temp_obj:.2f}°C | Pasos: {step_count}")
    payload = {"presion": round(pressure, 2), "humedad": round(humidity, 2), "temp_objeto": round(temp_obj, 2), "pasos": step_count}
    publicar_mqtt(payload)
    time.sleep(0.2)
