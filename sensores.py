import time
import smbus2
import json
import paho.mqtt.client as mqtt
import math
import sqlite3
import socket
from collections import deque

# MQTT
MQTT_BROKER = "52.203.81.35"
MQTT_PORT = 1883
MQTT_TOPIC = "sensores/datos"

# DB local
DB_FILE = "datos_sensores.db"

# === SQLite ===
def inicializar_db():
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    c.execute('''
        CREATE TABLE IF NOT EXISTS datos (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            payload TEXT
        )
    ''')
    conn.commit()
    conn.close()

def guardar_localmente(payload):
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    c.execute("INSERT INTO datos (payload) VALUES (?)", (json.dumps(payload),))
    conn.commit()
    conn.close()
    print(" Guardado localmente")

def publicar_datos_pendientes():
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    c.execute("SELECT * FROM datos ORDER BY id ASC LIMIT 1")
    fila = c.fetchone()
    if fila:
        id_dato, payload = fila
        try:
            publicar_mqtt(json.loads(payload))
            c.execute("DELETE FROM datos WHERE id = ?", (id_dato,))
            conn.commit()
        except Exception as e:
            print(" Error al publicar pendiente:", e)
    conn.close()

# === WiFi ===
def hay_conexion():
    try:
        socket.create_connection(("8.8.8.8", 53), timeout=1)
        return True
    except OSError:
        return False

# === MQTT ===
def publicar_mqtt(payload):
    client = mqtt.Client()
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.publish(MQTT_TOPIC, json.dumps(payload))
    client.disconnect()
    print(" Publicado a MQTT:", payload)

# === Sensores ===
smbus = smbus2.SMBus(1)
BME280_ADDR = 0x76
MPU6050_ADDR = 0x68
MLX90614_ADDR = 0x5A

smbus.write_byte_data(MPU6050_ADDR, 0x6B, 0)

def write_bme280(addr, reg, data):
    smbus.write_byte_data(addr, reg, data)

write_bme280(BME280_ADDR, 0xF2, 0x01)
write_bme280(BME280_ADDR, 0xF4, 0x27)
write_bme280(BME280_ADDR, 0xF5, 0xA0)
time.sleep(0.5)

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
    temperature = ((t_fine * 5 + 128) >> 8) / 100.0

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

    h = t_fine - 76800
    h = (((((hum_raw << 14) - (calib['H4'] << 20) - (calib['H5'] * h)) + 16384) >> 15) *
         (((((((h * calib['H6']) >> 10) * (((h * calib['H3']) >> 11) + 32768)) >> 10) + 2097152) *
           calib['H2'] + 8192) >> 14)) >> 10
    h = h - (((((h >> 15) * (h >> 15)) >> 7) * calib['H1']) >> 4)
    h = max(0, min(h, 419430400)) >> 12
    humidity = h / 1024.0

    return temperature, pressure, humidity

def read_acceleration():
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
    return ax, ay, az

def read_mlx90614_temp():
    raw = smbus.read_word_data(MLX90614_ADDR, 0x07)
    return (raw * 0.02) - 273.15

# === Inicialización final ===
window = deque(maxlen=10)
step_count = 0
last_step_time = 0
THRESHOLD = 1.15
MIN_STEP_INTERVAL = 0.4
step_detected = False

inicializar_db()
calib = read_calibration()
print(" Sistema iniciado con respaldo local")

# === Bucle principal ===
while True:
    temp_raw, pres_raw, hum_raw = read_bme280_raw()
    temperature, pressure, humidity = compensate(temp_raw, pres_raw, hum_raw, calib)
    ax, ay, az = read_acceleration()
    temp_obj = read_mlx90614_temp()

    acc_mag = math.sqrt(ax**2 + ay**2 + az**2)
    window.append(acc_mag)
    avg_mag = sum(window) / len(window)

    current_time = time.time()

    # Detección de pasos mejorada
    if not step_detected and avg_mag > THRESHOLD:
        step_detected = True
    elif step_detected and avg_mag < THRESHOLD:
        if (current_time - last_step_time) > MIN_STEP_INTERVAL:
            step_count += 1
            last_step_time = current_time
            print(f"👣 Paso detectado: {step_count}")
        step_detected = False

    payload = {
        "bme280": {"temperatura": round(temperature, 2), "presion": round(pressure, 2), "humedad": round(humidity, 2)},
        "mlx90614": {"temp_objeto": round(temp_obj, 2)},
        "mpu6050": {"pasos": step_count}
    }

    if hay_conexion():
        try:
            publicar_mqtt(payload)
            publicar_datos_pendientes()
        except:
            guardar_localmente(payload)
    else:
        guardar_localmente(payload)

    time.sleep(3)
