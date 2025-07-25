import serial
import sqlite3
import time
import requests
import paho.mqtt.client as mqtt
from datetime import datetime

# Configuración Serial
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# Configuración SQLite
conn = sqlite3.connect('sensores.db')
cursor = conn.cursor()
cursor.execute('''
CREATE TABLE IF NOT EXISTS gsr_data (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    value INTEGER,
    timestamp TEXT
)
''')

# Configuración MQTT
MQTT_BROKER = "192.168.x.x"  # Cambia por tu broker
MQTT_PORT = 1883
MQTT_TOPIC = "sensor/gsr"

client = mqtt.Client()

def check_internet():
    try:
        requests.get("http://www.google.com", timeout=3)
        return True
    except requests.ConnectionError:
        return False

def connect_mqtt():
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        print("[MQTT] Conectado al broker")
        return True
    except:
        print("[MQTT] No se pudo conectar")
        return False

def enviar_datos_pendientes():
    cursor.execute("SELECT id, value, timestamp FROM gsr_data")
    filas = cursor.fetchall()
    if not filas:
        return
    print(f"[MQTT] Enviando {len(filas)} datos pendientes...")
    for fila in filas:
        id_reg, valor, ts = fila
        payload = f'{{"gsr":{valor}, "timestamp":"{ts}"}}'
        try:
            client.publish(MQTT_TOPIC, payload)
            cursor.execute("DELETE FROM gsr_data WHERE id=?", (id_reg,))
            conn.commit()
            print(f"[OK] Enviado y eliminado ID {id_reg}")
        except Exception as e:
            print("[ERROR] No se pudo enviar:", e)
            break

# Main loop
print("Escuchando datos del ESP32...")

while True:
    try:
        # Leer datos del ESP32
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            if line.startswith("GSR:"):
                gsr_value = int(line.split(":")[1])
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                if check_internet() and connect_mqtt():
                    # Enviar a MQTT directamente
                    payload = f'{{"gsr":{gsr_value}, "timestamp":"{timestamp}"}}'
                    client.publish(MQTT_TOPIC, payload)
                    print(f"[MQTT] Enviado: {payload}")

                    # Además intenta vaciar la base de datos
                    enviar_datos_pendientes()
                else:
                    # Guardar en SQLite si no hay internet
                    cursor.execute("INSERT INTO gsr_data (value, timestamp) VALUES (?, ?)", (gsr_value, timestamp))
                    conn.commit()
                    print(f"[LOCAL] Guardado en SQLite: {gsr_value} ({timestamp})")

        time.sleep(1)

    except Exception as e:
        print("Error en bucle:", e)
        time.sleep(2)
