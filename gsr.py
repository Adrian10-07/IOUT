import serial
import json
import sqlite3
import time
import paho.mqtt.client as mqtt
import socket

# Configuraciones
SERIAL_PORT = '/dev/ttyUSB0'   # Ajusta según corresponda
BAUD_RATE = 115200
MQTT_BROKER = "52.203.81.35"
MQTT_PORT = 1883
MQTT_TOPIC = "GSR-SENSOR"
DB_PATH = "gsr_data.db"

# Conexión MQTT
client = mqtt.Client()

# Base de datos
def init_db():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE IF NOT EXISTS gsr_data (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        porcentaje REAL,
                        timestamp TEXT
                      )''')
    conn.commit()
    conn.close()

def guardar_dato(porcentaje):
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("INSERT INTO gsr_data (porcentaje, timestamp) VALUES (?, datetime('now'))", (porcentaje,))
    conn.commit()
    conn.close()

def enviar_datos_pendientes():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("SELECT id, porcentaje FROM gsr_data ORDER BY id ASC")
    rows = cursor.fetchall()

    for row in rows:
        id_, porcentaje = row
        payload = json.dumps({"porcentaje": porcentaje})
        try:
            client.publish(MQTT_TOPIC, payload)
            print(" Reenviado desde DB:", payload)
            cursor.execute("DELETE FROM gsr_data WHERE id = ?", (id_,))
            conn.commit()
            time.sleep(1)  # Pausa para evitar saturar
        except:
            print(" Error reenviando dato pendiente")
            break
    conn.close()

def wifi_disponible():
    try:
        socket.create_connection(("8.8.8.8", 53), timeout=2)
        return True
    except:
        return False

def main():
    init_db()
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print(" Esperando datos desde ESP32...")
    except serial.SerialException as e:
        print(" Error al abrir puerto serial:", e)
        return

    while True:
        try:
            line = ser.readline().decode().strip()
            if line:
                print(" Recibido:", line)
                try:
                    data = json.loads(line)
                    porcentaje = float(data["porcentaje"])

                    if wifi_disponible():
                        try:
                            if not client.is_connected():
                                client.connect(MQTT_BROKER, MQTT_PORT, 60)
                            client.publish(MQTT_TOPIC, json.dumps({"porcentaje": porcentaje}))
                            print(" Publicado MQTT:", porcentaje)
                            enviar_datos_pendientes()
                        except Exception as e:
                            print(" Error publicando:", e)
                            guardar_dato(porcentaje)
                    else:
                        print(" Sin WiFi. Guardando en DB.")
                        guardar_dato(porcentaje)
                except json.JSONDecodeError:
                    print(" JSON inválido:", line)
        except KeyboardInterrupt:
            print("\n Finalizado por el usuario.")
            break
        except Exception as e:
            print(" Error:", e)

if __name__ == "__main__":
    main()
