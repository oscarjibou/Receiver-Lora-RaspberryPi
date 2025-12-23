# test_rx.py

#FIXME: no se puede ejecutar esto con un virtual enviromente. Ver si hacer un docker o algo. 

from sx1262 import SX1262
import os
import time
import re
import json 
import struct
import paho.mqtt.client as mqtt
from dotenv import load_dotenv

load_dotenv() 

MQTT_BROKER = "localhost"
MQTT_PORT = int(os.getenv("MQTT_PORT"))
MQTT_TOPIC = os.getenv("MQTT_TOPIC")

# Crear cliente MQTT
client = mqtt.Client()
client.connect(MQTT_BROKER, MQTT_PORT, 60)

def limpiar_mensaje_corto(payload):
    """
    Parsea el nuevo formato de payload binario de 13 bytes:
    - Byte 0: src (MY_ID) - 1 byte
    - Bytes 1-2: seq (uint16, big-endian) - 2 bytes
    - Byte 3: ttl - 1 byte
    - Bytes 4-7: lat (float32, little-endian) - 4 bytes
    - Bytes 8-11: lon (float32, little-endian) - 4 bytes
    - Byte 12: state (sosMode) - 1 byte
    
    El payload recibido puede tener bytes adicionales al inicio (header del protocolo LoRa).
    """
    try:
        # Validar que el payload sea bytes
        if not isinstance(payload, bytes):
            return str(payload)
        
        # Necesitamos al menos 13 bytes de datos
        if len(payload) < 13:
            return f"[ERROR] Payload demasiado corto: {len(payload)} bytes (mínimo 13)"
        
        # Detectar el inicio de los datos reales
        # Buscar un patrón donde el primer byte es razonable (< 100) y los siguientes forman un seq válido
        data_start = 0
        
        # Si el payload tiene más de 13 bytes, hay bytes adicionales al inicio
        if len(payload) > 13:
            # Buscar el inicio de los datos: src debería ser un valor razonable (< 100)
            # y el seq (big-endian) también debería ser razonable
            for i in range(len(payload) - 12):
                if payload[i] < 100:  # src razonable
                    # Verificar que los siguientes bytes formen un seq razonable (big-endian)
                    seq_candidate = struct.unpack('>H', payload[i+1:i+3])[0]
                    if seq_candidate < 10000:  # seq razonable
                        data_start = i
                        break
        
        payload_data = payload[data_start:]
        
        # Validar que tenemos suficientes bytes
        if len(payload_data) < 13:
            return f"[ERROR] Payload demasiado corto después de saltar header: {len(payload_data)} bytes (mínimo 13)"
        
        # Parsear los campos según el nuevo formato
        # Byte 0: src (uint8)
        src = struct.unpack('<B', payload_data[0:1])[0]
        
        # Bytes 1-2: seq (uint16, big-endian)
        seq = struct.unpack('>H', payload_data[1:3])[0]
        
        # Byte 3: ttl (uint8)
        ttl = struct.unpack('<B', payload_data[3:4])[0]
        
        # Bytes 4-7: lat (float32, little-endian)
        lat = struct.unpack('<f', payload_data[4:8])[0]
        
        # Bytes 8-11: lon (float32, little-endian)
        lon = struct.unpack('<f', payload_data[8:12])[0]
        
        # Byte 12: state (uint8)
        state = struct.unpack('<B', payload_data[12:13])[0]
        
        # Formatear el resultado
        mensaje = f"src={src}, seq={seq}, ttl={ttl}, lat={lat}, lon={lon}, state={state}"
        return mensaje, src, seq, ttl, lat, lon, state
    
    except struct.error as e:
        return f"[ERROR] Error al parsear payload binario: {e}, payload: {payload}"
    except Exception as e:
        return f"[ERROR] Error inesperado: {e}, payload: {payload}"


def limpiar_mensaje(payload):
    """
    Parsea el payload binario que contiene una cabecera estructurada.
    El payload puede tener bytes adicionales al inicio (header del protocolo LoRa).
    Estructura esperada (después de cualquier header):
    - Byte 0: src (uint8)
    - Byte 1: dst (uint8)
    - Bytes 2-3: seq (uint16, big-endian)
    - Byte 4: ttl (uint8)
    - Bytes 5-8: lat (float32, little-endian) [opcional]
    - Bytes 9-12: lon (float32, little-endian) [opcional]
    - Byte 13: state (uint8) [opcional]
    - Bytes 14-17: batt (float32, little-endian) [opcional]
    """
    try:
        # Validar que el payload sea bytes
        if not isinstance(payload, bytes):
            return str(payload)
        
        # Intentar encontrar el inicio de los datos reales
        # El payload puede tener 18 o 20 bytes. Si tiene 20, probablemente hay 2 bytes extra al inicio
        data_start = 0
        
        # Si el payload tiene exactamente 20 bytes, probar saltar 1 o 2 bytes
        if len(payload) == 20:
            # Probar con 1 byte de header primero (más común)
            # Si los valores src y dst son razonables (< 100), usar este offset
            if payload[1] < 100 and payload[2] < 100:
                data_start = 1
            else:
                data_start = 2
        # Si el payload tiene 19 bytes, saltar el primer byte
        elif len(payload) == 19:
            data_start = 1
        # Si tiene 18 bytes, asumimos que no hay header adicional
        elif len(payload) == 18:
            data_start = 0
        # Si tiene más de 20, intentar detectar el patrón
        elif len(payload) > 20:
            # Buscar un patrón que indique el inicio: src y dst deberían ser valores razonables (< 100)
            for i in range(len(payload) - 17):
                if payload[i] < 100 and payload[i+1] < 100:  # src y dst razonables
                    data_start = i
                    break
        
        payload_data = payload[data_start:]
        
        # Necesitamos al menos 5 bytes para el header básico
        if len(payload_data) < 5:
            return f"[ERROR] Payload demasiado corto después de saltar header: {len(payload_data)} bytes (mínimo 5)"
        
        # Parsear el header básico (5 bytes)
        # Bytes 0-1: src, dst (uint8 cada uno)
        src, dst = struct.unpack('<BB', payload_data[0:2])
        
        # Bytes 2-3: seq (uint16, big-endian según el código C)
        seq = struct.unpack('>H', payload_data[2:4])[0]
        
        # Byte 4: ttl (uint8)
        ttl = struct.unpack('<B', payload_data[4:5])[0]
        
        # Si tenemos 18 bytes o más de datos, parsear también lat, lon, state, batt
        if len(payload_data) >= 18:
            lat, lon, state, batt = struct.unpack('<ffBf', payload_data[5:18])
            return f"src={src}, dst={dst}, seq={seq}, ttl={ttl}, lat={lat}, lon={lon}, state={state}, batt={batt:.2f}"
        else:
            # Solo header básico
            return f"src={src}, dst={dst}, seq={seq}, ttl={ttl}"
    
    except struct.error as e:
        return f"[ERROR] Error al parsear payload binario: {e}, payload: {payload}"
    except Exception as e:
        return f"[ERROR] Error inesperado: {e}, payload: {payload}"

def main():

    radio = SX1262(reset=18, busy=20, dio1=16, cs=21)

    try:
        print("[+] Configurando SX1262 en modo recepción...")

        radio.set_standby()
        radio.set_regulator_lodo()
        radio.set_packet_type()
        radio.set_dio2_rf_switch(True)            # conmutador RF del HAT
        radio.set_buffer_base(tx_base=0x00, rx_base=0x00)

        # ‼️ Debe coincide con los valores puestos en ESP32 (RadioLib)
        radio.set_rf_frequency(868_000_000)
        radio.set_modulation_params(sf=12, bw_hz=125_000, cr=5)
        radio.set_syncword(0x12)
        radio.set_packet_params(payload_len=255, crc_on=True, preamble_len=8, explicit_header=True, invert_iq=False)

        # IRQs: enrutar todo a DIO1, limpiar y entrar en RX continuo
        radio.clear_irq(0xFFFF)
        radio.set_dio_irq_params(irq_mask=0xFFFF, dio1_mask=0xFFFF, dio2_mask=0x0000, dio3_mask=0x0000)

        print("[+] Entrando en RX continuo…")
        radio.set_rx_continuous()

        rssi_previo = None
        paquete_num = 0
        
        while True:
            if radio.dio1_pin.value:  # hay IRQ en DIO1
                irq = radio.get_irq()
                
                # Leer RSSI inmediatamente después del IRQ, antes de leer el buffer
                # Esto asegura que el RSSI se lea cuando el chip aún tiene la información del paquete
                try:
                    rssi = radio.get_packet_rssi()
                except:
                    # Si falla, intentar método alternativo usando registro directo
                    try:
                        rssi = radio.get_packet_rssi_from_register()
                    except Exception as e:
                        print(f"[ERROR] No se pudo leer RSSI: {e}")
                        rssi = None
                
                # Leer estado del buffer
                length, offset = radio.get_rx_buffer_status()
                if length > 0:

                    payload = radio.read_buffer(offset, length)
                    mensaje_limpio, src, seq, ttl, lat, lon, state = limpiar_mensaje_corto(payload)
                    paquete_num += 1

                    print(mensaje_limpio)

                    # Debug: verificar que el RSSI varía
                    if rssi is not None:
                        if rssi_previo is not None:
                            diferencia = abs(rssi - rssi_previo)
                            #if diferencia < 0.1:
                             #   print(f"[DEBUG] ⚠️ RSSI muy similar al anterior: {rssi:.2f} vs {rssi_previo:.2f} dBm (diff: {diferencia:.2f})")
                            #else:
                             #  print(f"[DEBUG] ✓ RSSI varía correctamente: {rssi:.2f} vs {rssi_previo:.2f} dBm (diff: {diferencia:.2f})")
                        
                        # Validar que el RSSI está en un rango razonable
                        if rssi < -120 or rssi > -30:
                            #print(f"[DEBUG] ⚠️ RSSI fuera de rango esperado: {rssi:.2f} dBm (normal: -120 a -30 dBm)")
                            print("")
                        
                        rssi_previo = rssi
                    
                    # print(f"[Paquete #{paquete_num}] Mensaje: {mensaje_limpio}, RSSI: {rssi:.2f} dBm" if rssi is not None else f"[Paquete #{paquete_num}] Mensaje: {mensaje_limpio}, RSSI: N/A")

                    # payload_mqtt = {
                    #         "mensaje": mensaje_limpio,
                    #         "rssi": round(rssi, 2) if rssi is not None else None,
                    #         "timestamp": int(time.time())
                    #         }
                    payload_mqtt = {
                            "src": src,
                            "seq": seq,
                            "ttl": ttl,
                            "lat": lat,
                            "lon": lon,
                            "state": state,
                            "rssi": rssi
                            }
                    payload_mqtt_json = json.dumps(payload_mqtt)
                    # print("Enviando:", payload_mqtt_json)

                    client.publish(MQTT_TOPIC, payload_mqtt_json)

                else:
                    print(f"[IRQ] DIO1=1, irq=0x{irq:04X}, pero length=0")

                radio.clear_irq(0xFFFF)        # listo para el próximo paquete
                # re-entrar en RX si fuera necesario (normalmente no hace falta en modo continuo)
                # radio.set_rx_continuous()

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[+] Saliendo...")
    finally:
        radio.close()
        client.disconnect()
        print("[+] Radio cerrada y GPIO liberados")

if __name__ == "__main__":
    main()
