# 📡 LoRa Receiver Gateway - Raspberry Pi

[![Python](https://img.shields.io/badge/Python-3.8+-green.svg)](https://www.python.org/)
[![LoRa](https://img.shields.io/badge/LoRa-SX1262-blue.svg)](https://www.semtech.com/products/wireless-rf/lora-transceivers/sx1262)
[![MQTT](https://img.shields.io/badge/MQTT-Enabled-orange.svg)](https://mqtt.org/)
[![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-Compatible-red.svg)](https://www.raspberrypi.org/)

**Gateway receptor LoRa para Raspberry Pi** que recibe paquetes binarios de una red mesh LoRa (ESP32) y los publica en un broker MQTT para su procesamiento posterior.

---

## 🎯 Descripción del Proyecto

Este proyecto implementa un **receptor LoRa gateway** en una Raspberry Pi que actúa como punto de entrada de una red mesh LoRa. El sistema:

- 📻 **Recibe paquetes LoRa** desde nodos ESP32 en la banda 868 MHz (EU868)
- 🔍 **Parsea payloads binarios** de 13 bytes con datos GPS, secuencia y estado
- 📊 **Mide RSSI** (fuerza de señal) de cada paquete recibido
- 📤 **Publica datos en MQTT** para integración con sistemas de almacenamiento y visualización

### Arquitectura del Sistema

```
┌─────────────┐      LoRa       ┌──────────────────┐      MQTT      ┌─────────────┐
│   ESP32     │ ───────────────▶│  Raspberry Pi    │ ──────────────▶│   Broker    │
│  (Nodos)    │  868 MHz        │  (Este Proyecto)  │   JSON        │   MQTT      │
│  Mesh LoRa  │                 │  SX1262 Receiver  │                │             │
└─────────────┘                 └──────────────────┘                └─────────────┘
                                                                           │
                                                                           ▼
                                                                   ┌──────────────┐
                                                                   │  InfluxDB    │
                                                                   │  + Grafana   │
                                                                   └──────────────┘
```

---

## ✨ Características Principales

- 🔌 **Control completo del SX1262** mediante SPI y GPIO
- 📡 **Recepción continua** en modo RX continuo con interrupciones
- 🎯 **Parsing inteligente** de payloads binarios con detección automática de headers
- 📈 **Medición de RSSI** en tiempo real para cada paquete
- 🔄 **Integración MQTT** con reconexión automática
- 🛡️ **Manejo robusto de errores** y validación de datos
- ⚡ **Bajo consumo** con polling eficiente de interrupciones

---

## 📋 Requisitos Previos

### Hardware

- **Raspberry Pi** (modelo 3B+ o superior recomendado)
- **Waveshare SX1262 LoRaWAN/GNSS HAT** o compatible
- **Antena LoRa** (868 MHz para EU868)

### Software

- **Raspberry Pi OS** (Raspbian/Debian)
- **Python 3.8+**
- **pip** (gestor de paquetes Python)
- **SPI habilitado** en la Raspberry Pi
- **Broker MQTT** (Mosquitto u otro) accesible en la red

### Verificar Hardware

```bash
# Verificar que SPI está habilitado
lsmod | grep spi

# Verificar dispositivos SPI disponibles
ls -l /dev/spi*

# Si no aparece, habilitar SPI:
sudo raspi-config
# → Interface Options → SPI → Enable
```

---

## 🚀 Instalación

### 1️⃣ Clonar el Repositorio

```bash
cd ~/Documents/TFG_Teleco/code
git clone <tu-repositorio> Receiver-Lora_RaspberryPi
cd Receiver-Lora_RaspberryPi
```

### 2️⃣ Instalar Dependencias

```bash
# Crear entorno virtual (recomendado)
python3 -m venv venv
source venv/bin/activate

# Instalar dependencias
pip install -r requirements.txt
```

Si no existe `requirements.txt`, instala manualmente:

```bash
pip install paho-mqtt python-dotenv gpiozero spidev
```

### 3️⃣ Configurar Variables de Entorno

Crea un archivo `.env` en la raíz del proyecto:

```bash
# Configuración MQTT
MQTT_PORT=1883
MQTT_TOPIC=loramesh/data

# Opcional: Configuración adicional
# MQTT_BROKER=localhost  # Por defecto es localhost
```

> ⚠️ **Nota**: El archivo `.env` está en `.gitignore` por seguridad.

### 4️⃣ Verificar Conexiones GPIO

El proyecto utiliza los siguientes pines GPIO de la Raspberry Pi:

| Pin GPIO | Función | Descripción |
|----------|---------|-------------|
| GPIO18   | RESET   | Reset del módulo SX1262 |
| GPIO20   | BUSY    | Indicador de estado ocupado |
| GPIO16   | DIO1    | Interrupción de RX/TX |
| GPIO21   | CS      | Chip Select (SPI) |
| GPIO10   | MOSI    | SPI Master Out Slave In |
| GPIO9    | MISO    | SPI Master In Slave Out |
| GPIO11   | SCK     | SPI Serial Clock |

**Nota**: Los pines SPI (MOSI, MISO, SCK) son fijos en Raspberry Pi y no se configuran en el código.

---

## 🔧 Configuración LoRa

El receptor está configurado para trabajar con nodos ESP32 que usan **RadioLib**. Los parámetros deben coincidir exactamente:

### Parámetros de Radio (EU868)

```python
Frecuencia:     868.0 MHz
Spreading Factor: 12 (SF12)
Bandwidth:      125 kHz
Coding Rate:    5 (4/5)
Sync Word:      0x12
CRC:            Habilitado
Preamble:       8 bytes
```

### Formato de Paquete Binario

Los nodos ESP32 envían paquetes binarios de **13 bytes** con la siguiente estructura:

```
Byte 0:   src      (uint8)    - ID del nodo emisor
Bytes 1-2: seq     (uint16 BE) - Número de secuencia
Byte 3:   ttl      (uint8)    - Time To Live (saltos restantes)
Bytes 4-7: lat     (float32 LE) - Latitud GPS
Bytes 8-11: lon    (float32 LE) - Longitud GPS
Byte 12:  state    (uint8)    - Estado del nodo (0=OK, 1=SOS)
```

**Endianness**:
- `seq`: **Big-Endian** (MSB primero)
- `lat`, `lon`: **Little-Endian** (LSB primero)

---

## 🎮 Uso

### Ejecución Básica

```bash
# Activar entorno virtual (si usas uno)
source venv/bin/activate

# Ejecutar el receptor
python3 receiver_lora.py
```

### Salida Esperada

```
[+] Configurando SX1262 en modo recepción...
[+] Entrando en RX continuo…
src=1, seq=42, ttl=3, lat=40.416800, lon=-3.703800, state=0
src=2, seq=15, ttl=2, lat=40.416900, lon=-3.703900, state=0
...
```

### Verificar Datos en MQTT

En otra terminal, suscríbete al topic MQTT:

```bash
# Si usas Mosquitto
mosquitto_sub -h localhost -p 1883 -t "loramesh/data" -v

# Deberías ver mensajes JSON como:
# loramesh/data {"src": 1, "seq": 42, "ttl": 3, "lat": 40.4168, "lon": -3.7038, "state": 0, "rssi": -85.5}
```

### Detener el Receptor

Presiona `Ctrl+C` para detener el receptor de forma segura. El programa:
- Cierra la conexión SPI
- Libera los pines GPIO
- Desconecta el cliente MQTT

---

## 📊 Formato de Datos MQTT

Cada paquete recibido se publica en MQTT como un objeto JSON:

```json
{
  "src": 1,              // ID del nodo emisor (uint8)
  "seq": 42,             // Número de secuencia (uint16)
  "ttl": 3,              // Time To Live (uint8)
  "lat": 40.4168,        // Latitud GPS (float32)
  "lon": -3.7038,        // Longitud GPS (float32)
  "state": 0,            // Estado: 0=OK, 1=SOS (uint8)
  "rssi": -85.5          // Fuerza de señal en dBm (float, puede ser null)
}
```

---

## 🏗️ Estructura del Código

```
Receiver-Lora_RaspberryPi/
├── receiver_lora.py      # Script principal del receptor
├── sx1262.py            # Driver/clase para controlar el SX1262
├── .env                 # Variables de entorno (no en Git)
├── .gitignore           # Archivos ignorados por Git
└── README.md            # Este archivo
```

### `receiver_lora.py`

Script principal que:
- Inicializa el módulo SX1262
- Configura parámetros de radio
- Entra en modo recepción continua
- Procesa interrupciones DIO1
- Parsea payloads binarios
- Publica datos en MQTT

**Funciones principales**:
- `limpiar_mensaje_corto()`: Parsea payloads de 13 bytes
- `limpiar_mensaje()`: Parsea payloads de 18 bytes (formato alternativo)
- `main()`: Bucle principal de recepción

### `sx1262.py`

Clase `SX1262` que encapsula:
- **Comunicación SPI** de bajo nivel
- **Control GPIO** (RESET, BUSY, DIO1, CS)
- **Comandos LoRa** de alto nivel (frecuencia, modulación, paquetes)
- **Lectura de registros** (RSSI, estado del buffer)

**Métodos clave**:
- `set_rf_frequency()`: Configura frecuencia de radio
- `set_modulation_params()`: Configura SF, BW, CR
- `set_rx_continuous()`: Entra en modo RX continuo
- `get_packet_rssi()`: Lee RSSI del último paquete
- `read_buffer()`: Lee datos del buffer de recepción

---

## 🔍 Detalles Técnicos

### Detección Automática de Headers

El parser detecta automáticamente si el payload tiene bytes adicionales al inicio (headers del protocolo LoRa):

```python
# Si el payload tiene más de 13 bytes, busca el inicio real de los datos
# Busca un patrón donde:
# - src < 100 (valor razonable para ID de nodo)
# - seq < 10000 (valor razonable para secuencia)
```

### Medición de RSSI

El RSSI se lee inmediatamente después de recibir un IRQ, antes de leer el buffer:

```python
# Método 1: Usando GET_PACKET_STATUS
rssi = radio.get_packet_rssi()

# Método 2 (fallback): Leyendo registro directamente
rssi = radio.get_packet_rssi_from_register()
```

**Fórmula de conversión**: `RSSI (dBm) = -RSSI_PACKET / 2.0`

### Modo RX Continuo

El receptor permanece en modo RX continuo, escuchando constantemente:

```python
radio.set_rx_continuous()  # No necesita re-entrar después de cada paquete
```

Las interrupciones se detectan mediante polling del pin DIO1:

```python
if radio.dio1_pin.value:  # Hay IRQ
    irq = radio.get_irq()
    # Procesar paquete...
    radio.clear_irq(0xFFFF)  # Limpiar para próximo paquete
```

---

## 🐛 Troubleshooting

### No se reciben paquetes

1. **Verificar conexiones GPIO**:
   ```bash
   # Verificar que los pines están correctamente conectados
   # Revisar soldaduras y conexiones del HAT
   ```

2. **Verificar configuración de radio**:
   - Asegúrate de que los parámetros coinciden con los nodos ESP32
   - Verifica la frecuencia (868 MHz para EU868)
   - Comprueba el Sync Word (0x12)

3. **Verificar antena**:
   - Asegúrate de que la antena está conectada
   - Verifica que es una antena para 868 MHz

4. **Ver logs**:
   ```bash
   python3 receiver_lora.py
   # Busca mensajes de error o advertencias
   ```

### RSSI siempre es None o incorrecto

1. **Verificar timing**: El RSSI debe leerse inmediatamente después del IRQ
2. **Probar método alternativo**: El código ya incluye un fallback
3. **Verificar registros**: Usa `read_register(0x0890, 1)` para leer RSSI directamente

### Errores de parsing

1. **Verificar formato del payload**: Debe ser exactamente 13 bytes (o más con headers)
2. **Verificar endianness**: `seq` es big-endian, `lat`/`lon` son little-endian
3. **Revisar logs**: Los errores de parsing se muestran en la consola

### Problemas con MQTT

1. **Verificar conexión al broker**:
   ```bash
   # Probar conexión manual
   mosquitto_pub -h localhost -p 1883 -t "test" -m "hello"
   mosquitto_sub -h localhost -p 1883 -t "test" -v
   ```

2. **Verificar variables de entorno**:
   ```bash
   # Verificar que .env existe y tiene las variables correctas
   cat .env
   ```

3. **Verificar firewall**:
   ```bash
   # Si el broker está en otra máquina, verificar que el puerto está abierto
   telnet <IP_BROKER> 1883
   ```

### Error: "Permission denied" en SPI

```bash
# Añadir usuario al grupo spi
sudo usermod -a -G spi,gpio $USER

# Reiniciar sesión o ejecutar:
newgrp spi
```

### El módulo no responde

1. **Verificar reset**: El módulo se resetea al inicializar
2. **Verificar alimentación**: Asegúrate de que el HAT recibe suficiente corriente
3. **Verificar BUSY pin**: Debe estar en bajo cuando el módulo está listo

---

## 🔗 Integración con el Sistema Completo

Este receptor forma parte de un sistema más grande:

1. **Nodos ESP32** (`Lora-Mesh/`): Envían paquetes LoRa
2. **Raspberry Pi** (este proyecto): Recibe y publica en MQTT
3. **MacBook** (`MQTT-Raspberry/`): Procesa MQTT y almacena en InfluxDB
4. **Grafana**: Visualiza datos en tiempo real

### Flujo Completo

```
ESP32 → LoRa (868 MHz) → Raspberry Pi → MQTT → MacBook → InfluxDB → Grafana
```

Para más información sobre el sistema completo, consulta:
- `../MQTT-Raspberry/README.md`: Sistema de almacenamiento y visualización
- `../Lora-Mesh/README.md`: Nodos emisores ESP32

---

## 📚 Referencias y Documentación

### Hardware

- [SX1262 Datasheet](https://www.semtech.com/products/wireless-rf/lora-transceivers/sx1262)
- [Waveshare SX1262 HAT](https://www.waveshare.com/wiki/SX1262_LoRaWAN_GPS_HAT)
- [Raspberry Pi GPIO](https://www.raspberrypi.org/documentation/usage/gpio/)

### Software

- [RadioLib Documentation](https://github.com/jgromes/RadioLib) (usado en ESP32)
- [paho-mqtt Python](https://www.eclipse.org/paho/clients/python/)
- [spidev Documentation](https://github.com/doceme/py-spidev)

### Protocolos

- [LoRaWAN Specification](https://lora-alliance.org/lorawan-for-developers/)
- [MQTT Protocol](https://mqtt.org/)

---

## 🔐 Seguridad

### Buenas Prácticas

1. **No subas `.env` a Git**: Ya está en `.gitignore`
2. **Usa autenticación MQTT**: Si tu broker lo soporta, configura usuario/contraseña
3. **Restringe acceso a MQTT**: Usa firewall para limitar acceso al puerto 1883
4. **Valida datos recibidos**: El código ya incluye validaciones básicas

### Configurar Autenticación MQTT

Si tu broker MQTT requiere autenticación, modifica `receiver_lora.py`:

```python
# En la función main(), después de crear el cliente:
client.username_pw_set("usuario", "contraseña")
client.connect(MQTT_BROKER, MQTT_PORT, 60)
```

---

## 🧪 Testing

### Test Manual

1. **Iniciar receptor**:
   ```bash
   python3 receiver_lora.py
   ```

2. **Enviar paquete de prueba desde ESP32**:
   - Asegúrate de que el ESP32 está configurado con los mismos parámetros
   - El receptor debería mostrar el paquete recibido

3. **Verificar MQTT**:
   ```bash
   mosquitto_sub -h localhost -p 1883 -t "loramesh/data" -v
   ```

### Test de RSSI

El código incluye validación de RSSI:
- Verifica que el RSSI está en rango razonable (-120 a -30 dBm)
- Compara RSSI entre paquetes para detectar problemas

---

## 📝 Notas de Desarrollo

### Limitaciones Conocidas

- ⚠️ **No se puede ejecutar con virtual environment** (según comentario en código)
- 🔄 **Polling de DIO1**: Usa polling en lugar de interrupciones GPIO reales
- 📦 **Formato fijo**: Solo soporta payloads de 13 bytes (formato corto)

### Mejoras Futuras

- [ ] Implementar interrupciones GPIO reales (más eficiente)
- [ ] Soporte para múltiples formatos de payload
- [ ] Configuración mediante archivo YAML/JSON
- [ ] Logging estructurado (JSON logs)
- [ ] Métricas de rendimiento (paquetes/segundo, tasa de error)
- [ ] Soporte para Docker (containerización)

---

## 🤝 Contribuciones

Este proyecto forma parte de un **Trabajo de Fin de Grado (TFG)** en Telecomunicaciones.

---

## 📄 Licencia

Este proyecto es parte de un trabajo académico. Consulta con el autor para más detalles.

---

## 👤 Autor

**Oscar Jiménez Bou**  
Trabajo de Fin de Grado - Telecomunicaciones

---

## 🎓 Agradecimientos

Proyecto desarrollado como parte del TFG en Telecomunicaciones, integrando tecnologías IoT, redes mesh LoRa, MQTT y sistemas embebidos.

---

## ❓ Preguntas Frecuentes

### ¿Puedo usar otro módulo LoRa además del SX1262?

El código está específicamente diseñado para el SX1262. Para usar otro módulo (SX1276, SX1278, etc.), necesitarías adaptar la clase `SX1262` o usar una librería diferente.

### ¿Qué rango de alcance tiene?

Depende de varios factores:
- **Spreading Factor**: SF12 ofrece mayor alcance pero menor tasa de datos
- **Potencia de transmisión**: Los nodos ESP32 usan 14 dBm (~25 mW)
- **Obstáculos**: Edificios, árboles reducen el alcance
- **Altura de antenas**: Mayor altura = mayor alcance

En condiciones ideales (campo abierto, sin obstáculos), el alcance puede ser de varios kilómetros.

### ¿Puedo recibir de múltiples nodos simultáneamente?

Sí, el receptor puede recibir paquetes de múltiples nodos. Cada paquete incluye el campo `src` que identifica el nodo emisor.

### ¿Cómo sincronizo el receptor con los nodos?

No es necesario sincronizar. El receptor está en modo RX continuo y escucha constantemente. Los nodos envían cuando tienen datos.

---

**¿Preguntas o problemas?** Revisa la sección de [Troubleshooting](#-troubleshooting) o consulta los logs del receptor.

