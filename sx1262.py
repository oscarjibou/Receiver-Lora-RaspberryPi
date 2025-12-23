# sx1262.py
import spidev
import time
from gpiozero import DigitalOutputDevice, DigitalInputDevice

class SX1262:
    """
    Waveshare SX1262 LoRaWAN/GNSS HAT (SPI)
      MOSI: GPIO10 (SPI0 MOSI)
      MISO: GPIO9  (SPI0 MISO)
      SCK : GPIO11 (SPI0 SCK)
      CS  : GPIO21 (software CS)
      RESET: GPIO18
      BUSY : GPIO20
      DIO1 : GPIO16
    """

    # --- Opcodes ---
    _SET_STANDBY             = 0x80
    _SET_PACKET_TYPE         = 0x8A
    _SET_RF_FREQUENCY        = 0x86
    _SET_MODULATION_PARAMS   = 0x8B
    _SET_PACKET_PARAMS       = 0x8C
    _SET_TX_PARAMS           = 0x8E
    _SET_BUFFER_BASE_ADDR    = 0x8F
    _SET_RX                  = 0x82
    _GET_STATUS              = 0xC0
    _WRITE_REGISTER          = 0x0D
    _READ_REGISTER           = 0x1D
    _WRITE_BUFFER            = 0x0E
    _READ_BUFFER             = 0x1E
    _CLR_IRQ_STATUS          = 0x02
    _GET_IRQ_STATUS          = 0x12
    _SET_DIO_IRQ_PARAMS      = 0x08
    _GET_RX_BUFFER_STATUS    = 0x13
    _GET_PACKET_STATUS       = 0x14  # Obtener estado del último paquete recibido
    _SET_DIO2_RF_SWITCH      = 0x9D
    _SET_REGULATOR_MODE      = 0x96  # 0=LDO, 1=DC-DC

    _REG_SYNCWORD = 0x0740  # MSB at 0x0740, LSB at 0x0741
    _REG_RSSI_PACKET = 0x0890  # RSSI del último paquete recibido
    
    reset = 18
    busy = 20
    dio1 = 16
    cs   = 21

    def __init__(self, reset=reset, busy=busy, dio1=dio1, cs=cs, spi_bus=0, spi_dev=0, spi_hz=2_000_000):
        # GPIO
        self.reset_pin = DigitalOutputDevice(reset, active_high=True, initial_value=True)
        self.busy_pin  = DigitalInputDevice(busy)
        self.dio1_pin  = DigitalInputDevice(dio1)
        self.cs_pin    = DigitalOutputDevice(cs, active_high=True, initial_value=True)

        # SPI (CE0 se mueve pero no está cableado al HAT; usamos CS por GPIO21)
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_dev)
        self.spi.max_speed_hz = spi_hz
        self.spi.mode = 0

        self.reset_module()

    def close(self):
        try:
            self.spi.close()
        finally:
            for pin in (self.reset_pin, self.busy_pin, self.dio1_pin, self.cs_pin):
                try: pin.close()
                except: pass

    # --- Low level ---
    def reset_module(self):
        self.reset_pin.off()
        time.sleep(0.01)
        self.reset_pin.on()
        time.sleep(0.01)

    def _wait_busy(self):
        while self.busy_pin.value:  # activo en alto
            time.sleep(0.0005)

    def _wr(self, opcode, data=None):
        self._wait_busy()
        self.cs_pin.off()
        try:
            frame = [opcode]
            if data: frame += list(data)
            self.spi.xfer2(frame)
        finally:
            self.cs_pin.on()

    def _rd(self, opcode, n, pre=None):
        self._wait_busy()
        self.cs_pin.off()
        try:
            tx = [opcode]
            if pre: tx += pre
            tx += [0x00]*n
            rx = self.spi.xfer2(tx)
            return rx
        finally:
            self.cs_pin.on()

    def write_register(self, addr, values):
        hi = (addr >> 8) & 0xFF
        lo = addr & 0xFF
        self._wr(self._WRITE_REGISTER, [hi, lo] + list(values))

    def read_register(self, addr, n=1):
        hi = (addr >> 8) & 0xFF
        lo = addr & 0xFF
        rx = self._rd(self._READ_REGISTER, n+1, pre=[hi, lo, 0x00])
        # rx[0]=opcode echo; rx[1]=status; rx[2:] data
        return rx[2:]

    def write_buffer(self, data: bytes, offset=0x00):
        self._wr(self._WRITE_BUFFER, [offset] + list(data))

    def read_buffer(self, offset, n):
        rx = self._rd(self._READ_BUFFER, n+1, pre=[offset, 0x00])
        # rx[0]=opcode echo; rx[1]=status; rx[2:] data
        return bytes(rx[2:])

    # --- High-level helpers ---
    def set_standby(self):           self._wr(self._SET_STANDBY, [0x01])  # RC
    def set_packet_type(self):       self._wr(self._SET_PACKET_TYPE, [0x01])  # LoRa
    def set_regulator_lodo(self):    self._wr(self._SET_REGULATOR_MODE, [0x00])  # LDO
    def set_dio2_rf_switch(self, on=True): self._wr(self._SET_DIO2_RF_SWITCH, [0x01 if on else 0x00])

    def set_rf_frequency(self, freq_hz=868_000_000):
        frf = int((freq_hz * (1<<25)) / 32_000_000)
        b = [(frf>>24)&0xFF,(frf>>16)&0xFF,(frf>>8)&0xFF,frf&0xFF]
        self._wr(self._SET_RF_FREQUENCY, b)

    def set_modulation_params(self, sf=12, bw_hz=125_000, cr=5):
        bw_map = {125_000:0x04, 250_000:0x05, 500_000:0x06}
        bw_code = bw_map.get(bw_hz, 0x04)
        cr_code = max(1, min(4, cr-4))  # 1..4 -> 4/5..4/8
        ldr = 0x01 if (sf>=11 and bw_hz==125_000) else 0x00
        self._wr(self._SET_MODULATION_PARAMS, [sf, bw_code, cr_code, ldr])

    def set_packet_params(self, payload_len=32, crc_on=True, preamble_len=8, explicit_header=True, invert_iq=False):
        pre_hi = (preamble_len >> 8) & 0xFF
        pre_lo = preamble_len & 0xFF
        header = 0x00 if explicit_header else 0x01
        crc = 0x01 if crc_on else 0x00
        iq  = 0x01 if invert_iq else 0x00
        self._wr(self._SET_PACKET_PARAMS, [pre_hi, pre_lo, header, payload_len & 0xFF, crc, iq])

    def set_tx_params(self, power_dbm=14): self._wr(self._SET_TX_PARAMS, [power_dbm & 0xFF, 0x04])
    def set_syncword(self, value=0x12):     self.write_register(self._REG_SYNCWORD, [value & 0xFF, value & 0xFF])
    def set_buffer_base(self, tx_base=0x00, rx_base=0x00): self._wr(self._SET_BUFFER_BASE_ADDR, [tx_base & 0xFF, rx_base & 0xFF])
    def clear_irq(self, mask=0xFFFF):       self._wr(self._CLR_IRQ_STATUS, [(mask>>8)&0xFF, mask&0xFF])
    def get_irq(self):
        rx = self._rd(self._GET_IRQ_STATUS, 2, pre=[0x00])
        # rx[0]=opcode echo; rx[1]=status; rx[2]=irq_hi; rx[3]=irq_lo
        return ((rx[2]&0xFF)<<8) | (rx[3]&0xFF)

    def set_dio_irq_params(self, irq_mask=0xFFFF, dio1_mask=0xFFFF, dio2_mask=0x0000, dio3_mask=0x0000):
        b = [(irq_mask>>8)&0xFF, irq_mask&0xFF,
             (dio1_mask>>8)&0xFF, dio1_mask&0xFF,
             (dio2_mask>>8)&0xFF, dio2_mask&0xFF,
             (dio3_mask>>8)&0xFF, dio3_mask&0xFF]
        self._wr(self._SET_DIO_IRQ_PARAMS, b)

    def get_rx_buffer_status(self):
        rx = self._rd(self._GET_RX_BUFFER_STATUS, 2, pre=[0x00])
        # rx[0]=opc; rx[1]=status; rx[2]=payload_len; rx[3]=rx_start_ptr
        return rx[2] & 0xFF, rx[3] & 0xFF

    def set_rx_continuous(self):
        self._wr(self._SET_RX, [0xFF, 0xFF, 0xFF])

    def get_status(self):
        rx = self._rd(self._GET_STATUS, 1, pre=[0x00])
        return rx[1]  # status byte

    def get_packet_status(self):
        """
        Obtiene el estado del último paquete recibido usando GET_PACKET_STATUS.
        Retorna una tupla (rssi_pkt, rssi_sync, snr)
        - rssi_pkt: RSSI del último paquete recibido
        - rssi_sync: RSSI promedio durante la sincronización
        - snr: SNR del último paquete recibido
        """
        rx = self._rd(self._GET_PACKET_STATUS, 3, pre=[0x00])
        # rx[0]=opcode echo; rx[1]=status; rx[2]=rssi_pkt; rx[3]=rssi_sync; rx[4]=snr
        rssi_pkt = rx[2] if len(rx) > 2 else 0
        rssi_sync = rx[3] if len(rx) > 3 else 0
        snr = rx[4] if len(rx) > 4 else 0
        return rssi_pkt, rssi_sync, snr
    
    def get_packet_rssi(self):
        """
        Obtiene el RSSI del último paquete recibido usando GET_PACKET_STATUS.
        Retorna el valor en dBm.
        RSSI = -RSSI_PACKET/2 dBm
        """
        rssi_pkt, _, _ = self.get_packet_status()
        # Convertir de complemento a 2 a dBm
        # RSSI = -RSSI_PACKET/2 dBm
        rssi_dbm = -(rssi_pkt / 2.0)
        return rssi_dbm
    
    def get_packet_rssi_from_register(self):
        """
        Obtiene el RSSI del último paquete recibido leyendo directamente el registro 0x0890.
        Este método es una alternativa más confiable si GetPacketStatus no funciona correctamente.
        Retorna el valor en dBm.
        RSSI = -RSSI_PACKET/2 dBm
        """
        rssi_raw = self.read_register(self._REG_RSSI_PACKET, 1)
        rssi_pkt = rssi_raw[0] if len(rssi_raw) > 0 else 0
        # Convertir de complemento a 2 a dBm
        # RSSI = -RSSI_PACKET/2 dBm
        rssi_dbm = -(rssi_pkt / 2.0)
        return rssi_dbm