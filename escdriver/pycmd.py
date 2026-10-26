import serial
import struct
import time

# ─── Configuration ────────────────────────────────────────────────────────────

SERIAL_PORT  = "/dev/ttyUSB0"
BAUD_RATE    = 115200
LOOP_DELAY   = 1.0   # seconds between telemetry requests
READ_TIMEOUT = 1.0   # seconds before giving up on a read

# VESC packet constants
PACKET_START_SHORT = 0x02
PACKET_STOP        = 0x03
COMM_GET_VALUES    = 0x04


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

# ─── Packet builder ───────────────────────────────────────────────────────────

def build_packet(payload: bytes) -> bytes:
    length = len(payload)
    crc    = crc16(payload)
    return bytes([
        PACKET_START_SHORT,
        length,
        *payload,
        (crc >> 8) & 0xFF,
        crc & 0xFF,
        PACKET_STOP,
    ])

# ─── Telemetry parsing ────────────────────────────────────────────────────────

def parse_get_values(payload: bytes) -> dict | None:
    if len(payload) < 55:
        print(f"Payload too short: {len(payload)} bytes")
        return None

    # Skip the command byte at index 0
    off = 1

    def i16():
        nonlocal off
        v = struct.unpack_from(">h", payload, off)[0]
        off += 2
        return v

    def i32():
        nonlocal off
        v = struct.unpack_from(">i", payload, off)[0]
        off += 4
        return v

    return {
        "temp_fet":        i16() / 10.0,
        "temp_motor":      i16() / 10.0,
        "current_motor":   i32() / 100.0,
        "current_input":   i32() / 100.0,
        "_skip":           (off := off + 8),   # skip id/iq
        "duty_cycle":      i16() / 1000.0,
        "rpm":             i32(),
        "voltage_input":   i16() / 10.0,
        "amp_hours":       i32() / 10000.0,
        "amp_hours_chg":   i32() / 10000.0,
        "watt_hours":      i32() / 10000.0,
        "watt_hours_chg":  i32() / 10000.0,
        "tachometer":      i32(),
        "tachometer_abs":  i32(),
        "fault_code":      payload[off],
    }

def print_values(v: dict):
    print("─────────────────────────────")
    print(f"FET temp:      {v['temp_fet']:.1f} °C")
    print(f"Motor temp:    {v['temp_motor']:.1f} °C")
    print(f"Motor current: {v['current_motor']:.2f} A")
    print(f"Input current: {v['current_input']:.2f} A")
    print(f"Duty cycle:    {v['duty_cycle'] * 100:.1f} %")
    print(f"RPM:           {v['rpm']}")
    print(f"Input voltage: {v['voltage_input']:.1f} V")
    print(f"Amp-hours:     {v['amp_hours']:.4f} Ah")
    print(f"Watt-hours:    {v['watt_hours']:.4f} Wh")
    print(f"Fault code:    {v['fault_code']}")

# ─── Main loop ────────────────────────────────────────────────────────────────

def main():
    ser = serial.Serial(
        port      = SERIAL_PORT,
        baudrate  = BAUD_RATE,
        bytesize  = serial.EIGHTBITS,
        parity    = serial.PARITY_NONE,
        stopbits  = serial.STOPBITS_ONE,
        timeout   = READ_TIMEOUT,
    )
    print(f"Connected to {SERIAL_PORT}. Reading telemetry...")

    request = build_packet(bytes([COMM_GET_VALUES]))

    while True:
        try:
            # 1. Send request
            ser.write(request)

            # 2. Read header: [start byte, length]
            header = ser.read(2)
            if len(header) < 2:
                print("Timeout waiting for response header")
                time.sleep(LOOP_DELAY)
                continue
            if header[0] != PACKET_START_SHORT:
                print(f"Bad start byte: 0x{header[0]:02X}")
                ser.reset_input_buffer()
                time.sleep(LOOP_DELAY)
                continue

            payload_len = header[1]

            # 3. Read payload + CRC (2 bytes) + stop byte (1 byte)
            tail = ser.read(payload_len + 3)
            if len(tail) < payload_len + 3:
                print("Timeout reading response body")
                time.sleep(LOOP_DELAY)
                continue

            payload   = tail[:payload_len]
            recv_crc  = (tail[payload_len] << 8) | tail[payload_len + 1]
            stop_byte = tail[payload_len + 2]

            # 4. Verify CRC and stop byte
            if recv_crc != crc16(payload):
                print("CRC mismatch!")
                time.sleep(LOOP_DELAY)
                continue
            if stop_byte != PACKET_STOP:
                print(f"Bad stop byte: 0x{stop_byte:02X}")
                time.sleep(LOOP_DELAY)
                continue

            # 5. Parse and print
            values = parse_get_values(payload)
            if values:
                print_values(values)

        except KeyboardInterrupt:
            print("\nExiting.")
            break
        except Exception as e:
            print(f"Error: {e}")

        time.sleep(LOOP_DELAY)

    ser.close()

if __name__ == "__main__":
    main()
