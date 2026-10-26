#uses pyserial to send VESC packets over usb

import struct
import time

#from "/home/mo/venv/pumptesting-venv/lib/python3.14/site-packages" import serial
import serial

#command ID numbers
COMM_FW_VERSION   = 0
COMM_GET_VALUES   = 4
COMM_SET_DUTY     = 5
COMM_SET_CURRENT  = 6
COMM_SET_CURRENT_BRAKE = 7
COMM_SET_RPM      = 8
COMM_SET_POS      = 9
COMM_SET_HANDBRAKE = 10
COMM_REBOOT       = 29
COMM_ALIVE        = 30

FAULT_CODES = {
    0: "NONE",
    1: "OVER_VOLTAGE",
    2: "UNDER_VOLTAGE",
    3: "DRV",
    4: "ABS_OVER_CURRENT",
    5: "OVER_TEMP_FET",
    6: "OVER_TEMP_MOTOR",
    7: "GATE_DRIVER_OVER_VOLTAGE",
    8: "GATE_DRIVER_UNDER_VOLTAGE",
    9: "MCU_UNDER_VOLTAGE",
    10: "BOOTING_FROM_WATCHDOG_RESET",
    11: "ENCODER_SPI",
    12: "ENCODER_SINCOS_BELOW_MIN_AMPLITUDE",
    13: "ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE",
    14: "FLASH_CORRUPTION",
    15: "HIGH_OFFSET_CURRENT_SENSOR_1",
    16: "HIGH_OFFSET_CURRENT_SENSOR_2",
    17: "HIGH_OFFSET_CURRENT_SENSOR_3",
    18: "UNBALANCED_CURRENTS",
}

def crc16(data: bytes) -> int:
    """CRC-16/XMODEM: poly 0x1021, init 0x0000, no reflection."""
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

class _Reader:
    """Sequential big-endian reader that raises EOFError past the end.

    Lets us parse telemetry field-by-field and stop gracefully when the
    firmware sends a shorter packet than the newest field list expects.
    (stole this from claude, hopefully it works lmao)
    """

    def __init__(self, buf: bytes):
        self.buf = buf
        self.i = 0

    def take(self, fmt: str):
        size = struct.calcsize(fmt)
        if self.i + size > len(self.buf):
            raise EOFError
        val = struct.unpack_from(fmt, self.buf, self.i)[0]
        self.i += size
        return val

class VESC:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.2):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(0.1)
        self.ser.reset_input_buffer()

    #message framers

    @staticmethod
    def _frame(payload: bytes) -> bytes:
        n = len(payload)
        if n < 256:
            head = bytes([0x02, n])
        else:
            head = bytes([0x03, (n >> 8) & 0xFF, n & 0xFF])
        return head + payload + struct.pack(">H", crc16(payload)) + b"\x03"

    def _send(self, payload: bytes) -> None:
        self.ser.write(self._frame(payload))
        self.ser.flush()

    def _read_frame(self):
        """Read one packet. Returns the payload, or None on timeout/bad CRC."""
        start = self.ser.read(1)
        if not start:
            return None
        if start[0] == 0x02:
            hdr = self.ser.read(1)
            if len(hdr) != 1:
                return None
            n = hdr[0]
        elif start[0] == 0x03:
            hdr = self.ser.read(2)
            if len(hdr) != 2:
                return None
            n = struct.unpack(">H", hdr)[0]
        else:
            return None  #got garbage, caller can retry to resync

        payload = self.ser.read(n)
        tail = self.ser.read(3)
        if len(payload) != n or len(tail) != 3:
            return None
        if tail[2] != 0x03:
            return None
        if struct.unpack(">H", tail[:2])[0] != crc16(payload):
            return None
        return payload

    def _request(self, payload: bytes, expect_id: int, retries: int = 3):
        """Send a request and wait for a reply with the matching command ID."""
        for _ in range(retries):
            self.ser.reset_input_buffer()
            self._send(payload)
            deadline = time.time() + 0.5
            while time.time() < deadline:
                reply = self._read_frame()
                if reply and reply[0] == expect_id:
                    return reply
        return None

    #reads

    def get_firmware_version(self):
        reply = self._request(bytes([COMM_FW_VERSION]), COMM_FW_VERSION)
        if not reply or len(reply) < 3:
            return None
        return (reply[1], reply[2])

    def get_values(self):
        """Return a dict of telemetry, or None if no valid reply."""
        reply = self._request(bytes([COMM_GET_VALUES]), COMM_GET_VALUES)
        if not reply:
            return None

        r = _Reader(reply)
        r.take(">B")  #discard the command ID
        v = {}
        try:
            v["temp_fet_c"] = r.take(">h") / 10.0
            v["temp_motor_c"] = r.take(">h") / 10.0
            v["current_motor_a"] = r.take(">i") / 100.0
            v["current_in_a"] = r.take(">i") / 100.0
            v["id_a"] = r.take(">i") / 100.0
            v["iq_a"] = r.take(">i") / 100.0
            v["duty"] = r.take(">h") / 1000.0
            v["erpm"] = r.take(">i")
            v["v_in"] = r.take(">h") / 10.0
            v["amp_hours"] = r.take(">i") / 10000.0
            v["amp_hours_charged"] = r.take(">i") / 10000.0
            v["watt_hours"] = r.take(">i") / 10000.0
            v["watt_hours_charged"] = r.take(">i") / 10000.0
            v["tachometer"] = r.take(">i")
            v["tachometer_abs"] = r.take(">i")
            v["fault_code"] = r.take(">B")
            v["pid_pos"] = r.take(">i") / 1e6
            v["controller_id"] = r.take(">B")
            #fields below exist on newer firmware only
            v["temp_mos1_c"] = r.take(">h") / 10.0
            v["temp_mos2_c"] = r.take(">h") / 10.0
            v["temp_mos3_c"] = r.take(">h") / 10.0
            v["vd"] = r.take(">i") / 1000.0
            v["vq"] = r.take(">i") / 1000.0
        except EOFError:
            pass #catch old firmware

        v["fault"] = FAULT_CODES.get(v.get("fault_code", 0), "UNKNOWN")
        return v

    #writes
        
    #duty from -1.0 to 1.0
    def set_duty(self, duty: float) -> None:
        duty = max(-1.0, min(1.0, duty))
        self._send(struct.pack(">Bi", COMM_SET_DUTY, int(duty * 100000)))

    def set_current(self, amps: float) -> None:
        self._send(struct.pack(">Bi", COMM_SET_CURRENT, int(amps * 1000)))

    def set_brake_current(self, amps: float) -> None:
        self._send(struct.pack(">Bi", COMM_SET_CURRENT_BRAKE, int(amps * 1000)))

    def set_rpm(self, erpm: int) -> None:
        """Electrical RPM. Mechanical RPM = erpm / (pole_pairs)."""
        self._send(struct.pack(">Bi", COMM_SET_RPM, int(erpm)))

    def set_handbrake(self, amps: float) -> None:
        self._send(struct.pack(">Bi", COMM_SET_HANDBRAKE, int(amps * 1000)))

    #reset timeout watchdog
    def alive(self) -> None:
        self._send(bytes([COMM_ALIVE]))

    #gee I wonder what this does?!
    def stop(self) -> None:
        self.set_current(0.0)


    #command cycle fns
    def close(self) -> None:
        try:
            self.stop()
        finally:
            self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

if __name__ == "__main__":
    PORT = "/dev/ttyACM0" #linux port
    #PORT = "COM3"

    with VESC(PORT) as v:
        fw = v.get_firmware_version()
        if fw is None:
            raise SystemExit("no comms")
        print(f"Connected -- firmware {fw[0]}.{fw[1]}")

        #telem loop
        for _ in range(20):
            d = v.get_values()
            if d:
                print(
                    f"{d['v_in']:5.1f} V | {d['current_in_a']:6.2f} A in | "
                    f"{d['erpm']:7d} erpm | duty {d['duty']:+.3f} | "
                    f"FET {d['temp_fet_c']:4.1f} C | {d['fault']}"
                )
            time.sleep(0.1)

        #
        # try:
        #     for _ in range(50):          # ~5 s at 5% duty
        #         v.set_duty(0.05)
        #         print(v.get_values()["erpm"])
        #         time.sleep(0.1)
        # finally:
        #     v.stop()
