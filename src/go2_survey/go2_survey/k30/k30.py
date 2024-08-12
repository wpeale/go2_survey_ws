import serial
import struct
import time

K30_BAUD = 9600
CO2_PPM_CMD = b"\xFE\x44\x00\x08\x02\x9F\x25"

class K30Sensor:
    def __init__(self, serial_port="/dev/ttyUSB0", timeout=.5):
        self.ser = serial.Serial(serial_port, baudrate=K30_BAUD, timeout=timeout)
        self.ser.flushInput()
        time.sleep(1)

    def read_co2_ppm(self):
        self.ser.flushInput()
        self.ser.write(CO2_PPM_CMD)
        response = self.ser.read(7)
        co2 = struct.unpack('>H', response[3:5])[0]
        return co2

if __name__ == "__main__":
    sensor = K30Sensor()
    while True:
        try:
            ppm = sensor.read_co2_ppm()
            print(f"CO2 concentration is: {ppm} ppm.")
            time.sleep(1)  # delay between readings
        except KeyboardInterrupt:
            print("\nExiting...")
            break
