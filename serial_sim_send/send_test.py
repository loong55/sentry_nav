import serial
import time

hex_str = "5A 23 0B 77 00 00 00 00 07 01 C4 07 D0 07 00 00 00 00 00 00 00 00 00 00 00 00 00 00 15 8E 06 43 10 00 00 00 00 00 00 5F 04"
data = bytes.fromhex(hex_str)

SERIAL_PORT = "/tmp/ttyV1"
BAUD_RATE = 115200
FLOW_CONTROL = "none"
PARITY = "none"
STOP_BITS = "1"

ser = serial.Serial(
    port=SERIAL_PORT,
    baudrate=BAUD_RATE,
    timeout=0,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE if PARITY == "none" else serial.PARITY_EVEN,
    stopbits=serial.STOPBITS_ONE if STOP_BITS == "1" else serial.STOPBITS_TWO,
    xonxoff=(FLOW_CONTROL == "software"),
    rtscts=(FLOW_CONTROL == "hardware"),
    dsrdtr=False,
)

while True:
    ser.write(data)
    ser.flush()
    time.sleep(0.05)