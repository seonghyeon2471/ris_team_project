import serial
import time

# 아두이노 UART
ser = serial.Serial(
    "/dev/serial0",
    115200,
    timeout=0.1
)

time.sleep(2)

print("START")

# 직진
ser.write(b"0.2,0.0\n")

start = time.time()

while time.time() - start < 5:

    line = ser.readline().decode(
        errors="ignore"
    ).strip()

    if line:
        print(line)

# 정지
ser.write(b"0.0,0.0\n")

print("STOP")
