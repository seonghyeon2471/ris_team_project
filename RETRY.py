import serial
import time

# =========================================
# MOTOR SERIAL
# =========================================

motor_ser = serial.Serial(
    "/dev/serial0",
    115200,
    timeout=1
)

# =========================================
# MOTOR COMMAND
# =========================================

def send_cmd(v, w):

    msg = f"{v:.3f},{w:.3f}\n"

    motor_ser.write(msg.encode())

# =========================================
# MAIN
# =========================================

print("RIGHT TURN TEST")

try:

    while True:

        # 제자리 우회전

        v = 0.0

        w = -1.0

        send_cmd(v, w)

        print(
            f"v:{v:.2f} | "
            f"w:{w:.2f}"
        )

        time.sleep(0.1)

except KeyboardInterrupt:

    print("STOP")

    send_cmd(0.0, 0.0)

    motor_ser.close()
