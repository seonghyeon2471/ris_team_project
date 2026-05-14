import time

from lidar import SimpleLidar
from mapping import LocalMapper
from planner import GapPlanner
from motor import MotorController

lidar = SimpleLidar()
mapper = LocalMapper()
planner = GapPlanner()
motor = MotorController()

print("SYSTEM START")

try:

    while True:

        scan = lidar.read_scan()

        if not scan:
            continue

        mapper.update(scan)

        v, w = planner.compute_control(scan)

        motor.send(v, w)

        print(f"v={v:.2f}, w={w:.2f}")

        # 🔥 핵심: sleep 너무 크면 buffer 쌓임
        time.sleep(0.01)

except KeyboardInterrupt:

    print("STOP")

    motor.stop()
    lidar.stop()
