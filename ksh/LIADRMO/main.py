import time

from lidar import SimpleLidar
from mapping import LocalMapper
from planner import GapPlanner
from motor import MotorController

# =========================
# INIT
# =========================

lidar = SimpleLidar()

mapper = LocalMapper()

planner = GapPlanner()

motor = MotorController()

print("SYSTEM START")

# =========================
# LOOP
# =========================

try:

    while True:

        scan = lidar.read_scan()

        if len(scan) == 0:
            continue

        mapper.update(scan)

        v, w = planner.compute_control(scan)

        motor.send(v, w)

        time.sleep(0.03)

except KeyboardInterrupt:

    print("STOP")

    motor.stop()
