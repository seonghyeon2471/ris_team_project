from rplidar import RPLidar
import time

PORT = "/dev/ttyUSB0"

lidar = RPLidar(
    PORT,
    baudrate=460800,
    timeout=3
)

print("connected")

time.sleep(1)

print("reset")
lidar.reset()

time.sleep(2)

print("motor start")
lidar.start_motor()

time.sleep(2)

print("get info")
print(lidar.get_info())

print("health")
print(lidar.get_health())

print("scan start")

for scan in lidar.iter_scans():

    print("count =", len(scan))

    print(scan[:5])
