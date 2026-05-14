from lidar import SimpleLidar

lidar = SimpleLidar()

while True:

    scan = lidar.read_scan()

    print(len(scan))
