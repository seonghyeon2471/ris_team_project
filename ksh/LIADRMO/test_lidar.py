from lidar import SimpleLidar

lidar = SimpleLidar()

while True:

    scan = lidar.read_scan()

    if len(scan) > 0:

        print(
            scan[:5],
            "count =",
            len(scan)
        )
