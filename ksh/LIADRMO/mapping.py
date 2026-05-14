import numpy as np
import math

from config import *

class LocalMapper:

    def __init__(self):

        self.grid = np.zeros(
            (MAP_SIZE, MAP_SIZE),
            dtype=np.uint8
        )

        self.visit = np.zeros(
            (MAP_SIZE, MAP_SIZE),
            dtype=np.float32
        )

        self.robot_x = MAP_SIZE // 2
        self.robot_y = MAP_SIZE - 50

        self.block_back_area()

    def block_back_area(self):

        self.grid[self.robot_y:, :] = 1

    def update(self, scan):

        self.visit *= VISIT_DECAY

        self.visit[
            self.robot_y - 2:self.robot_y + 2,
            self.robot_x - 2:self.robot_x + 2
        ] += 1

        for angle, dist in scan:

            rad = math.radians(angle)

            x = int(
                self.robot_x +
                (dist / 1000.0) *
                math.sin(rad) /
                MAP_RESOLUTION
            )

            y = int(
                self.robot_y -
                (dist / 1000.0) *
                math.cos(rad) /
                MAP_RESOLUTION
            )

            if x < 0 or x >= MAP_SIZE:
                continue

            if y < 0 or y >= MAP_SIZE:
                continue

            self.grid[y, x] = 1
