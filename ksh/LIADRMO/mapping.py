import numpy as np
import math
from config import *

class LocalMapper:

    def __init__(self):

        self.grid = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.float32)
        self.visit = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.float32)

        self.robot_x = MAP_SIZE // 2
        self.robot_y = MAP_SIZE - 50

    # 🔥 출발 뒤쪽 + 하단 완전 금지
    def add_virtual_wall(self):

        self.grid[self.robot_y:, :] = 1.0

    def update(self, scan):

        # decay
        self.visit *= VISIT_DECAY

        # robot footprint
        self.visit[self.robot_y-2:self.robot_y+2,
                   self.robot_x-2:self.robot_x+2] += 1.0

        self.add_virtual_wall()

        # lidar projection
        for angle, dist in scan:

            rad = math.radians(angle)

            x = int(self.robot_x + (dist/1000.0) * math.sin(rad) / MAP_RESOLUTION)
            y = int(self.robot_y - (dist/1000.0) * math.cos(rad) / MAP_RESOLUTION)

            if 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE:
                self.grid[y, x] = 1.0
