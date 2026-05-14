import numpy as np
import math
from config import *

class LocalMapper:

    def __init__(self):

        self.grid = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.float32)
        self.visit = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.float32)

        self.robot_x = MAP_SIZE // 2
        self.robot_y = MAP_SIZE - 50

    def add_virtual_wall(self):

        # 뒤쪽 완전 금지 (부채꼴)
        for y in range(self.robot_y, MAP_SIZE):
            for x in range(MAP_SIZE):
                self.grid[y, x] = 1.0

    def update(self, scan):

        self.visit *= VISIT_DECAY

        self.add_virtual_wall()

        # 로봇 위치 cost 감소
        self.visit[self.robot_y-2:self.robot_y+2,
                   self.robot_x-2:self.robot_x+2] += 1

        for angle, dist in scan:

            rad = math.radians(angle)

            x = int(self.robot_x + (dist/1000.0) * math.sin(rad) / MAP_RESOLUTION)
            y = int(self.robot_y - (dist/1000.0) * math.cos(rad) / MAP_RESOLUTION)

            if 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE:
                self.grid[y, x] = 1.0
