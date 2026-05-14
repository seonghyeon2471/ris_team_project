import numpy as np
from config import *

class LocalMapper:

    def __init__(self):

        self.grid = np.zeros((400, 400), dtype=np.float32)

        self.robot_x = 200
        self.robot_y = 350

    def update(self, scan):

        # 뒤쪽 차단 (출발역행 방지)
        self.grid[self.robot_y:, :] = 1.0

        for angle, dist in scan:

            pass  # 현재는 planner에서 처리 (light weight)
