import numpy as np
import math
from scipy.ndimage import binary_dilation

from config import *


class LocalMapper:
    """
    로봇 중심 고정 로컬 그리드 맵.
    - grid   : 0/1 점유 맵 (라이다 hit)
    - inflated: footprint 팽창 적용 costmap (A* 용)
    - visit  : 방문 빈도 히트맵
    """

    def __init__(self):

        self.grid = np.zeros(
            (MAP_SIZE, MAP_SIZE),
            dtype=np.uint8
        )

        self.inflated = np.zeros(
            (MAP_SIZE, MAP_SIZE),
            dtype=np.uint8
        )

        self.visit = np.zeros(
            (MAP_SIZE, MAP_SIZE),
            dtype=np.float32
        )

        # 로봇 위치는 항상 맵 하단 중앙 고정
        self.robot_x = MAP_SIZE // 2
        self.robot_y = MAP_SIZE - 50

        self.block_back_area()

        # -------------------------------------------------------
        # 팽창 커널: footprint 반치수 + 마진을 셀 단위로 환산
        # -------------------------------------------------------
        inflate_m = max(ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH) + FOOTPRINT_MARGIN
        inflate_cells = int(math.ceil(inflate_m / MAP_RESOLUTION))

        # 원형 커널
        r = inflate_cells
        y_idx, x_idx = np.ogrid[-r:r + 1, -r:r + 1]
        self._inflate_kernel = (x_idx ** 2 + y_idx ** 2 <= r ** 2)

    # ------------------------------------------------------------------

    def block_back_area(self):
        self.grid[self.robot_y:, :] = 1
        self.inflated[self.robot_y:, :] = 1

    # ------------------------------------------------------------------

    def update(self, scan):

        # 방문 히트맵 갱신
        self.visit *= VISIT_DECAY
        self.visit[
            self.robot_y - 2:self.robot_y + 2,
            self.robot_x - 2:self.robot_x + 2
        ] += 1

        # 이전 동적 장애물 초기화 (뒷영역 고정 제외)
        self.grid[:self.robot_y, :] = 0

        # 라이다 hit → 점유 셀 기록
        for angle, dist in scan:

            rad = math.radians(angle)

            # 라이다 오프셋 보정: 라이다는 로봇 중심보다 앞에 있음
            lidar_cell_offset = int(LIDAR_OFFSET_X / MAP_RESOLUTION)
            lidar_y = self.robot_y - lidar_cell_offset   # 맵에서 라이다 y위치

            x = int(
                self.robot_x +
                (dist / 1000.0) * math.sin(rad) / MAP_RESOLUTION
            )

            y = int(
                lidar_y -
                (dist / 1000.0) * math.cos(rad) / MAP_RESOLUTION
            )

            if 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE:
                self.grid[y, x] = 1

        # footprint 팽창 → inflated costmap 재계산
        occupied = self.grid.astype(bool)
        self.inflated = binary_dilation(
            occupied,
            structure=self._inflate_kernel
        ).astype(np.uint8)

        # 뒷영역은 항상 막음
        self.inflated[self.robot_y:, :] = 1

    # ------------------------------------------------------------------
    # 외부에서 쉽게 쿼리할 수 있는 헬퍼
    # ------------------------------------------------------------------

    def is_free(self, cx, cy):
        """셀 (cx, cy)가 inflated 맵에서 자유 공간인지."""
        if cx < 0 or cx >= MAP_SIZE or cy < 0 or cy >= MAP_SIZE:
            return False
        return self.inflated[cy, cx] == 0

    def robot_cell(self):
        return self.robot_x, self.robot_y
