from typing import List, Tuple
from geometry import Point

class Viewport:
    def __init__(self):
        self.min_x: float = 0.0
        self.min_y: float = 0.0
        self.max_x: float = 1.0
        self.max_y: float = 1.0

    def fit_points(self, pts: List[Point], padding: float = 20.0):
        if not pts:
            self.min_x, self.min_y, self.max_x, self.max_y = 0, 0, 1, 1
            return
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        
        # Ensure non-zero dimensions
        if abs(max_x - min_x) < 1e-9:
            min_x -= 0.5
            max_x += 0.5
        if abs(max_y - min_y) < 1e-9:
            min_y -= 0.5
            max_y += 0.5
            
        self.min_x = min_x - padding
        self.min_y = min_y - padding
        self.max_x = max_x + padding
        self.max_y = max_y + padding

    def world_to_screen(self, p: Point, width: int, height: int) -> Point:
        dx = max(self.max_x - self.min_x, 1e-9)
        dy = max(self.max_y - self.min_y, 1e-9)
        wx = (p[0] - self.min_x) / dx
        wy = (p[1] - self.min_y) / dy
        sx = int(wx * width)
        sy = int((1 - wy) * height)
        return (sx, sy)

    def screen_to_world(self, x: int, y: int, width: int, height: int) -> Point:
        dx = self.max_x - self.min_x
        dy = self.max_y - self.min_y
        vx = self.min_x + (x / width) * dx
        vy = self.max_y - (y / height) * dy
        return (vx, vy)
