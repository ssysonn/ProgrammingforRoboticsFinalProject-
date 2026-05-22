import math
from constants import CELL_SIZE

class HazardMap:
    def __init__(self):
        # Dictionary cache mapping grid tuples to hazards
        self.grid = {}

    def world_to_grid(self, x, z):
        # math.floor guarantees that -1.69 maps to -2, and -1.05 maps to -2 consistently
        gx = int(math.floor(x / CELL_SIZE))
        gz = int(math.floor(z / CELL_SIZE))
        return (gx, gz)

    def mark_hazard(self, x, z, danger_level):
        cell = self.world_to_grid(x, z)
        if cell not in self.grid:
            self.grid[cell] = danger_level
            print(f"[MAPPER] Logged Danger Zone at Cell {cell} | World Coordinates -> X: {x:.2f}, Z: {z:.2f}")

    def is_dangerous(self, x, z):
        cell = self.world_to_grid(x, z)
        return cell in self.grid