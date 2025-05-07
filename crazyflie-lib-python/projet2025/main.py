import numpy as np
import time 


from map import OccupancyGrid as OG


if __name__ == "__main__":
    # Initialize the occupancy grid
    occupancy_grid = OG()
    # Set the gates in the occupancy grid
    occupancy_grid.set_gates([0, 0, 0, 0], [10, 10, 0, 0], [1, 1, 0, 0], [2, 2, 0, 0], [3, 3, 0, 0], [4, 4, 0, 0])
    # Display the occupancy grid
    while(True):
        occupancy_grid.display_map()