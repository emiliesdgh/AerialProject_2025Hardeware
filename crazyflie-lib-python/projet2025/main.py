import numpy as np
import cv2
import time 


from map import OccupancyGrid as OG


if __name__ == "__main__":
    # Initialize the occupancy grid
    occupancy_grid = OG()
    # # Set the gates in the occupancy grid
    # occupancy_grid.set_gates([10, 10, 0, 0], [100, 100, 0, 0], [40, 20, 0, 45], [120, 35, 0, 25], [100, 90, 0, 90], [50, 120, 0, 135])
    # # Display the occupancy grid
    # while(True):
    #     occupancy_grid.display_map()

    # Create the occupancy grid

    # Set start, end, and gate positions (x, y, z, theta, extra)
    start = np.array([75, 25, 1, 0])
    end = np.array([75, 25, 1, 0])
    gate1 = np.array([100, 50, 0.5, 45, 0])
    gate2 = np.array([100, 100, 0.75, 135, 0])
    gate3 = np.array([50, 100, 1, 225, 0])
    gate4 = np.array([50, 50, 0.75, 315, 0])

    # Set the gate and waypoint data
    occupancy_grid.set_gates(start, end, gate1, gate2, gate3, gate4)
    occupancy_grid.plan_through_gates()

    # Optionally: add obstacles to the map
    # og.grid[50:70, 50:70] = 255

    # Run path planning
    # path = occupancy_grid.path_planning()

    # Display result in a loop (until key press)
    while True:
        occupancy_grid.display_map()
        occupancy_grid.display_3d_map()
        if cv2.waitKey(50) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
