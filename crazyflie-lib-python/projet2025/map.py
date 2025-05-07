############################## Description ##############################
# This file contains the OccupancyGrid class 
# This class does 3 main things:
# 1. Update the map with the given information
# 2. Display the map with OpenCV
# 3. Run the A* algorithm to find the optimal path from start to end
#########################################################################

import numpy as np
# import matplotlib.pyplot as plt
import cv2
import heapq

# Constants
min_x, max_x = 0, 5.0 # meter
min_y, max_y = 0, 3.0 # meter
res_pos = 0.1   # meter


obstacle_augmentation = 2   # margin around obstacles
# map = np.zeros((int((max_x-min_x)/res_pos), int((max_y-min_y)/res_pos))) # 0 = unknown, 1 = free, -1 = occupied


class OccupancyGrid:
    def __init__(self):
        # Initialize the occupancy grid
        self.map = np.zeros((int((max_x-min_x)/res_pos), int((max_y-min_y)/res_pos))) # 0 = unknown, 1 = free, -1 = occupied

        self.grid = np.zeros((100, 100), dtype=np.uint8)
        # Initialize the waypoints
        #    gates             x  y  z theta ...?
        self.gate1 = np.array([0, 0, 0, 0, 0])
        self.gate2 = np.array([0, 0, 0, 0, 0])
        self.gate3 = np.array([0, 0, 0, 0, 0])
        self.gate4 = np.array([0, 0, 0, 0, 0])

        # Initialize the start and end points
        self.start = np.array([0, 0, 0, 0])
        self.end = np.array([0, 0, 0, 0])

        # Path planning
        self.optimal_path = None

    def display_map(self):
        # Display the occupancy grid using OpenCV
        # cv2.imshow('Occupancy Grid', self.grid)
        # cv2.waitKey(1)
        """
        Displays the occupancy grid map with the drone position and the planned path
        """

        zoom = 20 * self.map.shape[0]
        map_display = np.transpose(self.map)
        map_display = (map_display+1)/2
        width, height = np.shape(map_display)

        map_display = cv2.resize(map_display,(zoom*height, zoom*width), interpolation=cv2.INTER_NEAREST) # zoom for better visualization
        map_display = cv2.cvtColor(map_display.astype('float32'), cv2.COLOR_GRAY2BGR) # convert to color image
    
        # # Mark drone position in blue
        # if sensor_data is not None :
        #     pos_x = sensor_data['x_global']
        #     pos_y = sensor_data['y_global']
        #     idx_x, idx_y = self.cell_from_pos([pos_x, pos_y], display = True)
        #     cv2.circle(map_display, (zoom*idx_x, zoom*idx_y), 2,(255,0,0),-1)

        # Mark path
        if self.optimal_path is not None:
            # path in green
            for cell in self.optimal_path :
                cv2.circle(map_display, zoom*tuple(cell), 1,(0,255,0),-1)
            # target in red
            cv2.circle(map_display, zoom*tuple(self.optimal_cell_path[-1]), 2,(0,0, 255),-1)


        # flip the image to have the origin at the bottom left
        map_display = np.flip(map_display, axis= 0).astype(float)
        cv2.imshow('Cell map and planned path', map_display)
        cv2.waitKey(1)

    def set_gates(self, start, end, gate1, gate2, gate3, gate4):
        # Set the gates in the occupancy grid
        self.gate1 = gate1
        self.gate2 = gate2
        self.gate3 = gate3
        self.gate4 = gate4

        self.start = start
        self.end = end

    def path_planning(self):
        # Run the A* algorithm to find the optimal path from start to end
        open_set = []
        heapq.heappush(open_set, (0, self.start))
        came_from = {}
        g_score = {tuple(self.start): 0}
        f_score = {tuple(self.start): self.heuristic(self.start, self.end)}

        while open_set:
            current = heapq.heappop(open_set)[1]
            if np.array_equal(current, self.end):
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[tuple(current)] + 1
                if tuple(neighbor) not in g_score or tentative_g_score < g_score[tuple(neighbor)]:
                    came_from[tuple(neighbor)] = current
                    g_score[tuple(neighbor)] = tentative_g_score
                    f_score[tuple(neighbor)] = tentative_g_score + self.heuristic(neighbor, self.end)
                    if tuple(neighbor) not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[tuple(neighbor)], neighbor))

        return None
    
    
    