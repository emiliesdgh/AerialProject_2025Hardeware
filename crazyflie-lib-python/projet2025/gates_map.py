import numpy as np
import cv2
# import heapq
from scipy.interpolate import splprep, splev
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# Constants
min_x, max_x = 0, 4.15 # meter
min_y, max_y = 0, 2.55 # meter
min_z, max_z = 0, 2 # meter


# obstacle_augmentation = 2

class GatesMap:
    def __init__(self):
        # Initialize the occupancy grid
        self.grid = np.zeros((255, 415), dtype=np.uint8)
        # Initialize the waypoints
        #    gates             x  y  z theta ...?
        self.gate1 = np.zeros(5)
        self.gate2 = np.zeros(5)
        self.gate3 = np.zeros(5)
        self.gate4 = np.zeros(5)

        # Initialize the start and end points
        self.start = np.zeros(5)
        self.end = np.zeros(5)

        self.waypoints = []

        # Path planning
        self.optimal_path = []
        
    def set_gates(self, start, end, gate1, gate2, gate3, gate4):
        # Set the gates in the occupancy grid
        self.start = (start+np.array([1, 1, 1, 0, 0])) * np.array([100,100,100,1,100])
        self.end = (end+np.array([1, 1, 1, 0, 0])) * np.array([100,100,100,1,100])

        self.gate1 = (gate1+np.array([1, 1, 1, 0, 0])) * np.array([100,100,100,1,100])
        self.gate2 = (gate2+np.array([1, 1, 1, 0, 0])) * np.array([100,100,100,1,100])
        self.gate3 = (gate3+np.array([1, 1, 1, 0, 0])) * np.array([100,100,100,1,100])
        self.gate4 = (gate4+np.array([1, 1, 1, 0, 0])) * np.array([100,100,100,1,100])

        self.waypoints = [self.start[:3], self.gate1[:3], self.gate2[:3], self.gate3[:3], self.gate4[:3], self.end[:3]]
        print("Waypoints set:", self.waypoints)

    def set_optimal_path(self, path):
        # Set the optimal path
        self.optimal_path = (path+1)*100

###########################
### === Map display === ###
###########################
    def draw_rotated_rectangle(self, img, center, angle_deg, size, color, label=None):
        cx, cy = center
        w, h = size
        angle_rad = np.deg2rad(angle_deg)

        # Define the rectangle corners relative to the center
        rect = np.array([
            [-w/2, -h/2],
            [ w/2, -h/2],
            [ w/2,  h/2],
            [-w/2,  h/2] ])

        # Rotation matrix
        rotation = np.array([
            [np.cos(angle_rad), -np.sin(angle_rad)],
            [np.sin(angle_rad),  np.cos(angle_rad)] ])

        # Rotate and translate corners
        rotated_rect = np.dot(rect, rotation) + np.array([cx, cy])
        pts = rotated_rect.astype(np.int32).reshape((-1, 1, 2))

        # Draw the rectangle
        cv2.polylines(img, [pts], isClosed=True, color=color, thickness=1)

        # Optional: draw label
        if label:
            cv2.putText(img, label, (int(cx + 5), int(cy - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    
    def display_map(self):
        # Display the occupancy grid using OpenCV
        # Convert to color for visualization (grayscale to BGR)
        # slice_2d = self.grid[0]  # Use the first slice for 2D visualization
        color_grid = cv2.cvtColor(self.grid, cv2.COLOR_GRAY2BGR)

        # Draw gates with different colors
        gates = {
            "G1": (self.gate1, (0, 0, 255)),   # Red
            "G2": (self.gate2, (0, 255, 0)),   # Green
            "G3": (self.gate3, (255, 0, 0)),   # Blue
            "G4": (self.gate4, (0, 255, 255))  # Yellow
        }
        for label, (gate, color) in gates.items():
            x, y = int(gate[0]), int(gate[1])
            theta = gate[3]  # heading in degrees
            size = gate[4], 1  # size of the gate
            self.draw_rotated_rectangle(color_grid, (x, y), theta, size, color=color, label=label)

        # Draw waypoints
        for i, waypoint in enumerate(self.waypoints):
            x, y = int(waypoint[0]), int(waypoint[1])
            cv2.circle(color_grid, (x, y), 4, (0, 165, 255), -1)  # orange for waypoints

        # Draw start and end points
        sx, sy = int(self.start[0]), int(self.start[1])
        ex, ey = int(self.end[0]), int(self.end[1])
        cv2.circle(color_grid, (sx, sy), 4, (255, 255, 255), -1)
        cv2.putText(color_grid, 'Start', (sx-35, sy+10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        cv2.circle(color_grid, (ex, ey), 4, (255, 255, 255), -1)  
        cv2.putText(color_grid, 'End', (ex+5, ey-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        for i in range(1, len(self.optimal_path)):
            pt1 = (int(self.optimal_path[i - 1][0]), int(self.optimal_path[i - 1][1]))
            pt2 = (int(self.optimal_path[i][0]), int(self.optimal_path[i][1]))

            cv2.line(color_grid, pt1, pt2, (255, 255, 255), 1) # white line
            
        for point in self.optimal_path:
            x, y = int(round(point[0])), int(round(point[1]))
            cv2.circle(color_grid, (x, y), 1, (200, 200, 200), -1)  # gray dots

        # Display the map
        # Resize for better visibility
        cv2.circle(color_grid, (10, 10), 4, (0, 165, 255), -1)
        cv2.putText(color_grid, 'Waypoints', (20, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
        scale = 2#5  # Increase this for a bigger display
        resized_grid = cv2.resize(color_grid, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        cv2.imshow('Gate map in 2D', resized_grid)
        cv2.waitKey(1)