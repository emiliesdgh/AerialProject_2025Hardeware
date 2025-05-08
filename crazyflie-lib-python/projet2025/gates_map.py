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
        self.grid = np.zeros((3, 5), dtype=np.uint8)
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
        self.start = start
        self.end = end

        self.gate1 = gate1
        self.gate2 = gate2
        self.gate3 = gate3
        self.gate4 = gate4

        self.waypoints = [start[:3], gate1[:3], gate2[:3], gate3[:3], gate4[:3], end[:3]]

    def set_optimal_path(self, path):
        # Set the optimal path
        self.optimal_path = path
        

    def densify_path(self, path, spacing=1.0):
        if len(path) < 2:
            return path

        densified = []
        for i in range(len(path) - 1):
            x1, y1, *rest1 = path[i]
            x2, y2, *rest2 = path[i + 1]
            dist = np.hypot(x2 - x1, y2 - y1) #--> 2D
            # dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2) # 3D
            num_points = max(int(dist / spacing), 1)

            for j in range(num_points):
                t = j / num_points
                x = x1 + t * (x2 - x1)
                y = y1 + t * (y2 - y1)
                # z = z1 + t * (z2 - z1)
                densified.append((x, y))#, z))

        densified.append(path[-1])  # include the final point
        return densified


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
            [-w/2,  h/2]
        ])

        # Rotation matrix
        rotation = np.array([
            [np.cos(angle_rad), -np.sin(angle_rad)],
            [np.sin(angle_rad),  np.cos(angle_rad)]
        ])

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
            size = gate[4]*10, 1  # size of the gate
            self.draw_rotated_rectangle(color_grid, (x, y), theta, size, color=color, label=label)

        # for label, (pos, color) in gates.items():
        #     x, y = int(pos[0]), int(pos[1])
        #     cv2.circle(color_grid, (x, y), 3, color, -1)
        #     cv2.putText(color_grid, label, (x+5, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # Draw waypoints
        for i, waypoint in enumerate(self.waypoints):
            x, y = int(waypoint[0]), int(waypoint[1])
            cv2.circle(color_grid, (x, y), 4, (255, 255, 255), -1)  # White for waypoints
            cv2.putText(color_grid, f'W{i+1}', (x+5, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1) # Orange

        # Draw start and end points
        sx, sy = int(self.start[0]), int(self.start[1])
        ex, ey = int(self.end[0]), int(self.end[1])
        cv2.circle(color_grid, (sx, sy), 4, (255, 255, 255), -1)
        cv2.putText(color_grid, 'Start', (sx+5, sy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        cv2.circle(color_grid, (ex, ey), 4, (255, 255, 255), -1)  
        cv2.putText(color_grid, 'End', (ex+5, ey-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)

        if self.optimal_path:
            # for i in range(1, len(self.optimal_path)):
            #     pt1 = (int(self.optimal_path[i - 1][0]), int(self.optimal_path[i - 1][1]))
            #     pt2 = (int(self.optimal_path[i][0]), int(self.optimal_path[i][1]))

            #     cv2.line(color_grid, pt1, pt2, (255, 255, 255), 1)
            densified_points = self.densify_path(self.optimal_path, spacing=1.0)
            for point in densified_points:
                x, y = int(round(point[0])), int(round(point[1]))
                # Draw the densified point as a small dot
                cv2.circle(color_grid, (x, y), 1, (0, 255, 255), -1)  # Yellow dots
            
            # Smooth the path before drawing it
            # smoothed_path = self.smooth_path(self.optimal_path, smoothing_factor=0)
            # for i in range(len(smoothed_path) - 1):
            #     pt1 = (int(round(smoothed_path[i][0])), int(round(smoothed_path[i][1])))
            #     pt2 = (int(round(smoothed_path[i + 1][0])), int(round(smoothed_path[i + 1][1])))
            #     cv2.line(color_grid, pt1, pt2, (255, 255, 255), 1)
            for i in range(len(densified_points) - 1):
                pt1 = (int(round(densified_points[i][0])), int(round(densified_points[i][1])))
                pt2 = (int(round(densified_points[i + 1][0])), int(round(densified_points[i + 1][1])))
                cv2.line(color_grid, pt1, pt2, (255, 255, 255), 1)  # White for the path


        # Display the map
        # Resize for better visibility
        scale = 50  # Increase this for a bigger display
        resized_grid = cv2.resize(color_grid, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        cv2.imshow('Occupancy Grid with Gates -- 2D slice (z=0)', resized_grid)
        cv2.waitKey(1)



    # def display_3d_map(self):
    #     fig = plt.figure()
    #     ax = fig.add_subplot(111, projection='3d')

    #     # Gate points
    #     gates = {
    #         "G1": self.gate1,  # [x, y, z, theta]
    #         "G2": self.gate2,
    #         "G3": self.gate3,
    #         "G4": self.gate4
    #     }
        
    #     for label, gate in gates.items():
    #         x, y, z = gate[0], gate[1], gate[2]
    #         ax.scatter(x, y, z, label=label)
    #         ax.text(x, y, z, label, size=10, color='k')
        
    #     # Start and End points
    #     ax.scatter(self.start[0], self.start[1], self.start[2], c='r', label='Start')
    #     ax.scatter(self.end[0], self.end[1], self.end[2], c='g', label='End')

    #     # Plot path (assuming self.path is a list of [x, y, z])
    # # if hasattr(self, 'path'):
    #     xs = [p[0] for p in self.optimal_path]
    #     ys = [p[1] for p in self.optimal_path]
    #     zs = [p[2] for p in self.optimal_path]
    #     ax.plot(xs, ys, zs, c='blue', label='Path')
    
    
    #     # Display the 3D plot
    #     plt.title('3D Occupancy Grid with Gates')
    #     plt.legend(loc='upper left', fontsize='small', frameon=True)
    #     plt.show()

    # def astar(self, start, goal):
    #     open_set = []
    #     heapq.heappush(open_set, (0, start))
    #     came_from = {}
    #     g_score = {tuple(start): 0}
    #     f_score = {tuple(start): self.heuristic(start, goal)}

    #     while open_set:
    #         current = heapq.heappop(open_set)[1]

    #         if np.array_equal(current[:2], goal[:2]):
    #             return self.reconstruct_path(came_from, current)

    #         # if current == goal:
    #         #     return self.reconstruct_path(came_from, current)

    #         for neighbor in self.get_neighbors(current):
    #             move_cost = np.linalg.norm(np.array(neighbor[:2])-np.array(current[:2]))
    #             tentative_g = g_score[tuple(current)] + move_cost
    #             if tuple(neighbor) not in g_score or tentative_g < g_score[tuple(neighbor)]:
    #                 came_from[tuple(neighbor)] = current
    #                 g_score[tuple(neighbor)] = tentative_g
    #                 f_score[tuple(neighbor)] = tentative_g + self.heuristic(neighbor, goal)
    #                 if tuple(neighbor) not in [i[1] for i in open_set]:
    #                     heapq.heappush(open_set, (f_score[tuple(neighbor)], neighbor))
    #     return None
    
    # def get_neighbors(self, node):
    #     x, y, z = int(node[0]), int(node[1]), int(node[2])
    #     neighbors = []

    #     ### in 3D ###
    #     # 26 possible directions (6 straight + 18 diagonal, 3D)
    #     directions = [(-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0), (0, 0, -1), (0, 0, 1),  # Straight directions
    #                   (-1, -1, 0), (1, -1, 0), (-1, 1, 0), (1, 1, 0),  # Diagonal on xy-plane
    #                   (-1, 0, -1), (1, 0, -1), (0, -1, -1), (0, 1, -1), (-1, -1, -1), (1, -1, -1),
    #                   (-1, 1, -1), (1, 1, -1), (-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
    #                   (-1, -1, 1), (1, -1, 1), (-1, 1, 1), (1, 1, 1)]  # Diagonal on z-axis

    #     for dx, dy, dz in directions:
    #         nx, ny, nz = x + dx, y + dy, z + dz
    #         # if 0 <= nx < self.grid.shape[1] and 0 <= ny < self.grid.shape[0] and 0 <= nz < self.grid.shape[2]:    
    #         if 0 <= nz < self.grid.shape[0] and 0 <= ny < self.grid.shape[1] and 0 <= nx < self.grid.shape[2]:

    #             if self.grid[nz, ny, nx] != 255:  # Assuming 255 is obstacle
    #                 neighbors.append((nx, ny, nz))
    #     return neighbors


    # def heuristic(self, a, b):
    #     # return (abs(a[0] - b[0]) + abs(a[1] - b[1]))  # Manhattan distance --> in 2D
    #     return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2)  # Euclidean distance --> in 3D


    # def smooth_path(self, path, smoothing_factor=0):
    #     # print("len(path):", len(path))
    #     if len(path) < 3:
    #         print("Path too short to smooth")
    #         return path  # Too short to smooth

    #     x = [p[0] for p in path]
    #     y = [p[1] for p in path]
    #     z = [p[2] for p in path]

    #     try:
    #     # Parametric spline
    #         tck, u = splprep([x, y, z], s=smoothing_factor)

    #         # Generate a smoother path using spline interpolation
    #         unew = np.linspace(0, 1.0, num=10 * len(path))
    #         out = splev(unew, tck)

    #         # Create the smoothed path
    #         smoothed_path = list(zip(np.round(out[0]).astype(int), np.round(out[1]).astype(int), np.round(out[2]).astype(int)))
    #         # print(f"Smoothed path (rounded): {smoothed_path}")
    #         return smoothed_path

    #     except Exception as e:
    #         # print(f"Error in smoothing the path: {e}")
    #         # print("x:", x)
    #         # print("y:", y)
    #         return path  # Return original path in case of error


