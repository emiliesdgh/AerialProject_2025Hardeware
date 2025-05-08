import numpy as np
import cv2
import time 


from gates_map import GatesMap as GM
from motion_planning import MotionPlanner3D as MP


if __name__ == "__main__":
    # Initialize the occupancy grid
    gate_map = GM()

    # Set start, end, and gate positions (x, y, z, theta, gate size)
    # all positions are given in meters and theta in degrees
    start = np.array([1.5, 1.5, 0, 0, 0.6])
    end = np.array([1.5, 1.5, 1, 0, 0.6])

    gate1 = np.array([2.25, 2.05, 0.8, 90, 0.6])
    gate2 = np.array([3.6, 1.5, 1.2, 180, 0.6])

    gate3 = np.array([2.3, 0.2, 1.7, 270, 0.6])
    gate4 = np.array([1.45, 0.8, 1.6, 0, 0.6])

    waypoints = (start[:3], gate1[:3], gate2[:3], gate3[:3], gate4[:3], end[:3])
    gate_positions = (gate1, gate2, gate3, gate4)

    # Create the motion planner
    motion_planner = MP(waypoints, gate_positions)

    # # # print the waypoints
    # # print("Waypoints:", waypoints) 
    # # # print the interpolated trajectory setpoints
    # # print("Interpolated trajectory setpoints:", motion_planner.trajectory_setpoints)

    # # Set the gate and waypoint data
    gate_map.set_gates(start, end, gate1, gate2, gate3, gate4)
    gate_map.set_optimal_path(motion_planner.trajectory_setpoints)

    # Display result in a loop (until key press)
    while True:
        gate_map.display_map()
        if cv2.waitKey(50) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
