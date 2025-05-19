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
    start = np.array([0, 0, 0.5, 0, 0.6])
    end = np.array([0, 0, 0.5, 0, 0.6])

    gate1 = np.array([1.14, -0.54, 0.81, 90, 0.6])
    gate2 = np.array([2.09, 0.33, 1.18, 180, 0.6])

    gate3 = np.array([0.55, 0.65, 1.5, 270, 0.6])
    gate4 = np.array([-0.4, 0.69, 1.6, 0, 0.6])

    waypoints = (start[:3], gate1[:3], gate2[:3], gate3[:3], gate4[:3], end[:3])
    gate_positions = (gate1, gate2, gate3, gate4)

    # trajectory_points = motion_planner.trajectory_setpoints

    # TODO : CHANGE THIS TO YOUR NEEDS
    # setpoints = [[1.14, -0.54, 0.81], [2.09, 0.33, 1.18], [0.55, 0.65, 1.5], [-0.4, 0.69, 1.6]]
    setpoints = (gate1[:3], gate2[:3], gate3[:3], gate4[:3])
    # times = np.linspace(0, 15, len(setpoints))
    # yaw_setpoints = [0, np.pi / 2, -np.pi, -np.pi / 2]
    # disc_steps = 50

    # Create the motion planner
    motion_planner = MP(setpoints, gate_positions)
    # motion_planner = MP(setpoints, gate_positions)

    poly_coeffs = motion_planner.compute_poly_coefficients(setpoints)
    # trajectory_setpoints, time_setpoints = motion_planner.poly_setpoint_extraction(poly_coeffs, waypoints)
    trajectory_setpoints, time_setpoints = motion_planner.poly_setpoint_extraction(poly_coeffs, setpoints)

    print("number of trajectory_setpoints", trajectory_setpoints.shape)

    # # Set the gate and waypoint data
    gate_map.set_gates(start, end, gate1, gate2, gate3, gate4)
    gate_map.set_optimal_path(motion_planner.trajectory_setpoints)

    print(trajectory_setpoints.shape)

    # Display result in a loop (until key press)
    while True:
        gate_map.display_map()
        if cv2.waitKey(50) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
