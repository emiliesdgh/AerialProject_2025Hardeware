import numpy as np
import cv2
import time 


from drone import Drone
from log_and_control import run_drone
from motion_planning import MotionPlanner3D as MP




def createPoints():
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

    # Create the motion planner
    motion_planner = MP(waypoints, gate_positions)

    # # # print the waypoints
    # # print("Waypoints:", waypoints) 
    # # # print the interpolated trajectory setpoints
    # # print("Interpolated trajectory setpoints:", motion_planner.trajectory_setpoints)
    return motion_planner

if __name__ == "__main__":
    mp = createPoints()
    drone = Drone()
    drone.set_trajectory(mp.trajectory_setpoints)

    run_drone("radio://0/80/2M/E7E7E7E718", drone)

