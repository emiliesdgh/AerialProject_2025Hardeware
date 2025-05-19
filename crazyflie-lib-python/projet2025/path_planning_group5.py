# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Simple example that connects to the first Crazyflie found, logs the Stabilizer
and prints it to the console.

The Crazyflie is controlled using the commander interface

Press q to Kill the drone in case of emergency

After 50s the application disconnects and exits.
"""
import logging
import time
from threading import Timer
import threading
import numpy as np
# from pynput import keyboard  # Import the keyboard module for key press detection

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

# TODO: CHANGE THIS URI TO YOUR CRAZYFLIE & YOUR RADIO CHANNEL
uri = uri_helper.uri_from_env(default="radio://0/100/2M/E7E7E7E720")

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 10s.
    """

    def __init__(self, link_uri):
        """Initialize and run the example with the specified link_uri"""

        self._cf = Crazyflie(rw_cache="./cache")

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print("Connecting to %s" % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print("Connected to %s" % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name="Stabilizer", period_in_ms=50)
        self._lg_stab.add_variable("stateEstimate.x", "float")
        self._lg_stab.add_variable("stateEstimate.y", "float")
        self._lg_stab.add_variable("stateEstimate.z", "float")
        self._lg_stab.add_variable("stabilizer.yaw", "float")

        # The fetch-as argument can be set to FP16 to save space in the log packet
        # self._lg_stab.add_variable('pm.vbat', 'FP16')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print("Could not start log configuration," "{} not found in TOC".format(str(e)))
        except AttributeError:
            print("Could not add Stabilizer log config, bad configuration.")

        # Start a timer to disconnect in 50s     TODO: CHANGE THIS TO YOUR NEEDS
        t = Timer(50, self._cf.close_link)
        t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print("Error when logging %s: %s" % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""

        # Print the data to the console
        # print(f'[{timestamp}][{logconf.name}]: ', end='')
        # for name, value in data.items():
        #     print(f'{name}: {value:3.3f} ', end='')
        # print()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print("Connection to %s failed: %s" % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print("Connection to %s lost: %s" % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print("Disconnected from %s" % link_uri)
        self.is_connected = False


# Define your custom callback function
def emergency_stop_callback(cf):
    def on_press(key):
        try:
            if key.char == "q":  # Check if the "space" key is pressed
                print("Emergency stop triggered!")
                cf.commander.send_stop_setpoint()  # Stop the Crazyflie
                cf.close_link()  # Close the link to the Crazyflie
                return False  # Stop the listener
        except AttributeError:
            pass

    # Start listening for key presses
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()


##########
def compute_poly_matrix(t):
    # Inputs:
    # - t: The time of evaluation of the A matrix (t=0 at the start of a path segment, else t >= 0) [Scalar]
    # Outputs:
    # - The constraint matrix "A_m(t)" [5 x 6]
    # The "A_m" matrix is used to represent the system of equations [x, \dot{x}, \ddot{x}, \dddot{x}, \ddddot{x}]^T  = A_m(t) * poly_coeffs (where poly_coeffs = [c_0, c_1, c_2, c_3, c_4, c_5]^T and represents the unknown polynomial coefficients for one segment)
    A_m = np.zeros((5, 6))

    # TASK: Fill in the constraint factor matrix values where each row corresponds to the positions, velocities, accelerations, snap and jerk here
    # SOLUTION ---------------------------------------------------------------------------------- ##

    A_m = np.array(
        [
            [t**5, t**4, t**3, t**2, t, 1],  # pos
            [5 * (t**4), 4 * (t**3), 3 * (t**2), 2 * t, 1, 0],  # vel
            [20 * (t**3), 12 * (t**2), 6 * t, 2, 0, 0],  # acc
            [60 * (t**2), 24 * t, 6, 0, 0, 0],  # jerk
            [120 * t, 24, 0, 0, 0, 0],  # snap
        ]
    )

    return A_m


def compute_times_from_waypoints_with_acc(setpoints, v_max, a_max):
    """
    Args:
        setpoints: list of waypoints [ [x0, y0, z0], [x1, y1, z1], ... ]
        v_max: maximum allowed velocity (float)
        a_max: maximum allowed acceleration (float)
    Returns:
        times: list of times corresponding to each waypoint
    """
    setpoints_array = np.array(setpoints)
    setpoints_array = setpoints_array[:, 0:3]
    # print(setpoints_array)
    dists = np.linalg.norm(setpoints_array[1:] - setpoints_array[:-1], axis=1)
    dists += 1e-8  # avoid singular matrix

    seg_times = []
    for d in dists:
        d_acc = (v_max**2) / (2 * a_max)

        if d >= 2 * d_acc:
            t_acc = v_max / a_max
            d_cruise = d - 2 * d_acc
            t_cruise = d_cruise / v_max
            t_total = 2 * t_acc + t_cruise
        else:
            v_peak = np.sqrt(a_max * d)
            t_acc = v_peak / a_max
            t_total = 2 * t_acc

        seg_times.append(t_total)

    times = [0.0]
    for t in seg_times:
        times.append(times[-1] + t)

    return times


def compute_poly_coefficients(path_waypoints, times):

    # Computes a minimum jerk trajectory given time and position waypoints.
    # Inputs:
    # - path_waypoints: The sequence of input path waypoints provided by the path-planner, including the start and final goal position: Vector of m waypoints, consisting of a tuple with three reference positions each as provided by AStar
    # Outputs:
    # - poly_coeffs: The polynomial coefficients for each segment of the path [6(m-1) x 3]

    # Use the following variables and the class function self.compute_poly_matrix(t) to solve for the polynomial coefficients

    seg_times = np.diff(times)  # The time taken to complete each path segment
    m = len(path_waypoints)  # Number of path waypoints (including start and end)
    poly_coeffs = np.zeros((6 * (m - 1), 3))

    # YOUR SOLUTION HERE ---------------------------------------------------------------------------------- ##

    # 1. Fill the entries of the constraint matrix A and equality vector b for x,y and z dimensions in the system A * poly_coeffs = b. Consider the constraints according to the lecture: We should have a total of 6*(m-1) constraints for each dimension.
    # 2. Solve for poly_coeffs given the defined system

    for dim in range(3):  # Compute for x, y, and z separately
        A = np.zeros((6 * (m - 1), 6 * (m - 1)))
        b = np.zeros(6 * (m - 1))
        pos = np.array([p[dim] for p in path_waypoints])
        A_0 = compute_poly_matrix(0)  # A_0 gives the constraint factor matrix A_m for any segment at t=0, this is valid for the starting conditions at every path segment

        # SOLUTION
        row = 0
        for i in range(m - 1):
            pos_0 = pos[i]  # Starting position of the segment
            pos_f = pos[i + 1]  # Final position of the segment
            # The prescribed zero velocity (v) and acceleration (a) values at the start and goal position of the entire path
            v_0, a_0 = 0, 0
            v_f, a_f = 0, 0
            A_f = compute_poly_matrix(seg_times[i])  # A_f gives the constraint factor matrix A_m for a segment i at its relative end time t=seg_times[i]
            if i == 0:  # First path segment
                #     # 1. Implement the initial constraints here for the first segment using A_0
                #     # 2. Implement the final position and the continuity constraints for velocity, acceleration, snap and jerk at the end of the first segment here using A_0 and A_f (check hints in the exercise description)
                A[row, i * 6 : (i + 1) * 6] = A_0[0]  # Initial position constraint
                b[row] = pos_0
                row += 1
                A[row, i * 6 : (i + 1) * 6] = A_f[0]  # Final position constraint
                b[row] = pos_f
                row += 1
                A[row, i * 6 : (i + 1) * 6] = A_0[1]  # Initial velocity constraint
                b[row] = v_0
                row += 1
                A[row, i * 6 : (i + 1) * 6] = A_0[2]  # Initial acceleration constraint
                b[row] = a_0
                row += 1
                # Continuity of velocity, acceleration, jerk, snap
                A[row : row + 4, i * 6 : (i + 1) * 6] = A_f[1:]
                A[row : row + 4, (i + 1) * 6 : (i + 2) * 6] = -A_0[1:]
                b[row : row + 4] = np.zeros(4)
                row += 4
            elif i < m - 2:  # Intermediate path segments
                #     # 1. Similarly, implement the initial and final position constraints here for each intermediate path segment
                #     # 2. Similarly, implement the end of the continuity constraints for velocity, acceleration, snap and jerk at the end of each intermediate segment here using A_0 and A_f
                A[row, i * 6 : (i + 1) * 6] = A_0[0]  # Initial position constraint
                b[row] = pos_0
                row += 1
                A[row, i * 6 : (i + 1) * 6] = A_f[0]  # Final position constraint
                b[row] = pos_f
                row += 1
                # Continuity of velocity, acceleration, jerk and snap
                A[row : row + 4, i * 6 : (i + 1) * 6] = A_f[1:]
                A[row : row + 4, (i + 1) * 6 : (i + 2) * 6] = -A_0[1:]
                b[row : row + 4] = np.zeros(4)
                row += 4
            elif i == m - 2:  # Final path segment
                #     # 1. Implement the initial and final position, velocity and accelerations constraints here for the final path segment using A_0 and A_f
                A[row, i * 6 : (i + 1) * 6] = A_0[0]  # Initial position constraint
                b[row] = pos_0
                row += 1
                A[row, i * 6 : (i + 1) * 6] = A_f[0]  # Final position constraint
                b[row] = pos_f
                row += 1
                A[row, i * 6 : (i + 1) * 6] = A_f[1]  # Final velocity constraint
                b[row] = v_f
                row += 1
                A[row, i * 6 : (i + 1) * 6] = A_f[2]  # Final acceleration constraint
                b[row] = a_f
                row += 1
        # Solve for the polynomial coefficients for the dimension dim

        poly_coeffs[:, dim] = np.linalg.solve(A, b)

    return poly_coeffs


def poly_setpoint_extraction(poly_coeffs, yaw_setpoints, times, disc_steps):

    # DO NOT MODIFY --------------------------------------------------------------------------------------- ##

    # Uses the class features: self.disc_steps, times, self.poly_coeffs, self.vel_lim, self.acc_lim
    x_vals, y_vals, z_vals = np.zeros((disc_steps * len(times), 1)), np.zeros((disc_steps * len(times), 1)), np.zeros((disc_steps * len(times), 1))
    v_x_vals, v_y_vals, v_z_vals = np.zeros((disc_steps * len(times), 1)), np.zeros((disc_steps * len(times), 1)), np.zeros((disc_steps * len(times), 1))
    a_x_vals, a_y_vals, a_z_vals = np.zeros((disc_steps * len(times), 1)), np.zeros((disc_steps * len(times), 1)), np.zeros((disc_steps * len(times), 1))

    # Define the time reference in self.disc_steps number of segements
    time_setpoints = np.linspace(times[0], times[-1], disc_steps * len(times))  # Fine time intervals

    # Extract the x,y and z direction polynomial coefficient vectors
    coeff_x = poly_coeffs[:, 0]
    coeff_y = poly_coeffs[:, 1]
    coeff_z = poly_coeffs[:, 2]

    for i, t in enumerate(time_setpoints):
        seg_idx = min(max(np.searchsorted(times, t) - 1, 0), len(coeff_x) - 1)
        # Determine the x,y and z position reference points at every refernce time
        x_vals[i, :] = np.dot(compute_poly_matrix(t - times[seg_idx])[0], coeff_x[seg_idx * 6 : (seg_idx + 1) * 6])
        y_vals[i, :] = np.dot(compute_poly_matrix(t - times[seg_idx])[0], coeff_y[seg_idx * 6 : (seg_idx + 1) * 6])
        z_vals[i, :] = np.dot(compute_poly_matrix(t - times[seg_idx])[0], coeff_z[seg_idx * 6 : (seg_idx + 1) * 6])
        # Determine the x,y and z velocities at every reference time
        v_x_vals[i, :] = np.dot(compute_poly_matrix(t - times[seg_idx])[1], coeff_x[seg_idx * 6 : (seg_idx + 1) * 6])
        v_y_vals[i, :] = np.dot(compute_poly_matrix(t - times[seg_idx])[1], coeff_y[seg_idx * 6 : (seg_idx + 1) * 6])
        v_z_vals[i, :] = np.dot(compute_poly_matrix(t - times[seg_idx])[1], coeff_z[seg_idx * 6 : (seg_idx + 1) * 6])
        # Determine the x,y and z accelerations at every reference time
        a_x_vals[i, :] = np.dot(compute_poly_matrix(t - times[seg_idx])[2], coeff_x[seg_idx * 6 : (seg_idx + 1) * 6])
        a_y_vals[i, :] = np.dot(compute_poly_matrix(t - times[seg_idx])[2], coeff_y[seg_idx * 6 : (seg_idx + 1) * 6])
        a_z_vals[i, :] = np.dot(compute_poly_matrix(t - times[seg_idx])[2], coeff_z[seg_idx * 6 : (seg_idx + 1) * 6])

    yaw_vals = np.zeros((disc_steps * len(times), 1))
    # compute yaw angle waypoint

    def angle_lerp(a0, a1, t):
        diff = (a1 - a0 + np.pi) % (2 * np.pi) - np.pi
        return a0 + diff * t

    yaw_waypoints = np.unwrap(yaw_setpoints)
    num_segments = len(yaw_waypoints) - 1
    segment_time = times[-1] / num_segments
    print(yaw_waypoints)

    yaw_vals = np.zeros_like(time_setpoints)
    for i, t in enumerate(time_setpoints):
        seg_idx = min(int(t // segment_time), num_segments - 1)
        t0 = seg_idx * segment_time
        alpha = (t - t0) / segment_time
        yaw_vals[i] = angle_lerp(yaw_waypoints[seg_idx], yaw_waypoints[seg_idx + 1], alpha)
    yaw_vals = np.array(yaw_vals).reshape(-1, 1)

    trajectory_setpoints = np.hstack((x_vals, y_vals, z_vals, yaw_vals))

    # Find the maximum absolute velocity during the segment
    vel_max = np.max(np.sqrt(v_x_vals**2 + v_y_vals**2 + v_z_vals**2))
    vel_mean = np.mean(np.sqrt(v_x_vals**2 + v_y_vals**2 + v_z_vals**2))
    acc_max = np.max(np.sqrt(a_x_vals**2 + a_y_vals**2 + a_z_vals**2))
    acc_mean = np.mean(np.sqrt(a_x_vals**2 + a_y_vals**2 + a_z_vals**2))

    return trajectory_setpoints, time_setpoints


##########


if __name__ == "__main__":
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    le = LoggingExample(uri)
    cf = le._cf

    cf.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.1)
    cf.param.set_value("kalman.resetEstimation", "0")
    time.sleep(2)

    # Replace the thread creation with the updated function
    emergency_stop_thread = threading.Thread(target=emergency_stop_callback, args=(cf,))
    emergency_stop_thread.start()

    # TODO : CHANGE THIS TO YOUR NEEDS
    setpoints = [[0.89, -0.17, 0.77], [1.8, 0.27, 1.2], [1.8, 0.27, 1.2], [-0.21, 0.65, 1.68]]
    times = np.linspace(0, 15, len(setpoints))
    yaw_setpoints = [0, np.pi / 2, -np.pi, -np.pi / 2]
    disc_steps = 50
    poly_coeffs = compute_poly_coefficients(setpoints, times)
    trajectory_setpoints, time_setpoints = poly_setpoint_extraction(poly_coeffs, yaw_setpoints, times, disc_steps)

    print("number of trajectory_setpoints", trajectory_setpoints.shape)


    print("Starting control")
    while le.is_connected:
        time.sleep(0.01)

        # Take-off
        for y in range(10):
            cf.commander.send_hover_setpoint(0, 0, 0, y / 25)
            time.sleep(0.1)
        for _ in range(20):
            cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
            time.sleep(0.1)

        # Move
        for setpoint, time_setpoint in zip(trajectory_setpoints, time_setpoints):
            cf.commander.send_position_setpoint(setpoint[0], setpoint[1], setpoint[2], setpoint[3])
            time.sleep(time_setpoint)

        # Land
        for _ in range(20):
            cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
            time.sleep(0.1)
        for y in range(10):
            cf.commander.send_hover_setpoint(0, 0, 0, (10 - y) / 25)
            time.sleep(0.1)

        cf.commander.send_stop_setpoint()
        break
