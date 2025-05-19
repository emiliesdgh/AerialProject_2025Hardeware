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

# from pynput import keyboard # Import the keyboard module for key press detection

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

from motion_planning import MotionPlanner3D as MP
from drone import Drone 

# TODO: CHANGE THIS URI TO YOUR CRAZYFLIE & YOUR RADIO CHANNEL
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E718')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

drone = None


class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 10s.
    """

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=50)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')

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
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        # Start a timer to disconnect in 50s     TODO: CHANGE THIS TO YOUR NEEDS
        t = Timer(50, self._cf.close_link)
        t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

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
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False


# Define your custom callback function
def emergency_stop_callback(cf):
    def on_press(key):
        try:
            if key.char == 'q':  # Check if the "space" key is pressed
                print("Emergency stop triggered!")
                cf.commander.send_stop_setpoint()  # Stop the Crazyflie
                cf.close_link()  # Close the link to the Crazyflie
                return False     # Stop the listener
        except AttributeError:
            pass

    # Start listening for key presses
    # with keyboard.Listener(on_press=on_press) as listener:
    #     listener.join()

def get_command(sensor_data):#, camera_data, dt):
    global drone#, start_time
    if drone is None:
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
        drone = Drone()
        drone.set_trajectory(motion_planner.trajectory_setpoints)
        drone.threshold = 1
        # drone.target_list =[
        #     [3.0, 3.0, 1, np.radians(0)],    # Bottom-left
        #     [5.0, 3.0, 1, np.radians(90)],   # Bottom-right
        #     [5.0, 5.0, 1, np.radians(180)],  # Top-right
        #     [3.0, 5.0, 1, np.radians(270)],  # Top-left
        #     [3.0, 3.0, 1, np.radians(0)],    # Back to start
        # ]
        
    #     start_time = time.time()
    
    # t = time.time() - start_time
    return drone.update(sensor_data)

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    le = LoggingExample(uri)
    cf = le._cf

    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

    # Replace the thread creation with the updated function
    emergency_stop_thread = threading.Thread(target=emergency_stop_callback, args=(cf,))
    emergency_stop_thread.start()

    # TODO : CHANGE THIS TO YOUR NEEDS
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
    # motion_planner = MP(waypoints, gate_positions)
    # trajectory_points = motion_planner.trajectory_setpoints


    # TODO : CHANGE THIS TO YOUR NEEDS
    # setpoints = [[0.89, -0.17, 0.77], [1.8, 0.27, 1.2], [1.8, 0.27, 1.2], [-0.21, 0.65, 1.68]]
    setpoints = (gate1[:3], gate2[:3], gate3[:3], gate4[:3])
    # times = np.linspace(0, 15, len(setpoints))
    # yaw_setpoints = [0, np.pi / 2, -np.pi, -np.pi / 2]
    # disc_steps = 50

    # Create the motion planner
    # motion_planner = MP(waypoints, gate_positions)
    motion_planner = MP(setpoints, gate_positions)
    trajectory_points = motion_planner.trajectory_setpoints

    poly_coeffs = motion_planner.compute_poly_coefficients(setpoints)
    # trajectory_setpoints, time_setpoints = motion_planner.poly_setpoint_extraction(poly_coeffs, waypoints)
    trajectory_setpoints, time_setpoints = motion_planner.poly_setpoint_extraction(poly_coeffs, setpoints)

    # print(motion_planner.trajectory_setpoints)


    print("Starting control")
    while le.is_connected:
        time.sleep(0.01)

        
        ### === Take-off === ###
        for y in range(10):
            cf.commander.send_hover_setpoint(0, 0, 0, y / 25)
            time.sleep(0.1)
        for _ in range(20):
            cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
            time.sleep(0.1)
        ### ================ ###


        ########### === A essayer === ###########
        #########################################
        # # premier essai, mettre le reste en commentaire

        # for i in range(120):
        #     for _ in range(3):
        #         cf.commander.send_position_setpoint(trajectory_points[i,0], trajectory_points[i,1], trajectory_points[i,2], 0)
        #         time.sleep(0.1)
        #     i = i+2
        
        # #########################################
        # # deuxieme essai, mettre le reste en commentaire
        # j = 0
        # for i in range(60):
        #     for _ in range(3):
        #         cf.commander.send_position_setpoint(trajectory_points[j,0], trajectory_points[j,1], trajectory_points[j,2], 0)
        #         time.sleep(0.1)
        #     j = j+2
        
        # #########################################
        # # troisieme essai, mettre le reste en commentaire
        # j = 0
        # for i in range(60):
        #     for _ in range(3):
        #         cf.commander.send_position_setpoint(trajectory_points[j][0], trajectory_points[j][1], trajectory_points[j][2], 0)
        #         time.sleep(0.1)
        #     j = j+2

        ### === Move === ###
        for setpoint, time_setpoint in zip(trajectory_setpoints, time_setpoints):
            cf.commander.send_position_setpoint(setpoint[0], setpoint[1], setpoint[2], setpoint[3])
            time.sleep(time_setpoint)
        ### ============ ###
        
        

        ### === Land === ###
        for _ in range(20):
            cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
            time.sleep(0.1)
        for y in range(10):
            cf.commander.send_hover_setpoint(0, 0, 0, (10 - y) / 25)
            time.sleep(0.1)
        ### ============ ###

        cf.commander.send_stop_setpoint()
        break
