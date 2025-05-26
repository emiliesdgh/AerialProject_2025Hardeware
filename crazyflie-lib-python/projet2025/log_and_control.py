PERIOD_MS = 50

import logging
import time
import threading
import numpy as np

from pynput import keyboard

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

class DroneInterface:
    def __init__(self, uri, drone_instance):
        self.uri = uri
        self.cf = Crazyflie(rw_cache='./cache')
        self.drone = drone_instance
        self.current_sensor_data = None

        self.cf.connected.add_callback(self._connected) # add callback function on connect event
        self.cf.disconnected.add_callback(self._disconnected) # add callback function on disconnect event
        print(f"Connecting to {uri}...")
        self.cf.open_link(uri)

        keyboard.Listener(on_press=self._on_key_press).start()  #listen on keyboard for input

    def _connected(self, link_uri):
        print(f"Connected to {link_uri}")

        #subscribing to position
        log_conf = LogConfig(name='Position', period_in_ms=50)
        log_conf.add_variable('kalman.stateX', 'float')
        log_conf.add_variable('kalman.stateY', 'float')
        log_conf.add_variable('kalman.stateZ', 'float')


        #registering the function to call when we get an answer
        def log_callback(timestamp, data, logconf):
            sensor_data = {
                'x_global': data['kalman.stateX'],
                'y_global': data['kalman.stateY'],
                'z_global': data['kalman.stateZ'],
                'yaw': 0.0
            }
            self.current_sensor_data = sensor_data
            target = self.drone.update(sensor_data)
            self.cf.commander.send_position_setpoint(*target)

        time.sleep(1)
        log_conf.data_received_cb.add_callback(log_callback)
        self.cf.log.add_config(log_conf)
        log_conf.start()

        self.cf.param.set_value('stabilizer.estimator', '2')
        self.cf.param.set_value('commander.enHighLevel', '1')

        self.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self.cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

    def _on_key_press(self, key):
        """Function fo emergency stop"""
        try:
            if key.char == 'q':
                print("Emergency stop triggered!")
                self.cf.commander.send_stop_setpoint()
                self.cf.close_link()
                return False
        except AttributeError:
            pass

    def _disconnected(self, uri):
        print("Disconnected.")

def run_drone(uri, drone):
    """wrapper to start everything"""
    cflib.crtp.init_drivers()
    interface = DroneInterface(uri, drone)
    while True:
        time.sleep(1)
