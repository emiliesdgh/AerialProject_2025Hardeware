import time
import struct
import sys
import os

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4DCompressedTrajectory
from cflib.crazyflie.high_level_commander import HighLevelCommander

from uav_trajectories import Trajectory

# === CONFIGURATION ===
URI = 'radio://0/80/2M/E7E7E7E718'  # Change if needed
TRAJ_ID = 1
CSV_FILE = 'gate_trajectory.csv'
WAYPOINTS = [  # [x, y, z, yaw]
    [0.0, 0.0, 0.4, 0.0],     # Start
    [1.0, 0.0, 0.4, 0.0],     # Gate 1
    [1.5, 0.5, 0.5, 45.0],    # Gate 2
    [2.0, 0.0, 0.4, 0.0],     # Gate 3
    [2.5, 0.0, 0.4, 0.0]      # End
]
TOTAL_TIME = 6.0  # seconds for entire trajectory
TIME_SCALE = 0.8  # 0.8 = 20% faster

class CrazyflieFlight:
    def __init__(self, uri):
        self.uri = uri
        self.cf = Crazyflie()
        self.mem = None
        self.high_level_commander = None
        self.connected = False

    def connect(self):
        cflib.crtp.init_drivers()
        print("Connecting to Crazyflie...")
        self.cf.connected.add_callback(self._on_connected)
        self.cf.disconnected.add_callback(self._on_disconnected)
        self.cf.connection_failed.add_callback(self._on_connection_failed)
        self.cf.open_link(self.uri)

    def _on_connected(self, link_uri):
        print(f"Connected to {link_uri}")
        self.connected = True

        self.cf.param.set_value('stabilizer.estimator', '2')       # Kalman
        self.cf.param.set_value('commander.enHighLevel', '1')      # High-level commander

        time.sleep(1)

        self.high_level_commander = HighLevelCommander(self.cf)
        self._generate_trajectory()
        self._upload_and_run_trajectory()

    def _generate_trajectory(self):
        if os.path.exists(CSV_FILE):
            print(f"Trajectory already exists: {CSV_FILE}")
            return
        print("Generating trajectory...")
        traj = Trajectory()
        traj.generate_waypoint_trajectory(WAYPOINTS, total_time=TOTAL_TIME)
        traj.write_to_file(CSV_FILE)
        print(f"Trajectory saved to {CSV_FILE}")

    def _upload_and_run_trajectory(self):
        traj = Poly4DCompressedTrajectory()
        traj.load_from_file(CSV_FILE)
        print(f"Loaded trajectory with {len(traj.segments)} segments.")

        def on_mem_found(mem):
            print("Found trajectory memory, uploading...")
            mem.write_data(0, traj.data, callback=on_upload_done)

        def on_upload_done(mem):
            print("Upload done. Defining and starting trajectory...")
            self.high_level_commander.define_trajectory(TRAJ_ID, 0, len(traj.segments), traj.type)
            time.sleep(0.5)

            print("Taking off...")
            self.high_level_commander.takeoff(0.4, 2.0)  # To 0.4m in 2 seconds
            time.sleep(3.0)

            print("Starting trajectory...")
            self.high_level_commander.start_trajectory(
                trajectory_id=TRAJ_ID,
                time_scale=TIME_SCALE,
                relative=True,
                reversed=False,
                group_mask=0b1111
            )

            # Wait for trajectory to complete
            estimated_duration = TOTAL_TIME * TIME_SCALE
            time.sleep(estimated_duration + 1.0)

            print("Landing...")
            self.high_level_commander.land(0.0, 2.0)
            time.sleep(3.0)

            print("Flight complete. Disconnecting.")
            self.cf.close_link()

        self.mem = MemoryElement(self.cf)
        self.mem.element_found.add_callback(on_mem_found)
        self.mem.request_element()

    def _on_disconnected(self, link_uri):
        print("Disconnected.")
        self.connected = False

    def _on_connection_failed(self, link_uri, msg):
        print(f"Connection failed: {msg}")
        sys.exit(1)


if __name__ == '__main__':
    flight = CrazyflieFlight(URI)
    flight.connect()

    while flight.connected:
        time.sleep(1)
