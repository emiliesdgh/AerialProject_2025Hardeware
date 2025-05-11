import numpy as np



# === Constants ===
TARGET_Z = 1.35
THRESHOLD = 1

class Drone:
    def __init__(self):
        self._state = "takeoff"
        self.target = None
        self.target_list = []
        self.base = [1, 4, TARGET_Z, 0]

        self._sensor_data = None



    def update(self, sensor_data):
        self._sensor_data = sensor_data

        if self._state == "takeoff":
            self._takeoff()
        elif self._state == "run":
            self._run()
        else:
            self.target = [sensor_data['x_global'], sensor_data['y_global'], TARGET_Z, sensor_data['yaw']]
        
        return self.target

    def _takeoff(self):
        target_z = 1.0
        self.target = [
            self._sensor_data['x_global'],
            self._sensor_data['y_global'],
            target_z,
            self._sensor_data['yaw']
        ]
        if self._target_reached():
            if self.target_list:
                self._state = "run"

    def feed_path(self, points):
        """Feed a list of [x, y, z, yaw] waypoints to follow after takeoff."""
        self.target_list = points.copy()
        if self._state != "takeoff":
            self._state = "run"

    def _run(self):
        self._follow_path(0.4)

    def _follow_path(self, threshold=THRESHOLD):
        if not self.target_list:
            return True

        self.target = self.target_list[0]
        if self._target_reached(threshold=threshold, check_yaw=True):
            self.target_list.pop(0)
            if self.target_list:
                self.target = self.target_list[0]
            else:
                return True
        return False

    def _target_reached(self, threshold=0.1, check_yaw=False, yaw_threshold=0.1):
        dx = self._sensor_data['x_global'] - self.target[0]
        dy = self._sensor_data['y_global'] - self.target[1]
        dz = self._sensor_data['z_global'] - self.target[2]
        distance = np.sqrt(dx**2 + dy**2 + dz**2)

        if check_yaw:
            dyaw = np.abs((self._sensor_data['yaw'] - self.target[3] + np.pi) % (2 * np.pi) - np.pi)
            return distance < threshold and dyaw < yaw_threshold
        else:
            return distance < threshold
