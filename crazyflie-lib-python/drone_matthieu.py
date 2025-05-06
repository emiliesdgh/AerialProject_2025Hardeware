import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

# === Constants ===
CENTER = np.array([4.0, 4.0])
TARGET_Z = 1.35
NUM_SECTORS = 12
MAX_GATES = 5
FOV = 1.5  # radians

LOWER_PURPLE = np.array([140, 100, 100])
UPPER_PURPLE = np.array([170, 255, 255])

#SEARCH_PATTERN_PARAM
NUM_TARGET_PTS = 40
THRESHOLD = 1
START_ANGLE = np.radians(90)-FOV/2
STOP_ANGLE1 = np.radians(155)-FOV/2
STOP_ANGLE2 = np.radians(270)-FOV/2



class Drone:
    def __init__(self):
        self._state = "takeoff"
        self.state_list = []
        self.gates = []
        self.sector_index = 7
        self.sector_progress = 0.0
        self.max_sector_distance = 4
        self.step_size = 0.5
        self.target = None
        self.target_list = []
        self.base = [0.1 ,1.4, TARGET_Z, 0]

        self._sensor_data = None
        self._camera_data = None

        self._gate_center = None
        self._gate_size = None
        self._gate_pixel_count = 0
        self._gate_seen = 0


        self._approach_data = type('approach_data', (), {})()  # simple empty object
        self._approach_data.missed_count = 0

        self._move_dir_data = type('approach_data', (), {})()  # simple empty object

        
    def set_state_search(self):
        self._gate_seen = 0
        self._generate_search_pattern()
        self.set_state_move([self.set_state_search])
        if(self._target_reached()):
            self._state = "search"

    def set_state_approach(self, yaw, pitch):
        self.target_list.clear()
        self._state = "approach"

        self._approach_data.pitch = pitch
        self._approach_data.yaw = yaw

    def set_state_exit_approach(self):
        self.target_list.clear()
        self.gates.append(self._get_drone_position())
        if(len(self.gates) >= MAX_GATES*2):
            self.set_state_run()
        else:
            self.set_state_search()

    def set_state_move(self, next_states=None):
        self._state = "move"
        self.state_list = next_states or []
        self.target = self.target_list.pop(0)

    def set_state_move_dir(self, yaw, pitch, distance, step = 0.3, next_states=None):
        self._move_dir_data.yaw = yaw
        self._move_dir_data.pitch = pitch
        self._move_dir_data.step = step
        self._move_dir_data.distance = distance
        self._move_dir_data.start_pos = self._get_drone_position()

        self._state = "move_dir"
        self.state_list = next_states or []

    def set_state_run(self):
        self._state = "run"
        self.target_list = self.gates + self.gates + [self.base]

    def update(self, sensor_data, camera_data, t):
        self._sensor_data = sensor_data
        self._camera_data = camera_data

        print(self._state)

        self._detect_purple_gate(True)


        if self._state == "takeoff":
            self._takeoff()
        elif self._state == "search":
            self._search()
        elif self._state == "approach":
            self._approach()
        elif self._state == "exit_approach":
            print("ERROR TRANSIENT STATE")
        elif self._state =="move":
            self._move()
        elif self._state == "move_dir":
            self._move_dir()
        elif self._state =="run":
            self._run()
        else:
            self.target = [sensor_data['x_global'], sensor_data['y_global'], TARGET_Z, sensor_data['yaw']]
        
        return self.target
 
    def _takeoff(self):
        target_z = 1.0
        self.target = [self._sensor_data['x_global'], self._sensor_data['y_global'], target_z, self._sensor_data['yaw']]
        if self._target_reached():
            self._generate_search_pattern()
            self.set_state_search()
    
    def _move(self):
            # If we reached the current target, pop it and go to the next one
            if self._target_reached(threshold= THRESHOLD,check_yaw = True):
                next_state_fn = self.state_list.pop(0)
                next_state_fn()

    def _move_dir(self):
        if(self._distance(self._get_drone_position(), self._move_dir_data.start_pos)> self._move_dir_data.distance):
            next_state_fn = self.state_list.pop(0)
            next_state_fn()

        yaw = self._move_dir_data.yaw
        pitch = self._move_dir_data.pitch 

        # Move 2 meters forward in that direction
        distance = self._move_dir_data.step
        target_x = self._sensor_data['x_global'] + distance * np.cos(yaw)
        target_y = self._sensor_data['y_global'] + distance * np.sin(yaw)

        target_z = self._sensor_data['z_global'] + distance * np.sin(pitch)
        self.target = [target_x, target_y, target_z, yaw]



    def _run(self):
        self._follow_path(0.5)

    def _follow_path(self, threshold =THRESHOLD):
        if not self.target_list:
            return True

        self.target = self.target_list[0]
        # If we reached the current target, pop it and go to the next one
        if self._target_reached(threshold= threshold, check_yaw=True):
            self.target = self.target_list.pop(0)
            if self.target_list:
                self.target = self.target = self.target_list[0]
            else:
                return True  # Path completed

        return False  # Still following path

    def _search(self):
        if(self._gate_center):
            self._gate_seen += 1
            if self._gate_seen >=10:
                img_height, img_width = self._camera_data.shape[:2]
                x_pixel = self._gate_center[0]
                y_pixel = self._gate_center[0]
                horizontal_angle_offset = (x_pixel - img_width / 2) / img_width * FOV
                vertical_angle_offset =   (y_pixel - img_height / 2) / img_height * FOV

                # Calculate absolute direction to gate
                yaw = self._sensor_data['yaw'] - horizontal_angle_offset
                pitch = - self._sensor_data['pitch'] + vertical_angle_offset
                self.set_state_approach(yaw, pitch);
        self._follow_path();


    def _generate_search_pattern(self):
        self.target_list.clear()
        
        move_angle = self._current_sector_angle()
        
        # These define the camera sweep while flying out
        stop_angle1 = move_angle + STOP_ANGLE1
        stop_angle2 = move_angle + STOP_ANGLE2
        start_angle = move_angle + START_ANGLE

        for i in range(1, NUM_TARGET_PTS + 1):
            t = i / NUM_TARGET_PTS  # goes from 1/NUM_TARGET_PTS to 1
            radius = t * self.max_sector_distance

            x = CENTER[0] + radius * np.cos(move_angle)
            y = CENTER[1] + radius * np.sin(move_angle)
            z = TARGET_Z

            # Interpolate yaw from start to stop angle
            yaw = (1 - t) * start_angle + t * stop_angle1

            self.target_list.append([x, y, z, yaw])
        for i in range(1, NUM_TARGET_PTS + 1):
            t = i / NUM_TARGET_PTS  # goes from 1/NUM_TARGET_PTS to 1
            radius = self.max_sector_distance
            angle = move_angle + t* np.radians(60)

            x = CENTER[0] + radius * np.cos(angle)
            y = CENTER[1] + radius * np.sin(angle)
            z = TARGET_Z

            # Interpolate yaw from start to stop angle
            yaw = (1 - t) * stop_angle1 + t * stop_angle2

            self.target_list.append([x, y, z, yaw])

        # Reverse the list to use .pop() in forward order
        #self.target_list.reverse()

    def _current_sector_angle(self):
        return np.radians(self.sector_index*30-15)


    def _approach(self):
        if ((self._gate_pixel_count > 0.30 * 300**2)):
            self.sector_index += 2
            self.target_list = [self.target]
            next_state = self.set_state_exit_approach
            self.set_state_move_dir(yaw=self._approach_data.yaw,
                                    pitch= 0, #self._approach_data.pitch,
                                    distance= 1.5,
                                    next_states= [next_state])
            self.gates.append(self._get_drone_position())
            return
        if ( self._gate_center is None):
            self._approach_data.missed_count+= 1
            yaw_to_gate = self._approach_data.yaw
            pitch_to_gate = self._approach_data.pitch
            if(self._approach_data.missed_count == 15):
                self.sector_index +=2
                self.set_state_search()
            
        else:
            self._approach_data.missed_count = 0
            img_height, img_width = self._camera_data.shape[:2]

            # Compute angle offset from center (in radians)
            x_pixel = self._gate_center[0]
            y_pixel = self._gate_center[0]
            horizontal_angle_offset = (x_pixel - img_width / 2) / img_width * FOV
            vertical_angle_offset =   (y_pixel - img_height / 2) / img_height * FOV

            # Calculate absolute direction to gate
            yaw_to_gate = self._sensor_data['yaw'] - horizontal_angle_offset
            pitch_to_gate = - self._sensor_data['pitch'] + vertical_angle_offset
            self._approach_data.yaw = yaw_to_gate
            self._approach_data.pitch = pitch_to_gate

        # Move 2 meters forward in that direction
        distance = 0.7
        print(pitch_to_gate)
        if (abs(self._sensor_data['yaw']- yaw_to_gate) < 0.3) and abs(pitch_to_gate < np.radians(3)):
            target_x = self._sensor_data['x_global'] + distance * np.cos(yaw_to_gate)
            target_y = self._sensor_data['y_global'] + distance * np.sin(yaw_to_gate)
        else:
            target_x = self._sensor_data['x_global']
            target_y = self._sensor_data['y_global']
        target_z = self._sensor_data['z_global'] + distance * np.sin(pitch_to_gate)
        self.target = [target_x, target_y, target_z, yaw_to_gate]


    def _distance(self, p1, p2):
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        dz = p1[2] - p2[2]
        distance = np.sqrt(dx**2 + dy**2 + dz**2)

        return distance

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

    def _detect_purple_gate(self, debug=False):
        hsv = cv2.cvtColor(self._camera_data, cv2.COLOR_BGR2HSV)

        # Threshold for purple
        mask = cv2.inRange(hsv, LOWER_PURPLE, UPPER_PURPLE)

        # Morphological filtering
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)
        self._gate_pixel_count = np.count_nonzero(mask)
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        max_area = 0
        best_box = None
        best_center = None
        best_dist_squared = 0.0

        for cnt in contours:
            if cv2.contourArea(cnt) < 500:
                continue

            rect = cv2.minAreaRect(cnt)  # (center (x,y), (width, height), angle)
            (cx, cy), (w, h), angle = rect

            if w == 0 or h == 0:
                continue

            candidate_box = cv2.boxPoints(rect).astype(int)
            candidate_center = (int(cx), int(cy))

            if self._state == "approach" and self._gate_center is not None:
                # === In approach mode: only pick the closest detection, no filters ===
                dx = candidate_center[0] - self._gate_center[0]
                dy = candidate_center[1] - self._gate_center[1]
                dist_squared = dx**2 + dy**2

                if dist_squared < best_dist_squared:
                    best_dist_squared = dist_squared
                    best_box = candidate_box
                    best_center = candidate_center
            else:
                # === In normal search mode: apply filters ===
                aspect_ratio = max(w, h) / min(w, h)
                area = w * h

                if (0.7 < aspect_ratio < 1/0.7 or area > 0.3 * 300**2) and area > max_area:
                    max_area = area
                    best_box = candidate_box
                    best_center = candidate_center

        # Debug display
        if debug:
            debug_frame = self._camera_data.copy()
            if best_box is not None:
                cv2.drawContours(debug_frame, [best_box], 0, (0, 255, 0), 2)
                cv2.drawMarker(
                    debug_frame,
                    position=best_center,
                    color=(0, 255, 0),
                    markerType=cv2.MARKER_CROSS,
                    markerSize=20,
                    thickness=2
                )
            cv2.imshow("Rotated Gate Detection", debug_frame)
            cv2.waitKey(1)

        # Update internal state
        if best_box is not None:
            self._gate_center = best_center
            self._gate_size = max_area
        else:
            self._gate_center = None
            self._gate_size = None


    def _get_drone_position(self):
        return [
            self._sensor_data["x_global"],
            self._sensor_data["y_global"],
            self._sensor_data["z_global"],
            self._sensor_data["yaw"]
        ]

