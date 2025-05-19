# from lib.a_star_3D import AStar3D
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class MotionPlanner3D():
    
    #Question: SIMON PID, what is vel_max set for PID? Check should be same here
    def __init__(self, waypoints, gates):#, bounds, grid_size):
        # Inputs:
        # - start: The sequence of input path waypoints provided by the path-planner, including the start and final goal position: Vector of m waypoints, consisting of a tuple with three reference positions each as provided by AStar 
        # - obstacles: 2D array with obstacle locations and obstacle widths [x, y, z, dx, dy, dz]*n_obs
        # - bounds: The bounds of the environment [x_min, x_max, y_min, y_max, z_min, z_max]
        # - grid_size: The grid size of the environment (scalar)
        # - goal: The final goal position of the drone (tuple of 3) 
        
        ## DO NOT MODIFY --------------------------------------------------------------------------------------- ##
        # # self.ast = AStar3D(start, goal, grid_size, obstacles, bounds)
        # # self.path = self.ast.find_path()

        self.path = waypoints # where waypoints is the list of waypoints => the gate centers including start and end points

        self.trajectory_setpoints = None

        self.gate_map = gates # position, size and orientation of the gates

        self.times = np.linspace(0, 15, len(waypoints))
        print("times", self.times)
        # self.times = self.compute_times_from_waypoints_with_acc(self.path, self.vel_lim, self.acc_lim)

        self.yaw_setpoints = [0, 0, np.pi / 2, -np.pi, -np.pi / 2, 0]
        self.disc_steps = 50

        self.init_params(self.path)

        self.run_planner(self.path) # pass empty obstacle list

        # ---------------------------------------------------------------------------------------------------- ##

    def run_planner(self, path_waypoints):    
        # Run the subsequent functions to compute the polynomial coefficients and extract and visualize the trajectory setpoints
         ## DO NOT MODIFY --------------------------------------------------------------------------------------- ##
    
        poly_coeffs = self.compute_poly_coefficients(path_waypoints)
        self.trajectory_setpoints, self.time_setpoints = self.poly_setpoint_extraction(poly_coeffs, path_waypoints)

        ## ---------------------------------------------------------------------------------------------------- ##

    def init_params(self, path_waypoints):

        # Inputs:
        # - path_waypoints: The sequence of input path waypoints provided by the path-planner, including the start and final goal position: Vector of m waypoints, consisting of a tuple with three reference positions each as provided by AStar

        # TUNE THE FOLLOWING PARAMETERS (PART 2) ----------------------------------------------------------------- ##
        self.disc_steps = 20 #Integer number steps to divide every path segment into to provide the reference positions for PID control # IDEAL: Between 10 and 20
        self.vel_lim = 7.0 #Velocity limit of the drone (m/s)
        self.acc_lim = 50.0 #Acceleration limit of the drone (m/s²)
        t_f = 2.8  # Final time at the end of the path (s)

        # Determine the number of segments of the path
        self.times = np.linspace(0, t_f, len(path_waypoints)) # The time vector at each path waypoint to traverse (Vector of size m) (must be 0 at start)

    def compute_poly_matrix(self, t):
        # Inputs:
        # - t: The time of evaluation of the A matrix (t=0 at the start of a path segment, else t >= 0) [Scalar]
        # Outputs: 
        # - The constraint matrix "A_m(t)" [5 x 6]
        # The "A_m" matrix is used to represent the system of equations [x, \dot{x}, \ddot{x}, \dddot{x}, \ddddot{x}]^T  = A_m(t) * poly_coeffs (where poly_coeffs = [c_0, c_1, c_2, c_3, c_4, c_5]^T and represents the unknown polynomial coefficients for one segment)
        A_m = np.zeros((5,6))
        
        # TASK: Fill in the constraint factor matrix values where each row corresponds to the positions, velocities, accelerations, snap and jerk here
        # SOLUTION ---------------------------------------------------------------------------------- ## 
        
        A_m = np.array([
            [t**5, t**4, t**3, t**2, t, 1], #pos
            [5*(t**4), 4*(t**3), 3*(t**2), 2*t, 1, 0], #vel
            [20*(t**3), 12*(t**2), 6*t, 2, 0, 0], #acc  
            [60*(t**2), 24*t, 6, 0, 0, 0], #jerk
            [120*t, 24, 0, 0, 0, 0] #snap
        ])

        return A_m

    def compute_poly_coefficients(self, path_waypoints):
        
        # Computes a minimum jerk trajectory given time and position waypoints.
        # Inputs:
        # - path_waypoints: The sequence of input path waypoints provided by the path-planner, including the start and final goal position: Vector of m waypoints, consisting of a tuple with three reference positions each as provided by AStar
        # Outputs:
        # - poly_coeffs: The polynomial coefficients for each segment of the path [6(m-1) x 3]

        # Use the following variables and the class function self.compute_poly_matrix(t) to solve for the polynomial coefficients
        
        seg_times = np.diff(self.times) #The time taken to complete each path segment
        m = len(path_waypoints) #Number of path waypoints (including start and end)
        poly_coeffs = np.zeros((6*(m-1),3))

        # YOUR SOLUTION HERE ---------------------------------------------------------------------------------- ## 

        # 1. Fill the entries of the constraint matrix A and equality vector b for x,y and z dimensions in the system A * poly_coeffs = b. Consider the constraints according to the lecture: We should have a total of 6*(m-1) constraints for each dimension.
        # 2. Solve for poly_coeffs given the defined system

        for dim in range(3):  # Compute for x, y, and z separately
            A = np.zeros((6*(m-1), 6*(m-1)))
            b = np.zeros(6*(m-1))
            pos = np.array([p[dim] for p in path_waypoints])
            A_0 = self.compute_poly_matrix(0) # A_0 gives the constraint factor matrix A_m for any segment at t=0, this is valid for the starting conditions at every path segment

            # SOLUTION
            row = 0
            for i in range(m-1):
                pos_0 = pos[i] #Starting position of the segment
                pos_f = pos[i+1] #Final position of the segment
                # The prescribed zero velocity (v) and acceleration (a) values at the start and goal position of the entire path
                v_0, a_0 = 0, 0
                v_f, a_f = 0, 0
                A_f = self.compute_poly_matrix(seg_times[i]) # A_f gives the constraint factor matrix A_m for a segment i at its relative end time t=seg_times[i]
                if i == 0: # First path segment
                #     # 1. Implement the initial constraints here for the first segment using A_0
                #     # 2. Implement the final position and the continuity constraints for velocity, acceleration, snap and jerk at the end of the first segment here using A_0 and A_f (check hints in the exercise description)
                    A[row, i*6:(i+1)*6] = A_0[0] #Initial position constraint
                    b[row] = pos_0
                    row += 1
                    A[row, i*6:(i+1)*6] = A_f[0] #Final position constraint
                    b[row] = pos_f
                    row += 1
                    A[row, i*6:(i+1)*6] = A_0[1] #Initial velocity constraint
                    b[row] = v_0
                    row += 1
                    A[row, i*6:(i+1)*6] = A_0[2] #Initial acceleration constraint
                    b[row] = a_0
                    row += 1
                    #Continuity of velocity, acceleration, jerk, snap
                    A[row:row+4, i*6:(i+1)*6] = A_f[1:]
                    A[row:row+4, (i+1)*6:(i+2)*6] = -A_0[1:]
                    b[row:row+4] = np.zeros(4)
                    row += 4
                elif i < m-2: # Intermediate path segments
                #     # 1. Similarly, implement the initial and final position constraints here for each intermediate path segment
                #     # 2. Similarly, implement the end of the continuity constraints for velocity, acceleration, snap and jerk at the end of each intermediate segment here using A_0 and A_f
                    A[row, i*6:(i+1)*6] = A_0[0] #Initial position constraint
                    b[row] = pos_0
                    row += 1
                    A[row, i*6:(i+1)*6] = A_f[0] #Final position constraint
                    b[row] = pos_f
                    row += 1
                    #Continuity of velocity, acceleration, jerk and snap
                    A[row:row+4, i*6:(i+1)*6] = A_f[1:]
                    A[row:row+4, (i+1)*6:(i+2)*6] = -A_0[1:]
                    b[row:row+4] = np.zeros(4)
                    row += 4
                elif i == m-2: #Final path segment
                #     # 1. Implement the initial and final position, velocity and accelerations constraints here for the final path segment using A_0 and A_f
                    A[row, i*6:(i+1)*6] = A_0[0] #Initial position constraint
                    b[row] = pos_0
                    row += 1
                    A[row, i*6:(i+1)*6] = A_f[0] #Final position constraint
                    b[row] = pos_f
                    row += 1
                    A[row, i*6:(i+1)*6] = A_f[1] #Final velocity constraint
                    b[row] = v_f
                    row += 1
                    A[row, i*6:(i+1)*6] = A_f[2] #Final acceleration constraint
                    b[row] = a_f
                    row += 1
            # Solve for the polynomial coefficients for the dimension dim

            poly_coeffs[:,dim] = np.linalg.solve(A, b)   

        return poly_coeffs

    def poly_setpoint_extraction(self, poly_coeffs, path_waypoints):

        # DO NOT MODIFY --------------------------------------------------------------------------------------- ##

        # Uses the class features: self.disc_steps, self.times, self.poly_coeffs, self.vel_lim, self.acc_lim
        x_vals, y_vals, z_vals = np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1))
        v_x_vals, v_y_vals, v_z_vals = np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1))
        a_x_vals, a_y_vals, a_z_vals = np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1))

        # Define the time reference in self.disc_steps number of segements
        time_setpoints = np.linspace(self.times[0], self.times[-1], self.disc_steps*len(self.times))  # Fine time intervals

        # Extract the x,y and z direction polynomial coefficient vectors
        coeff_x = poly_coeffs[:,0]
        coeff_y = poly_coeffs[:,1]
        coeff_z = poly_coeffs[:,2]

        for i,t in enumerate(time_setpoints):
            seg_idx = min(max(np.searchsorted(self.times, t)-1,0), len(coeff_x) - 1)
            # max_seg = len(coeff_x) // 6 - 1  # number of segments
            # seg_idx = min(max(np.searchsorted(self.times, t)-1, 0), max_seg)

            # print(f"i = {i}, t = {t}, seg_idx = {seg_idx}, coeff range = {seg_idx*6}:{(seg_idx+1)*6}, coeff_x size = {coeff_x.shape}")
            # Determine the x,y and z position reference points at every refernce time
            x_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[0],coeff_x[seg_idx*6:(seg_idx+1)*6])
            y_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[0],coeff_y[seg_idx*6:(seg_idx+1)*6])
            z_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[0],coeff_z[seg_idx*6:(seg_idx+1)*6])
            # Determine the x,y and z velocities at every reference time
            v_x_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[1],coeff_x[seg_idx*6:(seg_idx+1)*6])
            v_y_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[1],coeff_y[seg_idx*6:(seg_idx+1)*6])
            v_z_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[1],coeff_z[seg_idx*6:(seg_idx+1)*6])
            # Determine the x,y and z accelerations at every reference time
            a_x_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[2],coeff_x[seg_idx*6:(seg_idx+1)*6])
            a_y_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[2],coeff_y[seg_idx*6:(seg_idx+1)*6])
            a_z_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[2],coeff_z[seg_idx*6:(seg_idx+1)*6])

        yaw_vals = np.zeros((self.disc_steps*len(self.times),1))
        # compute yaw angle waypoint

        def angle_lerp(a0, a1, t):
            diff = (a1 - a0 + np.pi) % (2 * np.pi) - np.pi
            return a0 + diff * t

        yaw_waypoints = np.unwrap(self.yaw_setpoints)
        num_segments = len(yaw_waypoints) - 1
        segment_time = self.times[-1] / num_segments
        print(yaw_waypoints)

        yaw_vals = np.zeros_like(time_setpoints)
        for i, t in enumerate(time_setpoints):
            seg_idx = min(int(t // segment_time), num_segments - 1)
            t0 = seg_idx * segment_time
            alpha = (t - t0) / segment_time
            yaw_vals[i] = angle_lerp(yaw_waypoints[seg_idx], yaw_waypoints[seg_idx + 1], alpha)
        yaw_vals = np.array(yaw_vals).reshape(-1, 1)


        trajectory_setpoints = np.hstack((x_vals, y_vals, z_vals, yaw_vals))

        self.plot(path_waypoints, trajectory_setpoints)
            
        # Find the maximum absolute velocity during the segment
        vel_max = np.max(np.sqrt(v_x_vals**2 + v_y_vals**2 + v_z_vals**2))
        vel_mean = np.mean(np.sqrt(v_x_vals**2 + v_y_vals**2 + v_z_vals**2))
        acc_max = np.max(np.sqrt(a_x_vals**2 + a_y_vals**2 + a_z_vals**2))
        acc_mean = np.mean(np.sqrt(a_x_vals**2 + a_y_vals**2 + a_z_vals**2))

        print("Maximum flight speed: " + str(vel_max))
        print("Average flight speed: " + str(vel_mean))
        print("Average flight acceleration: " + str(acc_mean))
        print("Maximum flight acceleration: " + str(acc_max))
        
        # Check that it is less than an upper limit velocity v_lim
        assert vel_max <= self.vel_lim, "The drone velocity exceeds the limit velocity : " + str(vel_max) + " m/s"
        assert acc_max <= self.acc_lim, "The drone acceleration exceeds the limit acceleration : " + str(acc_max) + " m/s²"

        # ---------------------------------------------------------------------------------------------------- ##

        return trajectory_setpoints, time_setpoints

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


    def plot_gates(self, ax, gate, color='gray', alpha=0.3):
        """Plot a rectangular cuboid (obstacle) in 3D space."""
        x, y, z, dtheta, dyz = gate[0], gate[1], gate[2], gate[3], gate[4]

        # print("Gate position: ", x, y, z, dyz, dtheta)
        # Convert rotation angle from degrees to radians
        theta_rad = np.radians(dtheta+90)
        # Rotation matrix around Z-axis
        Rz = np.array([[np.cos(theta_rad), -np.sin(theta_rad), 0],
                       [np.sin(theta_rad),  np.cos(theta_rad), 0],
                       [0,                 0,                  1]])

        dx = 0.1

        # Compute corners relative to center
        dx2, dy2, dz2 = dx/2, dyz/2, dyz/2

        corners = np.array([[-dx2, -dy2, -dz2],
                            [ dx2, -dy2, -dz2],
                            [ dx2,  dy2, -dz2],
                            [-dx2,  dy2, -dz2],

                            [-dx2, -dy2,  dz2],
                            [ dx2, -dy2,  dz2],
                            [ dx2,  dy2,  dz2],
                            [-dx2,  dy2,  dz2]])
            # Apply rotation and translation
        # rotated_corners =  np.dot(corners, Rz.T) * np.array([1, dyz/2*10, dyz/2*10]) + np.array([x, y, z])
        rotated_corners =  np.dot(corners, Rz.T) + np.array([x, y, z])

        # Define 6 faces of the cuboid
        faces = [[rotated_corners[j] for j in [0, 1, 2, 3]],  # bottom
                 [rotated_corners[j] for j in [4, 5, 6, 7]],  # top
                 [rotated_corners[j] for j in [0, 1, 5, 4]],  # side 1
                 [rotated_corners[j] for j in [2, 3, 7, 6]],  # side 2
                 [rotated_corners[j] for j in [0, 3, 7, 4]],  # front
                 [rotated_corners[j] for j in [1, 2, 6, 5]]]  # back
 
        ax.add_collection3d(Poly3DCollection(faces, color=color, alpha=alpha))

    def plot(self, path_waypoints, trajectory_setpoints):

        start = np.array([0, 0, 0.5, 0, 0.6])
        end = np.array([0, 0, 0.5, 0, 0.6])

        path_waypoints = (start[:3], path_waypoints[0][:3], path_waypoints[1][:3], path_waypoints[2][:3], path_waypoints[3][:3], end[:3])

        # Plot 3D trajectory
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection='3d')

        for gates in self.gate_map:
            self.plot_gates(ax, gates)

        ax.plot(trajectory_setpoints[:,0], trajectory_setpoints[:,1], trajectory_setpoints[:,2], label="Minimum-Jerk Trajectory", linewidth=2)
        ax.set_xlim(0, 4.15) # meter
        ax.set_ylim(0, 2.55) # meter
        ax.set_zlim(0, 2) # meter

        # Plot waypoints
        waypoints_x = [p[0] for p in path_waypoints]
        waypoints_y = [p[1] for p in path_waypoints]
        waypoints_z = [p[2] for p in path_waypoints]
        ax.scatter(waypoints_x, waypoints_y, waypoints_z, color='red', marker='o', label="Waypoints")

        # Plot interpolated trajectory setpoints as small dots
        ax.scatter(
            trajectory_setpoints[:, 0],  # x
            trajectory_setpoints[:, 1],  # y
            trajectory_setpoints[:, 2],  # z
            color='green', s=10, alpha=0.6, label="Interpolated Waypoints"
        )

        # Labels and legend
        ax.set_xlabel("X Position")
        ax.set_ylabel("Y Position")
        ax.set_zlabel("Z Position")
        ax.set_title("3D Motion planning trajectories")
        ax.legend()
        plt.show()

# def poly_setpoint_extraction(self, poly_coeffs,yaw_setpoints, times, disc_steps):# path_waypoints):

#         # DO NOT MODIFY --------------------------------------------------------------------------------------- ##

#         # Uses the class features: self.disc_steps, self.times, self.poly_coeffs, self.vel_lim, self.acc_lim
#         x_vals, y_vals, z_vals = np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1))
#         v_x_vals, v_y_vals, v_z_vals = np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1))
#         a_x_vals, a_y_vals, a_z_vals = np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1))

#         # Define the time reference in self.disc_steps number of segements
#         time_setpoints = np.linspace(self.times[0], self.times[-1], self.disc_steps*len(self.times))  # Fine time intervals

#         # Extract the x,y and z direction polynomial coefficient vectors
#         coeff_x = poly_coeffs[:,0]
#         coeff_y = poly_coeffs[:,1]
#         coeff_z = poly_coeffs[:,2]

#         for i,t in enumerate(time_setpoints):
#             seg_idx = min(max(np.searchsorted(self.times, t)-1,0), len(coeff_x) - 1)
#             # Determine the x,y and z position reference points at every refernce time
#             x_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[0],coeff_x[seg_idx*6:(seg_idx+1)*6])
#             y_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[0],coeff_y[seg_idx*6:(seg_idx+1)*6])
#             z_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[0],coeff_z[seg_idx*6:(seg_idx+1)*6])
#             # Determine the x,y and z velocities at every reference time
#             v_x_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[1],coeff_x[seg_idx*6:(seg_idx+1)*6])
#             v_y_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[1],coeff_y[seg_idx*6:(seg_idx+1)*6])
#             v_z_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[1],coeff_z[seg_idx*6:(seg_idx+1)*6])
#             # Determine the x,y and z accelerations at every reference time
#             a_x_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[2],coeff_x[seg_idx*6:(seg_idx+1)*6])
#             a_y_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[2],coeff_y[seg_idx*6:(seg_idx+1)*6])
#             a_z_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[2],coeff_z[seg_idx*6:(seg_idx+1)*6])

#         yaw_vals = np.zeros((self.disc_steps*len(self.times),1))
#         trajectory_setpoints = np.hstack((x_vals, y_vals, z_vals, yaw_vals))

#         self.plot(path_waypoints, trajectory_setpoints)
            
#         # Find the maximum absolute velocity during the segment
#         vel_max = np.max(np.sqrt(v_x_vals**2 + v_y_vals**2 + v_z_vals**2))
#         vel_mean = np.mean(np.sqrt(v_x_vals**2 + v_y_vals**2 + v_z_vals**2))
#         acc_max = np.max(np.sqrt(a_x_vals**2 + a_y_vals**2 + a_z_vals**2))
#         acc_mean = np.mean(np.sqrt(a_x_vals**2 + a_y_vals**2 + a_z_vals**2))

#         print("Maximum flight speed: " + str(vel_max))
#         print("Average flight speed: " + str(vel_mean))
#         print("Average flight acceleration: " + str(acc_mean))
#         print("Maximum flight acceleration: " + str(acc_max))
        
#         # Check that it is less than an upper limit velocity v_lim
#         assert vel_max <= self.vel_lim, "The drone velocity exceeds the limit velocity : " + str(vel_max) + " m/s"
#         assert acc_max <= self.acc_lim, "The drone acceleration exceeds the limit acceleration : " + str(acc_max) + " m/s²"

#         # ---------------------------------------------------------------------------------------------------- ##

#         return trajectory_setpoints, time_setpoints