from my_utils import get_quat_from_velocity
import numpy as np
from scipy.optimize import minimize
from scipy.optimize import differential_evolution
import robotic as ry
from my_utils import get_quat_from_velocity
def find_initial_point_from_release(C:ry.Config,release_position, release_velocity, height_threshold=0.2):
    """
    Finds the initial position for the throwing motion based on the release position and velocity.


    Args:
        C: Robot configuration (KOMO simulation object).
        release_position: The release point (3D coordinates).
        release_velocity: The release velocity vector.
        height_threshold: Minimum allowable height for the initial position.


    Returns:
        initial_position: The computed initial position, if found.
    """
    with open("throwing_params.txt","r") as f:
        distance_xy = f.read().strip()
        print(distance_xy)
        try:
            distance_xy = np.float64(distance_xy)
            print("Successfully converted:", distance_xy)
        except ValueError as e:
            print("Conversion failed:", e)
    velxy_ratio = release_velocity[1]/release_velocity[0]
    x = distance_xy/np.sqrt(1+np.square(velxy_ratio))
    base_position = C.getFrame("l_panda_base").getPosition()
    initial_position  = np.array([-x +base_position[0],-np.multiply(velxy_ratio,x) + base_position[1],0.3])
    iteration = 0
    gravity = np.array([0,0,-9.8])
    iteration = 0
    #resetting the velocity and position after calculating initial position
    velocity = np.array(release_velocity)
    time_step = .1
    position = np.array(release_position)
    while position[2] > height_threshold:
        position += velocity * time_step
        velocity += gravity * time_step
        iteration += 1
        frame_quat = get_quat_from_velocity(velocity)
        frame = C.addFrame(f"trajectory-{iteration}").setShape(ry.ST.marker,[.2]).setColor([0,0,1]).setPosition(position).setQuaternion(frame_quat)
        C.view()
        print(f"Frame {iteration} added with position {frame.getPosition()} new vel:{velocity}")
    return initial_position  # No valid initial position found



def pick_last_object_if_valid(C:ry.Config,release_position, release_velocity):
   """
   Simulates the robot to pick the last object if conditions are met.
   Args:
       C: Robot configuration (KOMO simulation object).
       release_position: The release point (3D coordinates).
       release_velocity: The release velocity vector.
       robot_base: Position of the robot base.
   """
   # Find the initial position
   initial_position = find_initial_point_from_release(C,release_position, release_velocity)

   if initial_position is not None:
       print(f"Initial Position Found: {initial_position}")
       # Visualize the initial position
    #    C.addFrame("initial_position").setPosition(initial_position).setShape(ry.ST.marker, [0.4]).setContact(0).setColor([0,1,0]).setQuaternion(rotation_matrix_to_quaternion(velocity_to_rotation_matrix(release_velocity)))
       C.addFrame("initial_position").setPosition(initial_position).setShape(ry.ST.marker, [0.4]).setContact(0).setColor([0,1,0])
       C.view()
       return initial_position
       # Simulate the robot to pick the last object
       # Add your grasping or motion planning logic here
   else:
       print("No valid initial position found.")



def find_velocity(C:ry.Config):
    g = 9.81
    initial_pos = C.getFrame("throw").getPosition()
    bin_dims = C.getFrame("bin").getPosition()

    # Bin and robot parameters
    x_bin, y_bin, z_bin = bin_dims[0], bin_dims[1], bin_dims[2]
    x0, y0, z0 = initial_pos[0], initial_pos[1], initial_pos[2]

    # Objective function: minimize velocity magnitude
    def objective(params):
        v0, theta, phi = params
        return v0

    # Constraints: ball lands in the bin's X position
    def constraint_x(params):
        g=9.81
        v0, theta, phi = params
        vx = v0 * np.cos(theta) * np.cos(phi)
        t_land = (-v0 * np.sin(theta) + np.sqrt((v0 * np.sin(theta))**2 + 2 * g * (z0 - z_bin))) / g
        return x0 + vx * t_land - x_bin

    # Constraints: ball lands in the bin's Y position
    def constraint_y(params):
        v0, theta, phi = params
        vy = v0 * np.cos(theta) * np.sin(phi)
        t_land = (-v0 * np.sin(theta) + np.sqrt((v0 * np.sin(theta))**2 + 2 * g * (z0 - z_bin))) / g
        return y0 + vy * t_land - y_bin

    # Constraints: ball lands in the bin's Z position (height)
    def constraint_z(params):
        v0, theta, phi = params
        vz = v0 * np.sin(theta)
        t_land = (-vz + np.sqrt(vz**2 + 2 * g * (z0 - z_bin))) / g
        return z0 + vz * t_land - 0.5 * g * t_land**2 - z_bin

    # Constraint: Ball does not hit the bin's sides
    def constraint_sides_height(params):
        v0, theta, phi = params
        # Time to reach maximum height
        t_up = v0 * np.sin(theta) / g
        # Ball's height at time t_up (maximum height)
        z_max = z0 + (v0 * np.sin(theta))**2 / (2 * g)
        
        # If the bin's sides are higher than the max height, no problem
        if z_max < z_bin:
            return 0  # No issue, the ball won't hit the sides
        else:
            # If the bin's sides are lower, we need to check at different points in time
            t_down = np.sqrt(2 * (z_max - z_bin) / g)
            z_ball_at_t = z0 + v0 * np.sin(theta) * t_down - 0.5 * g * t_down**2
            return z_ball_at_t - z_bin  # Ball height should not exceed bin sides
        # Initial guess and bounds
    initial_guess = [5, np.pi / 4, np.pi / 4]  # [velocity, theta, phi]
    bounds = [(1, 20), (0, np.pi / 2), (0, 2 * np.pi)]  # Bounds for velocity and angles
    # Solve the optimization
    result = minimize(objective, initial_guess, constraints=[
        {'type': 'eq', 'fun': constraint_x},
        {'type': 'eq', 'fun': constraint_y},
        {'type': 'eq', 'fun': constraint_z},
        {'type': 'ineq', 'fun': constraint_sides_height}  # Ensure ball avoids sides' height
    ], bounds=bounds)

    # Extract results
    v0_opt, theta_opt, phi_opt = result.x
    print(f"Optimal velocity: {v0_opt:.2f} m/s, Theta: {np.degrees(theta_opt):.2f}°, Phi: {np.degrees(phi_opt):.2f}°")

    v_x = v0_opt * np.cos(theta_opt) * np.cos(phi_opt)
    v_y = v0_opt * np.cos(theta_opt) * np.sin(phi_opt)
    v_z = v0_opt * np.sin(theta_opt)
    velocity = [v_x, v_y, v_z]
    rf = C.addFrame("release_frame").setPosition(initial_pos).setShape(ry.ST.marker,[.4]).setColor([1,0,0]).setContact(0)
    new_quat = get_quat_from_velocity(velocity)
    rf.setQuaternion(new_quat)
    return velocity