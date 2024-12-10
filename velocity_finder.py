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
    velxy_ratio = np.abs(release_velocity[1]/release_velocity[0])
    x = distance_xy/np.sqrt(1+np.square(velxy_ratio))
    base_position = C.getFrame("l_panda_base").getPosition()
    sign_velx = release_velocity[0]/np.abs(release_velocity[0])
    sign_vely = release_velocity[1]/np.abs(release_velocity[1])
    initial_position  = np.array([-(sign_velx)*x +base_position[0],-sign_vely*np.multiply(velxy_ratio,x) + base_position[1],0.3])
    iteration = 0
    gravity = np.array([0,0,-9.8])
    iteration = 0
    #resetting the velocity and position after calculating initial position
    velocity = np.array(release_velocity)
    time_step = .06
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


def find_velocity(C: ry.Config):
    g = 9.81
    initial_pos = C.getFrame("throw").getPosition()
    bin_dims = C.getFrame("bin").getPosition()
    #currently the constraint cannot optimize the left of the robot. Hence
    isInverted = False
    if bin_dims[1] < initial_pos[1]:
        y_distance = initial_pos[1] - bin_dims[1]
        bin_dims[1] += 2*y_distance
        isInverted = True
    x_bin, y_bin, z_bin = bin_dims[0], bin_dims[1], bin_dims[2]
    x0, y0, z0 = initial_pos[0], initial_pos[1], initial_pos[2]

    print(f"Initial Position: {initial_pos}")
    print(f"Bin Position: {bin_dims}")

    def objective(params):
        v0, theta, phi = params
        penalty_x = np.abs(constraint_x(params))
        penalty_y = np.abs(constraint_y(params))
        penalty_z = np.abs(constraint_z(params))
        return v0 + 100 * (penalty_x + penalty_y + penalty_z)


    def constraint_x(params):
        v0, theta, phi = params
        vx = v0 * np.cos(theta) * np.cos(phi)
        t_land = (-v0 * np.sin(theta) + np.sqrt((v0 * np.sin(theta))**2 + 2 * g * (z0 - z_bin))) / g
        x_land = x0 + vx * t_land
        print(f"constraint_x: x_land={x_land}, x_bin={x_bin}")
        return x_land - x_bin

    def constraint_y(params):
        v0, theta, phi = params
        vy = v0 * np.cos(theta) * np.sin(phi)
        t_land = (-v0 * np.sin(theta) + np.sqrt((v0 * np.sin(theta))**2 + 2 * g * (z0 - z_bin))) / g
        y_land = y0 + vy * t_land
        print(f"constraint_y: y_land={y_land}, y_bin={y_bin}")
        return y_land - y_bin

    def constraint_z(params):
        v0, theta, phi = params
        vz = v0 * np.sin(theta)
        t_land = (-vz + np.sqrt(vz**2 + 2 * g * (z0 - z_bin))) / g
        z_land = z0 + vz * t_land - 0.5 * g * t_land**2
        print(f"constraint_z: z_land={z_land}, z_bin={z_bin}")
        return z_land - z_bin

    base_position = C.getFrame("l_panda_base").getPosition()
    bin_rel_position = bin_dims - base_position
    bin_rel_position[2] = 0
    bin_rel_position_normalized = bin_rel_position/np.linalg.norm(bin_rel_position)
    initial_rel_position = np.array([1,0,0])
    phi_guess = np.arccos(np.dot(bin_rel_position_normalized,initial_rel_position))
    
    # initial_guess = [5, np.pi / 4, np.pi / 4] 
    initial_guess = [5, np.pi / 4, phi_guess] 
    # I made it dynamically to converge faster.

    bounds = [(1, 50), (0, np.pi / 2), (0, 2 * np.pi)]
    result = minimize(objective, initial_guess, constraints=[
        {'type': 'eq', 'fun': constraint_x},
        {'type': 'eq', 'fun': constraint_y},
        {'type': 'eq', 'fun': constraint_z}
    ], bounds=bounds, options={'maxiter': 1000})

    if not result.success:
        print(f"Optimization failed: {result.message}")
        return None
    
    v0_opt, theta_opt, phi_opt = result.x
    print(f"Optimal velocity: {v0_opt}, theta: {np.degrees(theta_opt)}, phi: {np.degrees(phi_opt)}")
    v_x = v0_opt * np.cos(theta_opt) * np.cos(phi_opt)
    v_y = v0_opt * np.cos(theta_opt) * np.sin(phi_opt)
    v_z = v0_opt * np.sin(theta_opt)
    velocity = [v_x, v_y if not isInverted else -v_y, v_z]
    rf = C.addFrame("release_frame").setPosition(initial_pos).setShape(ry.ST.marker,[.4]).setColor([1,0,0]).setContact(0)
    new_quat = get_quat_from_velocity(velocity)
    rf.setQuaternion(new_quat)
    return velocity, isInverted
