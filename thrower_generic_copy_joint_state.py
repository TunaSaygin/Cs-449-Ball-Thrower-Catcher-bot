import robotic as ry
import numpy as np
from my_utils import rotation_matrix_to_quaternion, get_quat_from_velocity
import time
from velocity_finder import find_velocity, pick_last_object_if_valid
from scipy.spatial import cKDTree
import json


def throw_sample(bin_new_position: list, isRender: bool, sleep_time: float = 20, bin_shape=[0.5, 0.5]):
    """
    Simulates the process of throwing an object into a bin and evaluates the results.

    Args:
        bin_new_position (list): The target position of the bin [x, y, z].
        isRender (bool): Whether to render the simulation.
        sleep_time (float): Time to pause the rendering after simulation (default: 20 seconds).
        bin_shape (list): The shape of the bin [length, width] (default: [0.5, 0.5]).

    Returns:
        tuple: Contains the result (bool), deviation from the bin (float), and trajectory deviations (float).
    """
    C = ry.Config()
    C.addFile("throwing_bare.g")
    print(f"Initial bin position: {C.getFrame('bin').getPosition()}")

    # Initialize the simulation environment
    init_environment(C, bin_new_position, bin_shape)
    bot = ry.BotOp(C, useRealRobot=False)

    # Estimate trajectory and release velocity
    estimated_trajectory_array = []
    release_velocity, isInverted = find_velocity(C)
    pick_last_object_if_valid(C, C.getFrame("release_frame").getPosition(), release_velocity, estimated_trajectory_array)

    # Perform the grasping operation
    time_deviation = grasp_object(C, bot)

    # Calculate the release timing based on target distance
    release_position = C.getFrame("release_frame").getPosition()
    target_distance = np.linalg.norm(release_position[:2] - bin_new_position[:2])
    time_delay = 0.07 if target_distance <= 1.5 else (0.03 if target_distance <= 2 else 0.02)
    wanted_sleep = 0.57 + time_delay
    time_sleep = time_deviation * wanted_sleep

    print(f"Starting throw with release velocity: {release_velocity}, target distance: {target_distance}")
    ball_trajectory_array = []
    cargo_height = C.getFrame("cargo").getSize()[2]
    throw_object(C, bot, time_sleep, release_velocity, ball_trajectory_array)

    # Evaluate if the object landed in the bin
    result, deviation = check_in_the_bin(
        C, bot, bin_new_position, C.getFrame("side2").getSize()[0] / 2,
        C.getFrame("side2").getSize()[2], cargo_height, ball_trajectory_array
    )

    # Render the trajectories
    render_actual_trajectory(C, 1, ball_trajectory_array)
    trajectory_deviations, interpolated_trajectory, _ = calculate_deviation(
        ball_trajectory_array, estimated_trajectory_array, bin_new_position[2]
    )
    render_actual_trajectory(C, 2, interpolated_trajectory, [1, 1, 0])

    print(f"Trajectory deviation: {trajectory_deviations}")
    if isRender:
        C.view()
        time.sleep(sleep_time)

    del C
    del bot
    return result, deviation, trajectory_deviations


def render_actual_trajectory(C: ry.Config, id: int, trajectory: list, color: list = [0.6, 0.2, 0.3]):
    """
    Renders the trajectory of the ball in the simulation.

    Args:
        C (ry.Config): Configuration object for the simulation.
        id (int): Unique identifier for the trajectory.
        trajectory (list): List of trajectory points.
        color (list): RGB color for trajectory markers (default: [0.6, 0.2, 0.3]).
    """
    for idx, point in enumerate(trajectory):
        C.addFrame(f"point-{id}-{idx}").setShape(ry.ST.marker, [0.3]).setPosition(point).setColor(color)


def calculate_deviation(position_red, position_blue, height_threshold):
    """
    Calculates the deviation between actual and estimated trajectories.

    Args:
        position_red (list): Points of the actual trajectory [x, y, z].
        position_blue (list): Points of the estimated trajectory [x, y, z].
        height_threshold (float): Minimum height for filtering points.

    Returns:
        tuple: RMS deviation, filtered actual trajectory, and matched estimated trajectory.
    """
    def filter_points(array, threshold):
        """Filter points based on the height threshold."""
        return np.array([point for point in array if point[2] >= threshold])

    position_red = filter_points(position_red, height_threshold)
    position_blue = filter_points(position_blue, height_threshold)

    if len(position_red) == 0 or len(position_blue) == 0:
        raise ValueError("No points above the height threshold.")

    kdtree_blue = cKDTree(position_blue)
    distances, indices = kdtree_blue.query(position_red)

    matched_red = position_red
    matched_blue = position_blue[indices]
    deviations = np.linalg.norm(matched_red - matched_blue, axis=1)
    rms_deviation = np.sqrt(np.mean(np.square(deviations)))

    return rms_deviation, matched_red, matched_blue


def check_in_the_bin(C: ry.Config, bot: ry.BotOp, bin_center, binxy_length, bin_height, cargo_height, trajectory):
    """
    Checks whether the object lands in the bin.

    Args:
        C (ry.Config): Configuration object for the simulation.
        bot (ry.BotOp): Robot operation interface.
        bin_center (list): Center of the bin [x, y, z].
        binxy_length (float): Half-length of the bin in x and y directions.
        bin_height (float): Height of the bin.
        cargo_height (float): Height of the cargo object.
        trajectory (list): List of trajectory points.

    Returns:
        tuple: Result (bool) and deviation from the bin center (float).
    """
    start = time.time()
    prev_pos = np.array(C.getFrame("cargo").getPosition())
    bot.sync(C, 0.01)
    cargo_pos = np.array(C.getFrame("cargo").getPosition())

    while not np.allclose(prev_pos, cargo_pos) or (prev_pos[2] >= cargo_height and cargo_pos[2] >= cargo_height):
        prev_pos = cargo_pos
        bot.sync(C, 0.03)
        trajectory.append(cargo_pos)
        cargo_pos = np.array(C.getFrame("cargo").getPosition())
        if time.time() - start > 8:
            break

    deviation = np.linalg.norm(cargo_pos - np.array(bin_center))
    success = (
        bin_center[0] - binxy_length <= cargo_pos[0] <= bin_center[0] + binxy_length and
        bin_center[1] - binxy_length <= cargo_pos[1] <= bin_center[1] + binxy_length and
        bin_center[2] <= cargo_pos[2] <= bin_center[2] + bin_height
    )
    return success, deviation


def throw_object(C, bot, time_sleep, velocity, trajectory):
    """
    Executes the throw operation.

    Args:
        C (ry.Config): Configuration object for the simulation.
        bot (ry.BotOp): Robot operation interface.
        time_sleep (float): Time to sleep during the throw.
        velocity (list): Release velocity [vx, vy, vz].
        trajectory (list): List to store trajectory points.
    """
    cargo_frame = C.getFrame("cargo")

    def vel_komo():
        komo = ry.KOMO(C, 1, 1, 1, True)
        komo.addObjective([], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e-1], velocity, 1)
        komo.addObjective([], ry.FS.positionDiff, ["l_gripper", "release_frame"], ry.OT.sos, [1e0], [0, 0, 0])
        ry.NLP_Solver(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=0).solve()
        return komo

    komo = vel_komo()
    path = komo.getPath()
    bot.move(path, [1.0])
    time.sleep(time_sleep)
    bot.gripperMove(ry._left, width=1)
    while bot.getTimeToEnd() > 0:
        trajectory.append(cargo_frame.getPosition())
        bot.sync(C, 0.03)

def grasp_object(C: ry.Config, bot: ry.BotOp, object_name: str = "cargo"):
    """
    Executes the grasping operation for the specified object.

    Args:
        C (ry.Config): Configuration object for the simulation.
        bot (ry.BotOp): Robot operation interface.
        object_name (str): Name of the object to be grasped (default: "cargo").

    Returns:
        float: Time deviation for grasping.
    """
    q0 = qHome = C.getJointState()
    komo_pre_grasp = pre_grasp_komo(C, "l_gripper", object_name, q0, qHome)
    path_pre_grasp = komo_pre_grasp.getPath()
    komo_post_grasp = post_grasp_komo(C, qHome)
    path_post_grasp = komo_post_grasp.getPath()

    shape = path_pre_grasp.shape[0] * 0.1
    bot.move(path_pre_grasp, [1.0])
    start = bot.getTimeToEnd()
    time.sleep(shape)
    end = bot.getTimeToEnd()

    while bot.getTimeToEnd() > 0:
        bot.sync(C, 0.1)

    bot.gripperClose(ry._left, width=0.06)
    while not bot.gripperDone(ry._left):
        bot.sync(C, 0.1)

    bot.move(path_post_grasp, [3.0])
    while bot.getTimeToEnd() > 0:
        bot.sync(C, 0.1)

    C.attach('l_gripper', object_name)
    time_deviation = shape / (start - end)
    return time_deviation


def pre_grasp_komo(C, gripper_name, grasp_frame_name, q0, qHome):
    """
    Configures the pre-grasp trajectory for the robot.

    Args:
        C (ry.Config): Configuration object for the simulation.
        gripper_name (str): Name of the gripper frame.
        grasp_frame_name (str): Name of the object frame to be grasped.
        q0 (list): Initial joint state.
        qHome (list): Home joint state.

    Returns:
        ry.KOMO: KOMO object with pre-grasp trajectory.
    """
    komo = ry.KOMO(C, 3, 1, 0, True)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, [1e1])
    komo.addObjective([], ry.FS.jointState, [], ry.OT.sos, [1e-1], q0)
    komo.addObjective([], ry.FS.jointState, [], ry.OT.sos, [1e-1], qHome)
    komo.addObjective([1.0, 3.0], ry.FS.positionRel, [gripper_name, grasp_frame_name], ry.OT.eq, [1e1], [0, 0, 0])
    komo.addObjective([1.0, 3.0], ry.FS.scalarProductYX, [gripper_name, grasp_frame_name], ry.OT.eq, [1e2], [1])
    komo.addObjective([1.0, 3.0], ry.FS.scalarProductZZ, [grasp_frame_name, gripper_name], ry.OT.eq, [1e1], [1])
    ry.NLP_Solver(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=0).solve()
    return komo


def post_grasp_komo(C: ry.Config, home):
    """
    Configures the post-grasp trajectory for the robot.

    Args:
        C (ry.Config): Configuration object for the simulation.
        home (list): Home joint state.

    Returns:
        ry.KOMO: KOMO object with post-grasp trajectory.
    """
    komo = ry.KOMO(C, 1, 1, 0, True)
    z_vector = np.array([0, 0, 1])
    robot_base = np.array(C.getFrame("l_panda_base").getPosition())
    initial_position = np.array(C.getFrame("initial_position").getPosition()) - robot_base

    komo.addObjective([], ry.FS.scalarProductXZ, ["initial_position", "l_panda_coll5"], ry.OT.eq, [1e1], [-1])
    komo.addObjective([], ry.FS.scalarProductXX, ["initial_position", "l_panda_coll7"], ry.OT.eq, [1e1], [1])
    ry.NLP_Solver(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=0).solve()
    return komo


def init_environment(C: ry.Config, bin_new_position: list, bin_shape):
    """
    Initializes the simulation environment with the specified bin position and shape.

    Args:
        C (ry.Config): Configuration object for the simulation.
        bin_new_position (list): New position of the bin [x, y, z].
        bin_shape (list): Shape of the bin [length, width].
    """
    if bin_new_position[2] < 0.08:
        bin_new_position[2] = 0.08  # Ensure bin is above ground.

    C.getFrame('cargo').unLink()
    base_position = np.array(C.getFrame("l_panda_base").getPosition())
    new_quat = rotate_bin(bin_new_position, base_position)

    C.addFrame("throw").setPosition([0, 1.8, 1.2]).setShape(ry.ST.marker, [0.3]).setQuaternion(new_quat)
    C.addFrame("release_frame").setPosition([0, 0, 0]).setShape(ry.ST.marker, [0.4]).setColor([1, 0, 0]).setContact(0)

    if bin_new_position[2] > 0.08:
        height = bin_new_position[2] - 0.025
        C.addFrame("bin-support") \
            .setPosition([bin_new_position[0], bin_new_position[1], height / 2]) \
            .setShape(ry.ST.ssBox, [bin_shape[0] + 0.001, bin_shape[1] + 0.001, height, 0]) \
            .setQuaternion(new_quat).setColor([0, 0, 0])

    C.getFrame('bin').setPosition(bin_new_position).setQuaternion(new_quat)
    C.view()


def rotate_bin(bin_position: np.ndarray, base_position: np.ndarray):
    """
    Calculates the rotation quaternion for aligning the bin to the base.

    Args:
        bin_position (np.ndarray): Position of the bin [x, y, z].
        base_position (np.ndarray): Position of the robot base [x, y, z].

    Returns:
        np.ndarray: Quaternion representing the rotation.
    """
    vector_base_bin = bin_position - base_position
    xy_distance = np.linalg.norm(vector_base_bin[:2])
    bin_x_vector = vector_base_bin[:2] / xy_distance
    bin_z_vector = np.array([0, 0, 1])
    bin_y_vector = np.cross(bin_z_vector, bin_x_vector)

    rotation_matrix = np.column_stack((bin_x_vector, bin_y_vector, bin_z_vector))
    new_quaternion = rotation_matrix_to_quaternion(rotation_matrix)
    return new_quaternion


def generate_homogeneous_points(robo_base, carpet_center, carpet_len, range_limit, z_min, z_max, num_points, grid_resolution=10):
    """
    Generates a grid of random 3D points within a specified range.

    Args:
        robo_base (list): Position of the robot base [x, y, z].
        carpet_center (list): Center of the restricted carpet area [x, y].
        carpet_len (float): Length of the carpet area.
        range_limit (float): Range around the robot base for generating points.
        z_min (float): Minimum height (z-coordinate) of points.
        z_max (float): Maximum height (z-coordinate) of points.
        num_points (int): Number of points to generate.
        grid_resolution (int): Resolution of the grid.

    Returns:
        list: List of generated points [x, y, z].
    """
    points = []
    x_min = robo_base[0] - range_limit
    x_max = robo_base[0] + range_limit
    y_min = robo_base[1] - range_limit
    y_max = robo_base[1] + range_limit

    x_bins = np.linspace(x_min, x_max, grid_resolution)
    y_bins = np.linspace(y_min, y_max, grid_resolution)

    for i in range(len(x_bins) - 1):
        for j in range(len(y_bins) - 1):
            for _ in range(num_points // ((grid_resolution - 1) ** 2)):
                x = np.random.uniform(x_bins[i], x_bins[i + 1])
                y = np.random.uniform(y_bins[j], y_bins[j + 1])
                z = np.random.uniform(z_min, z_max)
                if not (carpet_center[0] - carpet_len / 2 < x < carpet_center[0] + carpet_len / 2 and
                        carpet_center[1] - carpet_len / 2 < y < carpet_center[1] + carpet_len / 2):
                    points.append((x, y, z))

    return points
