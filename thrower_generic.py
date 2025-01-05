import robotic as ry
import numpy as np
from my_utils import rotation_matrix_to_quaternion
import time
from velocity_finder import find_velocity, pick_last_object_if_valid
from my_utils import get_quat_from_velocity
import json

## @brief Simulates the throwing of an object into a bin.
#  @param bin_new_position List specifying the new bin position [x, y, z].
#  @param isRender Boolean to decide if rendering is enabled.
#  @param sleep_time Time to sleep after rendering, in seconds.
#  @param bin_shape Shape of the bin as [length, width].
#  @return A tuple of (result, deviation) where result is True if successful, False otherwise, and deviation is the error distance.
def throw_sample(bin_new_position: list, isRender: bool, sleep_time: float = 20, bin_shape=[0.5, 0.5]):
    C = ry.Config()
    C.addFile("throwing_bare.g")
    print(f"Initial bin pos:{C.getFrame('bin').getPosition()}")
    init_environment(C, bin_new_position, bin_shape)
    bot = ry.BotOp(C, useRealRobot=False)
    release_velocity, isInverted = find_velocity(C)
    initial_position = pick_last_object_if_valid(C, C.getFrame("release_frame").getPosition(), release_velocity)

    # Adjust trajectory if inverted
    if isInverted:
        deviation_dist = 0.2
        a = -release_velocity[1] / release_velocity[0]
        dev_x = deviation_dist / np.sqrt(1 + np.square(a))
        dev_y = deviation_dist / np.sqrt(1 + np.square(1 / a))
        deviation_arr = np.array([dev_x, dev_y, 0])
        time_deviation = grasp_object(C, bot, isInverted, deviation_arr)
    else:
        time_deviation = grasp_object(C, bot, isInverted)

    print(f"Final bin pos:{C.getFrame('bin').getPosition()}")
    wanted_sleep = 0.55
    time_sleep = time_deviation * wanted_sleep
    throw_object(C, bot, time_sleep, release_velocity)
    cargo_height = C.getFrame("cargo").getSize()[2]
    result, deviation = check_in_the_bin(C, bot, bin_new_position, C.getFrame("side2").getSize()[0] / 2,
                                         C.getFrame("side2").getSize()[2], cargo_height)

    print(f"Result:{result} for bin pos:{bin_new_position}")

    if isRender:
        C.view()
        time.sleep(sleep_time)
    del C
    del bot
    return result, deviation


## @brief Checks if the cargo has successfully landed inside the bin.
#  @param C The robotic configuration object.
#  @param bot The robotic operation object.
#  @param bin_center The bin's center position [x, y, z].
#  @param binxy_length Half the bin's length and width.
#  @param bin_height Height of the bin.
#  @param cargo_height Height of the cargo.
#  @return A tuple (is_success, deviation), where is_success is True if successful, False otherwise, and deviation is the error distance.
def check_in_the_bin(C: ry.Config, bot: ry.BotOp, bin_center, binxy_length, bin_height, cargo_height):
    start = time.time()
    prev_pos = np.array(C.getFrame("cargo").getPosition())
    bot.sync(C, .01)
    cargo_pos = np.array(C.getFrame("cargo").getPosition())

    while (not np.array_equal(prev_pos, cargo_pos)) or (prev_pos[2] >= cargo_height and cargo_pos[2] >= cargo_height):
        prev_pos = cargo_pos
        bot.sync(C, .01)
        cargo_pos = np.array(C.getFrame("cargo").getPosition())
        if time.time() - start > 8:
            break

    deviation = np.linalg.norm(cargo_pos - np.array(bin_center))
    is_success = (bin_center[0] - binxy_length <= cargo_pos[0] <= bin_center[0] + binxy_length and
                  bin_center[1] - binxy_length <= cargo_pos[1] <= bin_center[1] + binxy_length and
                  bin_center[2] <= cargo_pos[2] <= bin_center[2] + bin_height)
    return is_success, deviation


## @brief Simulates the throwing action of the robot.
#  @param C The robotic configuration object.
#  @param bot The robotic operation object.
#  @param time_sleep Time to sleep before releasing the object.
#  @param velocity The release velocity for the object.
#  @param stub_function Optional function for additional processing during motion.
#  @param time_interval Synchronization interval for the robot.
def throw_object(C, bot, time_sleep, velocity, stub_function=None, time_interval=0.1):
    def vel_komo():
        q0 = C.getJointState()
        komo = ry.KOMO(C, 1, 1, 1, True)
        komo.addObjective([], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e-1], np.array(velocity), 1)
        komo.addObjective([], ry.FS.positionDiff, ["l_gripper", "release_frame"], ry.OT.sos, [1e0], [0, 0, 0])
        ry.NLP_Solver(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=0).solve()
        return komo

    komo = vel_komo()
    gripper_frame = C.getFrame("l_gripper")
    path = komo.getPath()
    bot.move(path, [1.])
    time.sleep(time_sleep)
    bot.gripperMove(ry._left, width=1)
    bot.sync(C, 0.001)
    if stub_function:
        stub_function()
    C.addFrame("actual_release").setPosition(gripper_frame.getPosition()).setShape(ry.ST.marker, [.2]).setColor([1, 1, 0])
    while bot.getTimeToEnd() > 0:
        if stub_function:
            stub_function()
        bot.sync(C, time_interval)


## @brief Grasps an object using the robot's gripper.
#  @param C The robotic configuration object.
#  @param bot The robotic operation object.
#  @param isInverted Boolean indicating if inversion adjustments are required.
#  @param deviation_arr Optional deviation array for position adjustments.
#  @param object_name Name of the object to grasp (default: "cargo").
#  @return The time deviation caused by the grasping motion.
def grasp_object(C: ry.Config, bot: ry.BotOp, isInverted: bool, deviation_arr=None, object_name: str = "cargo"):
    q0 = qHome = C.getJointState()
    komo_pre_grasp = pre_grasp_komo(C, "l_gripper", object_name, q0, qHome)
    path_pre_grasp = komo_pre_grasp.getPath()
    komo_post_grasp = post_grasp_komo(C, isInverted, deviation_arr)
    path_post_grasp = komo_post_grasp.getPath()
    bot.move(path_pre_grasp, [1.])
    while bot.getTimeToEnd() > 0:
        bot.sync(C, .1)
    bot.gripperClose(ry._left, width=0.06)
    while not bot.gripperDone(ry._left):
        bot.sync(C, .1)
    bot.move(path_post_grasp, [3.])
    while bot.getTimeToEnd() > 0:
        bot.sync(C, .1)
    C.attach('l_gripper', object_name)
    return (path_pre_grasp.shape[0] * 0.1)


## @brief Plans the pre-grasp motion using KOMO.
#  @param C The robotic configuration object.
#  @param gripper_name Name of the gripper frame.
#  @param grasp_frame_name Name of the target frame for grasping.
#  @param q0 The initial joint state.
#  @param qHome The home joint state.
#  @return The KOMO object for pre-grasp planning.
def pre_grasp_komo(C, gripper_name, grasp_frame_name, q0, qHome):
    komo = ry.KOMO(C, 3, 1, 0, True)
    komo.addObjective([], ry.FS.positionRel, [gripper_name, grasp_frame_name], ry.OT.eq, [1e1], [0, 0, 0])
    komo.addObjective([], ry.FS.scalarProductZZ, [grasp_frame_name, gripper_name], ry.OT.eq, [1e1], [1])
    komo.addObjective([], ry.FS.jointState, [], ry.OT.sos, [1e-1], q0)
    return komo


## @brief Plans the post-grasp motion using KOMO.
#  @param C The robotic configuration object.
#  @param isInverted Boolean indicating if adjustments are needed.
#  @param deviationarr Optional deviation array for adjustment.
#  @return The KOMO object for post-grasp planning.
def post_grasp_komo(C, isInverted: bool, deviationarr=None) -> ry.KOMO:
    komo = ry.KOMO(C, 1, 1, 0, True)
    komo.addObjective([], ry.FS.positionDiff, ["l_gripper", "initial_position"], ry.OT.sos, [1e0], deviationarr or [0, 0, 0])
    return komo


## @brief Initializes the environment with the bin and object positions.
#  @param C The robotic configuration object.
#  @param bin_new_position The position of the bin [x, y, z].
#  @param bin_shape Shape of the bin [length, width].
def init_environment(C: ry.Config, bin_new_position: list, bin_shape):
    if bin_new_position[2] < 0.08:
        bin_new_position[2] = 0.08
    base_position = np.array(C.getFrame("l_panda_base").getPosition())
    new_quat = rotate_bin(bin_new_position, base_position)
    C.getFrame('bin').setPosition(bin_new_position).setQuaternion(new_quat)


## @brief Rotates the bin to align with the robot base.
#  @param bin_position The position of the bin [x, y, z].
#  @param base_position The position of the robot base [x, y, z].
#  @return A quaternion representing the bin's rotation.
def rotate_bin(bin_position: np.ndarray, base_position: np.ndarray):
    vector_base_bin = bin_position - base_position
    xy_distance = np.sqrt(np.square(vector_base_bin[0]) + np.square(vector_base_bin[1]))
    bin_x_vector = vector_base_bin / xy_distance
    bin_z_vector = np.array([0, 0, 1])
    bin_y_vector = np.cross(bin_z_vector, bin_x_vector)
    rotation_matrix = np.column_stack((bin_x_vector, bin_y_vector, bin_z_vector))
    return rotation_matrix_to_quaternion(rotation_matrix)


## @brief Generates homogeneous points outside the carpet area.
#  @param robo_base The robot base position.
#  @param carpet_center The center of the carpet area.
#  @param carpet_len Length of the carpet area.
#  @param range_limit Range limit around the robot base.
#  @param z_min Minimum height for points.
#  @param z_max Maximum height for points.
#  @param num_points Number of points to generate.
#  @param grid_resolution Resolution of the grid for points.
#  @return A list of points outside the carpet area.
def generate_homogeneous_points(robo_base, carpet_center, carpet_len, range_limit, z_min, z_max, num_points, grid_resolution=10):
    points = []
    x_min = robo_base[0] - range_limit
    x_max = robo_base[0] + range_limit
    y_min = robo_base[1] - range_limit
    y_max = robo_base[1] + range_limit
    x_bins = np.linspace(x_min, x_max, grid_resolution)
    y_bins = np.linspace(y_min, y_max, grid_resolution)
    for _ in range(num_points):
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)
        z = np.random.uniform(z_min, z_max)
        points.append((x, y, z))
    return points
