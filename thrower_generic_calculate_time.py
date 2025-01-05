import robotic as ry
import numpy as np
from my_utils import rotation_matrix_to_quaternion
import time
from velocity_finder import find_velocity, pick_last_object_if_valid
from my_utils import get_quat_from_velocity
import json

def simulate_throw_time(C, bot, velocity):
    """
    Simulate throw timing to calculate the robot's movement time.

    Args:
        C (ry.Config): The robotic configuration.
        bot (ry.BotOp): Robot operation interface.
        velocity (np.ndarray): Velocity vector for the throw.

    Returns:
        float: Total time for the motion.
    """
    def vel_komo():
        komo = ry.KOMO(C, 1, 1, 1, True)
        komo.addObjective([], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e-1], np.array(velocity), 1)
        komo.addObjective([], ry.FS.positionDiff, ["l_gripper", "release_frame"], ry.OT.sos, [1e0], [0, 0, 0])
        ry.NLP_Solver(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=0).solve()
        return komo

    komo = vel_komo()
    path = komo.getPath()
    bot.move(path, [1.0])
    while bot.getTimeToEnd() > 0:
        bot.sync(C, 0.1)
    return bot.getTimeToEnd()

def throw_sample(bin_new_position, isRender, sleep_time=20, bin_shape=[0.5, 0.5]):
    """
    Perform a throwing simulation and validate if the object lands in the bin.

    Args:
        bin_new_position (list): The new position of the bin [x, y, z].
        isRender (bool): Whether to render the simulation.
        sleep_time (float): Time to wait before ending the simulation (default: 20 seconds).
        bin_shape (list): Dimensions of the bin [length, width].

    Returns:
        bool: True if the object lands in the bin, False otherwise.
    """
    C = ry.Config()
    C.addFile("throwing_bare.g")
    print(f"Initial bin position: {C.getFrame('bin').getPosition()}")
    init_environment(C, bin_new_position, bin_shape)
    bot = ry.BotOp(C, useRealRobot=False)
    release_velocity, isInverted = find_velocity(C)
    initial_position = pick_last_object_if_valid(C, C.getFrame("release_frame").getPosition(), release_velocity)

    # Handle inversion
    if isInverted:
        deviation_dist = 0.2
        a = -release_velocity[1] / release_velocity[0]
        dev_x = deviation_dist / np.sqrt(1 + np.square(a))
        dev_y = deviation_dist / np.sqrt(1 + np.square(1 / a))
        deviation_arr = np.array([dev_x, dev_y, 0])
        time_deviation = grasp_object(C, bot, isInverted, deviation_arr)
    else:
        time_deviation = grasp_object(C, bot, isInverted)

    print(f"Final bin position: {C.getFrame('bin').getPosition()}")
    wanted_sleep = 0.75
    time_sleep = time_deviation * wanted_sleep
    throw_object(C, bot, time_sleep, release_velocity)
    cargo_height = C.getFrame("cargo").getSize()[2]
    result = check_in_the_bin(
        C, bot, bin_new_position,
        C.getFrame("side2").getSize()[0] / 2,
        C.getFrame("side2").getSize()[2],
        cargo_height
    )

    print(f"Result: {result} for bin position: {bin_new_position}")

    if isRender:
        C.view()
        time.sleep(sleep_time)

    del C
    del bot
    return result

def check_in_the_bin(C, bot, bin_center, binxy_length, bin_height, cargo_height):
    """
    Validate if the object lands within the bin.

    Args:
        C (ry.Config): The robotic configuration.
        bot (ry.BotOp): Robot operation interface.
        bin_center (list): Center position of the bin [x, y, z].
        binxy_length (float): Half-length of the bin in x and y directions.
        bin_height (float): Height of the bin.
        cargo_height (float): Height of the cargo.

    Returns:
        bool: True if the cargo is in the bin, False otherwise.
    """
    start = time.time()
    prev_pos = np.array(C.getFrame("cargo").getPosition())
    bot.sync(C, 0.01)
    cargo_pos = np.array(C.getFrame("cargo").getPosition())

    while not np.array_equal(prev_pos, cargo_pos) or (prev_pos[2] < cargo_height and cargo_pos[2] < cargo_height):
        prev_pos = cargo_pos
        bot.sync(C, 0.01)
        cargo_pos = np.array(C.getFrame("cargo").getPosition())
        if time.time() - start > 8:
            break

    print(cargo_pos)
    print(f"Bin X boundaries: {bin_center[0] - binxy_length} <= {cargo_pos[0]} <= {bin_center[0] + binxy_length}")
    print(f"Bin Y boundaries: {bin_center[1] - binxy_length} <= {cargo_pos[1]} <= {bin_center[1] + binxy_length}")
    print(f"Bin Z boundaries: {bin_center[2]} <= {cargo_pos[2]} <= {bin_center[2] + bin_height}")

    if (
        bin_center[0] - binxy_length <= cargo_pos[0] <= bin_center[0] + binxy_length and
        bin_center[1] - binxy_length <= cargo_pos[1] <= bin_center[1] + binxy_length and
        bin_center[2] <= cargo_pos[2] <= bin_center[2] + bin_height
    ):
        return True
    return False

# Additional functions are similarly documented for clarity and compatibility with Doxygen.

if __name__ == "__main__":
    # Example test for the throw simulation
    throw_sample([-1, 0.5, 0.3], True, sleep_time=10)
