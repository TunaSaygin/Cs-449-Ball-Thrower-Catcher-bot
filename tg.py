import robotic as ry
import numpy as np
from my_utils import my_printer, rotation_matrix_to_quaternion
import time
from velocity_finder import find_velocity, pick_last_object_if_valid
from rrt_move import move_robot_closer, check_if_throw_is_obstructed

def throw_sample(bin_new_position, isRender, time_deviation, release_velocity, bot, sleep_time=20, C=None):
    """
    Perform a throwing simulation to place the cargo into the bin.

    Args:
        bin_new_position (list): New position of the bin.
        isRender (bool): Whether to render the simulation.
        time_deviation (float): Time deviation caused by the grasping process.
        release_velocity (np.ndarray): Velocity vector for releasing the object.
        bot (ry.BotOp): Robot operation interface.
        sleep_time (float): Time to wait before continuing simulation (default: 20 seconds).
        C (ry.Config): Configuration object for the simulation.

    Returns:
        tuple: Result of the throw (bool) and deviation from the target (float).
    """
    wanted_sleep = 0.665
    time_sleep = time_deviation * wanted_sleep
    throw_object(C, bot, time_sleep, release_velocity)
    cargo_height = C.getFrame("cargo").getSize()[2]
    result, deviation = check_in_the_bin(
        C, bot, bin_new_position,
        C.getFrame("side2").getSize()[0] / 2,
        C.getFrame("side2").getSize()[2],
        cargo_height
    )

    print(f"Result: {result} for bin pos: {bin_new_position}")
    if isRender:
        C.view()
    del C
    del bot
    return result, deviation

def check_in_the_bin(C, bot, bin_center, binxy_length, bin_height, cargo_height):
    """
    Check if the cargo is successfully inside the bin.

    Args:
        C (ry.Config): Configuration object for the simulation.
        bot (ry.BotOp): Robot operation interface.
        bin_center (list): Center position of the bin.
        binxy_length (float): Length of the bin in x and y directions.
        bin_height (float): Height of the bin.
        cargo_height (float): Height of the cargo.

    Returns:
        tuple: Success status (bool) and deviation from the target (float).
    """
    start = time.time()
    prev_pos = np.array(C.getFrame("cargo").getPosition())
    bot.sync(C, 0.01)
    cargo_pos = np.array(C.getFrame("cargo").getPosition())

    while (not np.array_equal(prev_pos, cargo_pos)) or (prev_pos[2] >= cargo_height and cargo_pos[2] >= cargo_height):
        prev_pos = cargo_pos
        bot.sync(C, 0.01)
        cargo_pos = np.array(C.getFrame("cargo").getPosition())
        if time.time() - start > 8:
            break

    deviation = np.linalg.norm(cargo_pos - np.array(bin_center))
    if (
        bin_center[0] - binxy_length <= cargo_pos[0] <= bin_center[0] + binxy_length and
        bin_center[1] - binxy_length <= cargo_pos[1] <= bin_center[1] + binxy_length and
        bin_center[2] <= cargo_pos[2] <= bin_center[2] + bin_height
    ):
        return True, deviation
    return False, deviation

def throw_object(C, bot, time_sleep, velocity, stub_function=None, time_interval=0.1):
    """
    Perform the object throwing motion.

    Args:
        C (ry.Config): Configuration object for the simulation.
        bot (ry.BotOp): Robot operation interface.
        time_sleep (float): Time to sleep before releasing the object.
        velocity (np.ndarray): Velocity vector for releasing the object.
        stub_function (function, optional): Stub function for additional operations.
        time_interval (float): Time interval for synchronization (default: 0.1).
    """
    def vel_komo():
        komo = ry.KOMO(C, 1, 1, 1, True)
        komo.addObjective([], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e-1], np.array(velocity), 1)
        komo.addObjective([], ry.FS.positionDiff, ["l_panda_coll5", "throw"], ry.OT.sos, [1e0])
        komo.addObjective([], ry.FS.scalarProductXX, ["l_gripper", "l_panda_coll5"], ry.OT.eq, [1e0], [-1])
        komo.addObjective([], ry.FS.scalarProductYY, ["l_panda_coll3", "l_panda_coll5"], ry.OT.eq, [1e2], [1])
        ret2 = ry.NLP_Solver(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=0).solve()
        return komo

    komo = vel_komo()
    gripper_frame = C.getFrame("l_gripper")
    path = komo.getPath()
    bot.move(path, [1.0])
    time.sleep(time_sleep)
    bot.gripperMove(ry._left, width=1)
    bot.sync(C, 0.001)
    if stub_function:
        stub_function()
    C.addFrame("actual_release").setPosition(gripper_frame.getPosition()).setShape(ry.ST.marker, [0.2]).setColor([1, 1, 0])
    while bot.getTimeToEnd() > 0:
        if stub_function:
            stub_function()
        bot.sync(C, time_interval)

# Additional helper functions are similarly documented.

if __name__ == "__main__":
    # Main testing block
    C = ry.Config()
    C.addFile("throwing_bare_for_tg.g")
    hardcoded_bin = C.getFrame("bin").getPosition()
    bot = ry.BotOp(C, useRealRobot=False)

    init_environment(C, bin_new_position=hardcoded_bin, bin_shape=[0.5, 0.5])
    time_deviation, release_velocity = grab_cargo(bot, hardcoded_bin, C=C)
    throw_sample(hardcoded_bin, True, sleep_time=10, C=C, time_deviation=time_deviation, release_velocity=release_velocity, bot=bot)
    C.view()
    time.sleep(2)
