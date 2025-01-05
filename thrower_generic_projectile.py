import robotic as ry
import numpy as np
from my_utils import rotation_matrix_to_quaternion
import time
from velocity_finder import find_velocity, pick_last_object_if_valid
from my_utils import get_quat_from_velocity
import json

## @file
#  @brief This file contains the implementation of a throwing system simulation using a robotic arm.
#         Functions include initialization, object grasping, throwing, and trajectory verification.

## @brief Executes the main simulation of throwing an object using the robotic arm.
#  @param C The configuration object for the simulation.
#  @param bot The robot operation object.
#  @param isRender Whether to render the simulation environment.
#  @param sleep_time Time in seconds to keep the simulation running for observation.
#  @param catch_callback1 Callback triggered when the object is released.
#  @param catch_callback2 Callback triggered after the throwing process completes.
#  @return A tuple containing the result (boolean) of whether the throw was successful and the deviation.
def throw_sample(C: ry.Config, bot: ry.BotOp, isRender: bool, sleep_time: float = 20, 
                 catch_callback1=None, catch_callback2=None):
    print(f"Initial bin pos:{C.getFrame('bin').getPosition()}")
    bin_new_position = C.getFrame('bin').getPosition()
    release_velocity, isInverted = find_velocity(C)
    initial_position = pick_last_object_if_valid(C, C.getFrame("release_frame").getPosition(), release_velocity)

    # Handle inversion deviation calculation
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
    throw_object(C, bot, time_sleep, release_velocity, catch_callback1)

    cargo_height = C.getFrame("cargo").getSize()[2]
    result, deviation = check_in_the_bin(C, bot, bin_new_position, 
                                         C.getFrame("side2").getSize()[0] / 2, 
                                         C.getFrame("side2").getSize()[2], cargo_height)
    catch_callback2()

    print(f"Result:{result} for bin pos:{bin_new_position}")

    if isRender:
        C.view()
        time.sleep(sleep_time)

    del C
    del bot
    return result, deviation

## @brief Checks whether the thrown object successfully lands in the designated bin.
#  @param C The configuration object for the simulation.
#  @param bot The robot operation object.
#  @param bin_center The center position of the bin.
#  @param binxy_length Half the length of the bin in the XY-plane.
#  @param bin_height The height of the bin.
#  @param cargo_height The height of the cargo.
#  @return A tuple containing a boolean indicating success and the deviation.
def check_in_the_bin(C: ry.Config, bot: ry.BotOp, bin_center, binxy_length, bin_height, cargo_height):
    start = time.time()
    prev_pos = np.array(C.getFrame("cargo").getPosition())
    bot.sync(C, .01)
    cargo_pos = np.array(C.getFrame("cargo").getPosition())

    # Wait for stabilization or timeout
    while (not np.array_equal(prev_pos, cargo_pos)) or (prev_pos[2] >= cargo_height and cargo_pos[2] >= cargo_height):
        prev_pos = cargo_pos
        bot.sync(C, .01)
        cargo_pos = np.array(C.getFrame("cargo").getPosition())
        if time.time() - start > 8:
            break

    print(f"Final cargo position: {cargo_pos}")
    deviation = np.linalg.norm(cargo_pos - np.array(bin_center))
    print(f"Deviation from target position: {deviation:.4f} meters")

    if (bin_center[0] - binxy_length <= cargo_pos[0] <= bin_center[0] + binxy_length and 
        bin_center[1] - binxy_length <= cargo_pos[1] <= bin_center[1] + binxy_length and 
        bin_center[2] <= cargo_pos[2] <= bin_center[2] + bin_height):
        return True, deviation
    return False, deviation

## @brief Executes the throwing operation using the robotic arm.
#  @param C The configuration object for the simulation.
#  @param bot The robot operation object.
#  @param time_sleep Time to wait before releasing the object.
#  @param velocity The velocity vector for the throw.
#  @param initial_position_callback Callback triggered before the throwing motion.
#  @param time_interval Synchronization interval for the robot.
def throw_object(C, bot, time_sleep, velocity, initial_position_callback=None, time_interval=0.001):
    print(f"velocity is !!!: {velocity}")
    def vel_komo():
        q0 = C.getJointState()
        komo = ry.KOMO(C, 1, 1, 1, True)
        komo.addObjective([], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e-1], np.array(velocity), 1)
        komo.addObjective([], ry.FS.positionDiff, ["l_gripper", "release_frame"], ry.OT.sos, [1e0], [0, 0, 0])
        komo.addObjective([], ry.FS.jointState, [], ry.OT.sos, [1e-1], q0)
        ret2 = ry.NLP_Solver(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=0).solve()
        print(f"ret!!!: {ret2}")
        return komo

    if initial_position_callback:
        initial_position_callback()

    komo = vel_komo()
    gripper_frame = C.getFrame("l_gripper")
    path = komo.getPath()
    bot.move(path, [1.])
    time.sleep(time_sleep)
    bot.gripperMove(ry._left, width=1)
    bot.sync(C, 0.001)
    C.addFrame("actual_release").setPosition(gripper_frame.getPosition()).setShape(ry.ST.marker, [.2]).setColor([1, 1, 0])
    print(f"bot end time:{bot.getTimeToEnd()}")
    while bot.getTimeToEnd() > 0:
        bot.sync(C, time_interval)
