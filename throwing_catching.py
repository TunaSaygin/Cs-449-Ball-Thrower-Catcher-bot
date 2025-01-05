import threading
import time
import robotic as ry
import numpy as np
from thrower_generic import find_velocity, pick_last_object_if_valid, grasp_object, throw_object, check_in_the_bin
from catcher_robot import VisionCatcherRobot

## @brief Coordinates the thrower robot's operations and checks the result.
#  @param C The shared configuration object for the simulation.
#  @param bot The BotOp object controlling the thrower robot.
#  @param isRender Whether to render the simulation.
#  @param sleep_time The time to wait before ending the simulation.
#  @return A tuple containing the throw result and deviation from the target.
def throw_catch(C: ry.Config, bot: ry.BotOp, isRender: bool, sleep_time: float = 20):
    print(f"Initial bin pos: {C.getFrame('bin').getPosition()}")
    bin_new_position = C.getFrame('bin').getPosition()

    # Calculate release velocity and determine if the motion is inverted
    release_velocity, isInverted = find_velocity(C)
    initial_position = pick_last_object_if_valid(C, C.getFrame("release_frame").getPosition(), release_velocity)

    # Handle inversion deviations
    if isInverted:
        deviation_dist = 0.2
        a = -release_velocity[1] / release_velocity[0]
        dev_x = deviation_dist / np.sqrt(1 + np.square(a))
        dev_y = deviation_dist / np.sqrt(1 + np.square(1 / a))
        deviation_arr = np.array([dev_x, dev_y, 0])
        time_deviation = grasp_object(C, bot, isInverted, deviation_arr)
    else:
        time_deviation = grasp_object(C, bot, isInverted)

    print(f"Final bin pos: {C.getFrame('bin').getPosition()}")

    # Calculate the appropriate sleep time before releasing the object
    wanted_sleep = 0.57
    time_sleep = time_deviation * wanted_sleep

    # Perform the throw operation
    throw_object(C, bot, time_sleep, release_velocity)

    # Check if the cargo lands inside the bin
    cargo_height = C.getFrame("cargo").getSize()[2]
    result, deviation = check_in_the_bin(
        C, bot, bin_new_position, C.getFrame("side2").getSize()[0] / 2,
        C.getFrame("side2").getSize()[2], cargo_height
    )

    print(f"Result: {result} for bin pos: {bin_new_position}")

    # Render the simulation if enabled
    if isRender:
        C.view()
        time.sleep(sleep_time)

    del C
    del bot
    return result, deviation

## @brief Function to operate the thrower robot.
def run_thrower():
    print("Thrower started...")
    throw_result, deviation = throw_catch(C, bot, render_simulation, sleep_time=0.01)
    print(f"Throw completed. Result: {throw_result}, Deviation: {deviation}")

## @brief Function to operate the catcher robot.
def run_catcher():
    catcher = VisionCatcherRobot(C, bot)
    print("Catcher started...")
    catcher.start()

## @brief Main function for initializing and running both the thrower and catcher robots.
if __name__ == "__main__":
    render_simulation = True  # Whether to render the simulation

    # Initialize the shared configuration object for both robots
    C = ry.Config()
    C.addFile("catching.g")
    qHome = C.getJointState()

    # Unlink the cargo and add key frames for throwing and releasing
    C.getFrame('cargo').unLink()
    C.addFrame("throw").setPosition([0, 1.8, 1.2]).setShape(ry.ST.marker, [.3])
    C.addFrame("release_frame").setPosition([0, 0, 0]).setShape(ry.ST.marker, [.4]).setColor([1, 0, 0]).setContact(0)
    C.view()

    bot = ry.BotOp(C, useRealRobot=False)

    # Create threads for thrower and catcher robots
    thrower_thread = threading.Thread(target=run_thrower)
    catcher_thread = threading.Thread(target=run_catcher)

    # Start both threads
    thrower_thread.start()
    time.sleep(0.3)  # Small delay to stagger thread execution
    catcher_thread.start()

    # Wait for both threads to finish execution
    thrower_thread.join()
    catcher_thread.join()

    time.sleep(25)  # Final pause before ending
