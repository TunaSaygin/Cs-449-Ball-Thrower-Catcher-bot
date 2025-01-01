import threading
import time
import robotic as ry
import numpy as np
from thrower_generic import find_velocity, pick_last_object_if_valid, grasp_object, throw_object, check_in_the_bin
from catcher_robot import VisionCatcherRobot

render_simulation = True       # Whether to render the simulation

# Shared configuration object for both robots
config = ry.Config()
config.addFile("catching.g")

def throw_catch(isRender:bool,sleep_time:float = 20):
    C = ry.Config()
    C.addFile("catching.g")
    print(f"Initial bin pos:{C.getFrame('bin').getPosition()}")
    bin_new_position = C.getFrame('bin').getPosition()

    qHome = C.getJointState()
    C.getFrame('cargo').unLink() # to remove a bug
    C.addFrame("throw").setPosition([0,1.8,1.2]).setShape(ry.ST.marker,[.3])
    C.addFrame("release_frame").setPosition([0,0,0]).setShape(ry.ST.marker,[.4]).setColor([1,0,0]).setContact(0)
    C.view()

    bot = ry.BotOp(C, useRealRobot=False)
    release_velocity,isInverted = find_velocity(C)
    initial_position = pick_last_object_if_valid(C,C.getFrame("release_frame").getPosition(),release_velocity)
    #invertion deviation calculation
    if isInverted:
        deviation_dist = 0.2
        a = -release_velocity[1]/release_velocity[0]
        dev_x = deviation_dist/np.sqrt(1+np.square(a))
        dev_y = deviation_dist/np.sqrt(1+np.square(1/a))
        deviation_arr = np.array([dev_x,dev_y,0])
        time_deviation = grasp_object(C,bot,isInverted,deviation_arr)
    else:
        time_deviation = grasp_object(C,bot,isInverted)
    print(f"Final bin pos:{C.getFrame('bin').getPosition()}")
    wanted_sleep = 0.5
    time_sleep = time_deviation * wanted_sleep
    throw_object(C,bot,time_sleep,release_velocity)
    cargo_height = C.getFrame("cargo").getSize()[2]
    result, deviation = check_in_the_bin(C,bot,bin_new_position,C.getFrame("side2").getSize()[0]/2,C.getFrame("side2").getSize()[2],cargo_height)

    print(f"Result:{result} for bin pos:{bin_new_position}")

    if isRender:
        C.view()
        time.sleep(sleep_time)
    del C
    del bot
    return result, deviation

def run_thrower():
    """
    Function to operate the thrower robot.
    """
    print("Thrower started...")
    throw_result, deviation = throw_catch(render_simulation, sleep_time=0.01)
    print(f"Throw completed. Result: {throw_result}, Deviation: {deviation}")

def run_catcher():
    """
    Function to operate the catcher robot.
    """
    catcher = VisionCatcherRobot(config)
    print("Catcher started...")

    # Start tracking the ball
    while True:
        catcher.process_frame()
        time.sleep(0.01)

# Main Function
if __name__ == "__main__":
    # Create threads for thrower and catcher
    thrower_thread = threading.Thread(target=run_thrower)
    catcher_thread = threading.Thread(target=run_catcher)

    # Start both threads
    thrower_thread.start()
    catcher_thread.start()

    # Wait for the threads to complete
    thrower_thread.join()
    catcher_thread.join()
