import threading
import time
import robotic as ry
import numpy as np
from thrower_generic import find_velocity, pick_last_object_if_valid, grasp_object, throw_object, check_in_the_bin
from catcher_robot_tuna import VisionCatcherRobot

def throw_catch(C: ry.Config, bot: ry.BotOp, isRender:bool,sleep_time:float = 20):
    catcher = VisionCatcherRobot(C,bot)
    cargo_frame = C.getFrame("cargo")
    def stub_function():
        catcher.capture_point_cloud(cargo_frame)
        catcher.update_catcher_position()
    print(f"Initial bin pos:{C.getFrame('bin').getPosition()}")
    bin_new_position = C.getFrame('bin').getPosition()

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
    wanted_sleep = 0.57
    time_sleep = time_deviation * wanted_sleep
    throw_object(C,bot,time_sleep,release_velocity,stub_function,0.01)
    cargo_height = C.getFrame("cargo").getSize()[2]
    result, deviation = check_in_the_bin(C,bot,bin_new_position,C.getFrame("side2").getSize()[0]/2,C.getFrame("side2").getSize()[2],cargo_height)

    print(f"Result:{result} for bin pos:{bin_new_position}")

    if isRender:
        C.view()
        time.sleep(sleep_time)
    del C
    del bot
    return result, deviation

def run_thrower(C, bot):
    """
    Function to operate the thrower robot.
    """
    print("Thrower started...")
    throw_result, deviation = throw_catch(C, bot, render_simulation, sleep_time=0.01)
    print(f"Throw completed. Result: {throw_result}, Deviation: {deviation}")

def run_catcher():
    """
    Function to operate the catcher robot.
    """
    catcher = VisionCatcherRobot(C, bot)
    print("Catcher started...")
    catcher.start()

# Main Function
if __name__ == "__main__":
    render_simulation = True       # Whether to render the simulation

    # Shared configuration object for both robots
    C = ry.Config()
    C.addFile("catching.g")
    qHome = C.getJointState()
    C.getFrame('cargo').unLink()
    C.addFrame("throw").setPosition([0,1.8,1.2]).setShape(ry.ST.marker,[.3])
    C.addFrame("release_frame").setPosition([0,0,0]).setShape(ry.ST.marker,[.4]).setColor([1,0,0]).setContact(0)
    C.view()
    bot = ry.BotOp(C, useRealRobot=False)
    run_thrower(C, bot)
    time.sleep(25)
