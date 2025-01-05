import threading
import time
import robotic as ry
import numpy as np
from thrower_generic_projectile import throw_sample
from catcher_robot_projectile import CatcherRobot

if __name__ == "__main__":
    """
    Main function to configure and simulate the interaction between the thrower and catcher robots.
    This script sets up the robotic environment, initializes the catcher and thrower robots,
    and handles the throwing and catching process.
    """

    # Initialize the robotic environment
    C = ry.Config()
    C.addFile("catching.g")
    qHome = C.getJointState()  # Get the robot's home configuration
    C.getFrame('cargo').unLink()  # Ensure the cargo is unlinked from any parent frame

    # Add necessary frames for simulation
    C.addFrame("throw").setPosition([0, 1.8, 1.2]).setShape(ry.ST.marker, [0.3])
    C.addFrame("release_frame").setPosition([0, 0, 0]).setShape(ry.ST.marker, [0.4]).setColor([1, 0, 0]).setContact(0)

    # View the initial configuration
    C.view()

    # Set simulation parameters
    ry.params_add({'botsim/verbose': 2.0, 'physx/motorKp': 100000.0, 'physx/motorKd': 10000.0})
    ry.params_add({'botsim/engine': 'physx'}) 
    ry.params_add({'physx/multibody': True}) 

    # Initialize the robot operation and the catcher robot
    bot = ry.BotOp(C, useRealRobot=False)
    catcher_robot = CatcherRobot(C, bot)

    def on_ball_release():
        """
        Callback function triggered when the ball is released by the thrower.

        This function starts the catcher robot's prediction and movement threads.
        """
        print(f"Ball released!")
        catcher_robot.start()

    def after_ball_release():
        """
        Callback function triggered after the ball release process.

        This function stops the catcher robot's prediction and movement threads.
        """
        catcher_robot.stop()

    # Simulate the throwing process with callbacks for the ball release and post-release actions
    throw_sample(
        C, bot, isRender=True, sleep_time=10, 
        catch_callback1=on_ball_release, 
        catch_callback2=after_ball_release
    )
