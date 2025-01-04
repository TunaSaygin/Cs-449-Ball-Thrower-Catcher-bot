import threading
import time
import robotic as ry
import numpy as np
from thrower_generic_projectile import throw_sample
from catcher_robot_projectile import CatcherRobot

if __name__ == "__main__":
    # Configure robots
    C = ry.Config()
    C.addFile("catching.g")
    qHome = C.getJointState()
    C.getFrame('cargo').unLink()
    C.addFrame("throw").setPosition([0,1.8,1.2]).setShape(ry.ST.marker,[.3])
    C.addFrame("release_frame").setPosition([0,0,0]).setShape(ry.ST.marker,[.4]).setColor([1,0,0]).setContact(0)
    C.view()
    bot = ry.BotOp(C, useRealRobot=False)
    catcher_robot = CatcherRobot(C, bot)

    def on_ball_release(initial_position, velocity):
        """
        Callback function triggered when the ball is released by the thrower.
        """
        print(f"Ball released! Position: {initial_position}, Velocity: {velocity}")
        catcher_robot.start(initial_position, velocity)

    # Start the thrower robot
    bin_position = [0, 0.7, 0.3]
    throw_sample(C, bot, isRender=True, sleep_time=10, catch_callback=on_ball_release)

