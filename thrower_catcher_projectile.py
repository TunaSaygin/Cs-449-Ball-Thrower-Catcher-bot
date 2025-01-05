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
    ry.params_add({'botsim/verbose': 2., 'physx/motorKp': 10000., 'physx/motorKd': 1000.})
    ry.params_add({'botsim/engine': 'physx'}) 
    ry.params_add({'physx/multibody': True}) 
    bot = ry.BotOp(C, useRealRobot=False)
    catcher_robot = CatcherRobot(C, bot)

    def on_ball_release():
        """
        Callback function triggered when the ball is released by the thrower.
        """
        print(f"Ball released!")
        catcher_robot.start()

    def after_ball_release():
        catcher_robot.stop()

    # Start the thrower robot
    bin_position = [0, 0.7, 0.3]
    throw_sample(C, bot, isRender=True, sleep_time=10, catch_callback1=on_ball_release, catch_callback2=after_ball_release)

