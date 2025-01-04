### Init
import time
import robotic as ry
import numpy as np
from catcher_robot import VisionCatcherRobot
max_trials = 10


def calc_target_pos(thrower_pos, catcher_pos, clearance_dist=2.5):
    """
    Function to calculate the target position for the thrower robot.

    Args:
    - thrower_pos: np.array, position of the thrower robot
    - catcher_pos: np.array, position of the catcher robot
    - clearance_dist: float, distance to keep from the catcher

    Returns:
    - target_pos: np.array, target position for the thrower robot
    """

    thrower_pos = np.array(thrower_pos)
    catcher_pos = np.array(catcher_pos)

    # Calculate the direction vector from catcher to thrower
    direction = thrower_pos - catcher_pos
    distance = np.linalg.norm(direction)

    # Normalize the direction vector
    unit_direction = direction / distance

    # Set the target position 2.5m away from the catcher along the line
    max_distance = clearance_dist
    target_pos = catcher_pos + unit_direction * max_distance

    # Set the z-coordinate to 0.2 for the target position
    target_pos[2] = 0.2

    return target_pos


def move_robot_closer(thrower_fname, catcher_fname, waypoints, C: ry.Config, isRender:bool, clearance_dist=2.5, target_pos:np.array=None, debug=False):
# def move_robot_closer(thrower_fname, catcher_fname, waypoints, C: ry.Config, isRender:bool, target_pos:np.array=np.array([2.5, 0, 0.2]), debug=False):
    """
    Function to move the robot closer to the bin/catcher bot.

    Args:
    - thrower_fname: string, frame name of the thrower robot
    - catcher_fname: string, frame name of the catcher robot
    - waypoints: list, list of waypoints' frame names
    - C: ry.Config, configuration of the robot
    - isRender: bool, flag to render the RRT simulation of the robot moving closer
    - clearance_dist: float, distance to keep from the catcher
    - target_pos: np.array, target position for the thrower robot
    - debug: bool, flag to print debug information
    """

    q0 = C.getJointState()
    C.getFrame(thrower_fname).setParent(C.getFrame("thrower_base"), True, False)
    if target_pos is None:
        target_pos = calc_target_pos(C.getFrame(thrower_fname).getPosition(), C.getFrame(catcher_fname).getPosition(), clearance_dist=clearance_dist)

    # define target location, i.e. a point [2.5, 0, 0.2] away from the bin/catcher
    target_fname = "target_marker"
    target_marker = C.addFrame(target_fname, catcher_fname) \
        .setShape(ry.ST.marker, [1]) \
        .setColor([1, 0, 0]) \
        .setPosition(target_pos) \
    
    target_marker.unLink()

    ## First solve KOMO problem
    komo_1 = ry.KOMO()
    komo_1.setConfig(C, True)
    komo_1.setTiming(1., 20, 5., 0)

    # add collision constraints
    komo_1.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, [1e-1])
    komo_1.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

    komo_1.addObjective(
        times=[],
        feature=ry.FS.positionDiff,
        frames=[thrower_fname, target_fname],
        type=ry.OT.eq,
        scale=[1e1],
    )

    komo_1.addObjective(
        times=[],
        feature=ry.FS.vectorXDiff,
        frames=[thrower_fname, target_fname],
        type=ry.OT.eq,
        scale=[1e2],
    )

    komo_1.addObjective(
        times=[],
        feature=ry.FS.vectorYDiff,
        frames=[thrower_fname, target_fname],
        type=ry.OT.eq,
        scale=[1e2],
    )


    # solve
    ret_1 = ry.NLP_Solver(komo_1.nlp(), verbose= 1) .solve()
    thrower_to_catcher = komo_1.getPath()

    if debug:
        if ret_1.feasible:
            print("KOMO_1 is feasible")
        else:
            print("KOMO_1 is NOT feasible")

    # Now solve RRT problem
    ry.params_clear()
    ry.params_add({'rrt/stepsize':.1, 'rrt/verbose': 0}) #verbose=3 makes it very slow, and displays result, and verbose=4 waits keypress..

    foundRRTPath = False
    numOfTrials = 0

    while (not foundRRTPath and numOfTrials < max_trials):
        rrt = ry.PathFinder()
        rrt.setProblem(C, [q0], [thrower_to_catcher[0]])
        ret_RRT_1 = rrt.solve()
        path_1 = ret_RRT_1.x
        foundRRTPath = ret_RRT_1.feasible
        numOfTrials = numOfTrials + 1

    if debug:
        if foundRRTPath:
            print("*** Found path at trial num #", numOfTrials, " ***")
        else:
            print("** No path found ***")

    if isRender:
        for t in range(0, path_1.shape[0]-1):
            C.setJointState(path_1[t])
            C.view()
            time.sleep(0.01)
    # cleanup
    C.delFrame(target_fname)
    C.getFrame(thrower_fname).unLink()
    del komo_1

def check_if_throw_is_obstructed(thrower_fname, catcher_fname, C, threshold=0.5):
    """
    Function to check if the throw is obstructed by any obstacle.

    Args:
    - thrower_fname: string, frame name of the thrower robot
    - catcher_fname: string, frame name of the catcher robot/bin
    - C: ry.Config, configuration of the robot
    - threshold: float, distance threshold to check if the throw is obstructed

    Returns:
    - isObstructed: bool, flag to indicate if the throw is obstructed
    """

    # get the thrower and catcher frames
    thrower = C.getFrame(thrower_fname)
    catcher = C.getFrame(catcher_fname)

    # get the positions of the thrower and catcher robots
    thrower_pos = thrower.getPosition()
    catcher_pos = catcher.getPosition()

    # get the distance between the thrower and catcher robots
    dist = np.linalg.norm(thrower_pos - catcher_pos)

    # if the distance is less than a threshold, the throw is obstructed
    return dist >= threshold


# Main Function only for demonstration purposes
if __name__ == "__main__":
    C = ry.Config()
    C.addFile("catching.g")

    thrower_fname = "l_panda_base"
    catcher_fname = "l2_panda_base"

    thrower = C.getFrame(thrower_fname)
    catcher = C.getFrame(catcher_fname)
    initial_thrower_pos = thrower.getPosition()
    initial_catcher_pos = catcher.getPosition()
    initial_bin_pos = C.getFrame("bin").getPosition()

    print("Initial thrower position:", initial_thrower_pos)
    print("Initial catcher position:", initial_catcher_pos)
    print("Initial bin position:", initial_bin_pos)

    # for demonstration purposes, move the thrower robot far from the catcher robot
    thrower.setPosition(initial_thrower_pos + np.array([-2.5, 0, 0]))

    wall = C.addFrame(name="wall", parent="base") \
        .setPosition(initial_thrower_pos + np.array([-1, 0.75, 0.8])) \
        .setShape(ry.ST.ssBox, [0.1, 2, 2, 0.02]) \
        .setColor([0, 1, 0]) \
        .setContact(1) \
        .setMass(1)

    wall_2 = C.addFrame(name="wall_2", parent="base") \
        .setPosition(initial_thrower_pos + np.array([-1, -1.75, 0.8])) \
        .setShape(ry.ST.ssBox, [0.1, 2, 2, 0.02]) \
        .setColor([0.5, 0, 0]) \
        .setContact(1) \
        .setMass(1)


    # TODO(Shayan): Add waypoints    
    isObstructed = check_if_throw_is_obstructed(thrower_fname, catcher_fname, C)
    print("Is throw obstructed:", isObstructed)

    # C.view()
    # time.sleep(5)

    if isObstructed:
        move_robot_closer(thrower_fname, catcher_fname, ["wall"], C, isRender=True)

    C.view()
    time.sleep(5)

    del C



















































# import threading
# import time
# import robotic as ry
# import numpy as np
# from catcher_robot import VisionCatcherRobot
# max_trials = 10


# def move_robot_closer(bin_pos, robot_pos, robot_size, C: ry.Config, bot: ry.BotOp, isRender:bool):
#     """
#     Function to move the robot closer to the bin.
#     """
#     ry.params_clear()
#     ry.params_add({'rrt/stepsize':.1, 'rrt/verbose': 0}) #verbose=3 makes it very slow, and displays result, and verbose=4 waits keypress..

#     foundRRTPath = False
#     numOfTrials = 0

#     ## First solve KOMO problem
#     komo_1 = ry.KOMO()
#     komo_1.setConfig(C, True)
#     komo_1.setTiming(1., 20, 5., 0)

#     # add collision constraints
#     komo_1.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, [1e-1])
#     komo_1.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

#     # solve
#     ret_1 = ry.NLP_Solver(komo_1.nlp(), verbose=0 ) .solve()
#     q_gripper_to_cargo = komo_1.getPath()

#     if ret_1.feasible:
#         print("KOMO_1 is feasible")
#     else:
#         print("KOMO_1 is NOT feasible")

#     qHome = C.getJointState()


#     while (not foundRRTPath and numOfTrials < max_trials):
#         rrt = ry.PathFinder()
#         rrt.setProblem(C, [qHome], [q_gripper_to_cargo[0]])
#         ret_RRT_1 = rrt.solve()
#         path_1 = ret_RRT_1.x
#         foundRRTPath = ret_RRT_1.feasible
#         numOfTrials = numOfTrials + 1
#         # print("*** Trial num #", numOfTrials, " result: ", foundRRTPath)
    
#     if foundRRTPath:
#         print("*** Found path at trial num #", numOfTrials, " ***")
#     else:
#         print("** No path found ***")





# if __name__ == "__main__":
#     render_simulation = True       # Whether to render the simulation

#     # Shared configuration object for both robots
#     C = ry.Config()
#     C.addFile("catching.g")

#     bot = ry.BotOp(C, useRealRobot=False)

#     initial_thrower_pos = C.getFrame("l_panda_base").getPosition()
#     initial_bin_pos = C.getFrame("bin").getPosition()

#     print("Initial thrower position:", initial_thrower_pos)
#     print("Initial bin position:", initial_bin_pos)

#     wall = C.addFrame(name="wall", parent="base")
#     wall.setPosition(initial_thrower_pos + np.array([1, 0, 0.5]))
#     wall.setShape(ry.ST.ssBox, [0.1, 1, 1, 0.02])
#     wall.setColor([0, 1, 0])
#     wall.setContact(1)
#     wall.setMass(1)

#     C.view()
#     time.sleep(10)
#     try:
#         del C
#         del bot
#     except:
#         pass