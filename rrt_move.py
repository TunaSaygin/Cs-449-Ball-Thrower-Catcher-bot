import time
import robotic as ry
import numpy as np
from catcher_robot_projectile import CatcherRobot

# Define maximum trials for pathfinding
max_trials = 10


def calc_target_pos(thrower_pos, catcher_pos, clearance_dist=2.5):
    """
    Calculate the target position for the thrower robot based on the catcher robot's position.

    Args:
        thrower_pos (np.array): Position of the thrower robot.
        catcher_pos (np.array): Position of the catcher robot.
        clearance_dist (float): Distance to maintain from the catcher robot.

    Returns:
        np.array: Target position for the thrower robot.
    """
    thrower_pos = np.array(thrower_pos)
    catcher_pos = np.array(catcher_pos)

    # Compute direction vector and normalize it
    direction = thrower_pos - catcher_pos
    distance = np.linalg.norm(direction)
    unit_direction = direction / distance

    # Calculate the target position along the direction vector
    target_pos = catcher_pos + unit_direction * clearance_dist
    target_pos[2] = 0.2  # Set z-coordinate for the target position
    return target_pos


def move_robot_closer(thrower_fname, catcher_fname, waypoints, C: ry.Config, isRender: bool, clearance_dist=2.5, target_pos: np.array = None, debug=False):
    """
    Move the robot closer to the target (bin or catcher robot) using KOMO and RRT.

    Args:
        thrower_fname (str): Frame name of the thrower robot.
        catcher_fname (str): Frame name of the catcher robot.
        waypoints (list): List of waypoint frame names.
        C (ry.Config): Configuration object for the robotic setup.
        isRender (bool): Whether to render the simulation.
        clearance_dist (float): Clearance distance from the target.
        target_pos (np.array): Target position for the thrower robot.
        debug (bool): Flag to enable debug information.
    """
    q0 = C.getJointState()
    C.getFrame(thrower_fname).setParent(C.getFrame("thrower_base"), True, False)

    if target_pos is None:
        target_pos = calc_target_pos(C.getFrame(thrower_fname).getPosition(), C.getFrame(catcher_fname).getPosition(), clearance_dist)

    # Define and configure the target marker
    target_fname = "target_marker"
    target_marker = C.addFrame(target_fname, catcher_fname) \
        .setShape(ry.ST.marker, [1]) \
        .setColor([1, 0, 0]) \
        .setPosition(target_pos)

    target_marker.unLink()

    # Solve KOMO problem for smooth trajectory
    komo_1 = ry.KOMO()
    komo_1.setConfig(C, True)
    komo_1.setTiming(1.0, 20, 5.0, 0)
    komo_1.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, [1e-1])
    komo_1.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
    komo_1.addObjective([], ry.FS.positionDiff, [thrower_fname, target_fname], ry.OT.eq, [1e1])
    komo_1.addObjective([], ry.FS.vectorXDiff, [thrower_fname, target_fname], ry.OT.eq, [1e2])
    komo_1.addObjective([], ry.FS.vectorYDiff, [thrower_fname, target_fname], ry.OT.eq, [1e2])

    # Solve and extract the trajectory
    ret_1 = ry.NLP_Solver(komo_1.nlp(), verbose=1).solve()
    thrower_to_catcher = komo_1.getPath()

    if debug:
        print("KOMO_1 is feasible" if ret_1.feasible else "KOMO_1 is NOT feasible")

    # Solve RRT problem for collision-free path
    ry.params_clear()
    ry.params_add({'rrt/stepsize': 0.1, 'rrt/verbose': 0})
    foundRRTPath = False
    numOfTrials = 0

    while not foundRRTPath and numOfTrials < max_trials:
        rrt = ry.PathFinder()
        rrt.setProblem(C, [q0], [thrower_to_catcher[0]])
        ret_RRT_1 = rrt.solve()
        path_1 = ret_RRT_1.x
        foundRRTPath = ret_RRT_1.feasible
        numOfTrials += 1

    if debug:
        print("*** Found path at trial num #", numOfTrials, "***" if foundRRTPath else "** No path found ***")

    # Render the simulation if enabled
    if isRender:
        for t in range(path_1.shape[0] - 1):
            C.setJointState(path_1[t])
            C.view()
            time.sleep(0.01)

    # Cleanup
    C.delFrame(target_fname)
    C.getFrame(thrower_fname).unLink()
    del komo_1


def check_if_throw_is_obstructed(thrower_fname, catcher_fname, C, threshold=0.5):
    """
    Check if the throw path between the thrower and catcher is obstructed.

    Args:
        thrower_fname (str): Frame name of the thrower robot.
        catcher_fname (str): Frame name of the catcher robot or bin.
        C (ry.Config): Configuration object for the robotic setup.
        threshold (float): Distance threshold to determine obstruction.

    Returns:
        bool: True if the throw is obstructed, False otherwise.
    """
    thrower = C.getFrame(thrower_fname)
    catcher = C.getFrame(catcher_fname)

    # Compute distance between thrower and catcher
    dist = np.linalg.norm(thrower.getPosition() - catcher.getPosition())
    return dist >= threshold


if __name__ == "__main__":
    # Main execution for demonstration purposes
    C = ry.Config()
    C.addFile("catching.g")

    thrower_fname = "l_panda_base"
    catcher_fname = "l2_panda_base"

    # Fetch initial positions
    thrower = C.getFrame(thrower_fname)
    catcher = C.getFrame(catcher_fname)
    initial_thrower_pos = thrower.getPosition()
    initial_catcher_pos = catcher.getPosition()

    print("Initial thrower position:", initial_thrower_pos)
    print("Initial catcher position:", initial_catcher_pos)

    # Place walls to simulate obstacles
    wall = C.addFrame("wall", "base") \
        .setPosition(initial_thrower_pos + np.array([-1, 0.75, 0.8])) \
        .setShape(ry.ST.ssBox, [0.1, 2, 2, 0.02]) \
        .setColor([0, 1, 0]) \
        .setContact(1)

    isObstructed = check_if_throw_is_obstructed(thrower_fname, catcher_fname, C)
    print("Is throw obstructed:", isObstructed)

    if isObstructed:
        move_robot_closer(thrower_fname, catcher_fname, ["wall"], C, isRender=True)

    C.view()
    time.sleep(5)

    del C
