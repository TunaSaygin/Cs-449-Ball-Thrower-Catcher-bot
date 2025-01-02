import robotic as ry
import numpy as np
from my_utils import rotation_matrix_to_quaternion
import time
from velocity_finder import find_velocity, pick_last_object_if_valid
from my_utils import get_quat_from_velocity
import json
from scipy.spatial import cKDTree
## this file is created to create generic bin position and get the expected results.


def throw_sample(bin_new_position:list,isRender:bool,sleep_time:float = 20, bin_shape = [0.5,0.5]):
    C = ry.Config()
    C.addFile("throwing_bare.g")
    print(f"Initial bin pos:{C.getFrame('bin').getPosition()}")
    init_environment(C,bin_new_position,bin_shape)
    print(C.eval(ry.FS.position,["l_gripper"]))
    bot = ry.BotOp(C, useRealRobot=False)
    estimated_trajectory_array = []
    release_velocity,isInverted = find_velocity(C)
    initial_position = pick_last_object_if_valid(C,C.getFrame("release_frame").getPosition(),release_velocity,estimated_trajectory_array)
    # print(f"Estimated trajectory array:{estimated_trajectory_array}")
    #invertion deviation calculation
    time_deviation = grasp_object(C,bot)
    release_position = C.getFrame("release_frame").getPosition()
    target_distance = np.sqrt(np.square(release_position[0]-bin_new_position[0]) + np.square(release_position[1]-bin_new_position[1]))
    print(f"Final bin pos:{C.getFrame('bin').getPosition()}")
    #elaborating on the release time calculation of the thrower robot
    print(f"release velocity: {release_velocity} & target distance: {target_distance}")
    time_delay = 0.0
    epsilon = 0.0003
    if target_distance<=1.5:
        time_delay = 0.07
    elif target_distance<=2:
        time_delay = 0.03
    elif target_distance<=3:
        time_delay = 0.02
    wanted_sleep = 0.57 + time_delay
    time_sleep = time_deviation * wanted_sleep
    ("printing Starting to throw")
    time.sleep(1)
    ball_trajectory_array = []
    cargo_height = C.getFrame("cargo").getSize()[2]
    throw_object(C,bot,time_sleep,release_velocity,ball_trajectory_array)
    print(f"time delay target_distance<1: {time_delay}")
    result,deviation = check_in_the_bin(C,bot,bin_new_position,C.getFrame("side2").getSize()[0]/2,C.getFrame("side2").getSize()[2],cargo_height,ball_trajectory_array)

    print(f"Result:{result} for bin pos:{bin_new_position}")
    render_actual_trajectory(C,1,ball_trajectory_array)
    trajectory_deviations,interpolated_trajectory,_ = calculate_deviation(ball_trajectory_array,estimated_trajectory_array,bin_new_position[2])
    render_actual_trajectory(C,2,interpolated_trajectory,[1,1,0])
    print(f"Trajectory deviation : {trajectory_deviations}")
    if isRender:
        C.view()
        time.sleep(sleep_time)
    del C
    del bot
    return result,deviation,trajectory_deviations
def render_actual_trajectory(C:ry.Config,id, ball_trajectory_array:list, color:list= [0.6,0.2,0.3]):
    for it,point in enumerate(ball_trajectory_array):
        C.addFrame(f"point-{id}-{it}").setShape(ry.ST.marker,[.3]).setPosition(point).setColor(color)

import numpy as np
from scipy.interpolate import interp1d

def calculate_deviation(position_red, position_blue, height_threshold):
    """
    Calculate RMS trajectory deviation between two sets of positions after filtering and matching.

    Parameters:
    - position_red: np.array of shape (n, 3) for the red trajectory positions (x, y, z).
    - position_blue: np.array of shape (m, 3) for the blue trajectory positions (x, y, z).
    - height_threshold: float, minimum height for filtering points.

    Returns:
    - rms_deviations: float, root mean square deviation.
    - matched_red: np.array of filtered and matched red trajectory positions.
    - matched_blue: np.array of filtered and matched blue trajectory positions.
    """
    def filter_points(array, height_threshold):
        """Filter points based on the height threshold."""
        return np.array([point for point in array if point[2] >= height_threshold])

    # Filter the points based on the height threshold
    position_red = filter_points(position_red, height_threshold)
    position_blue = filter_points(position_blue, height_threshold)
    # print(f"Position_blue{position_blue} and position_red:{position_red}")
    if len(position_red) == 0 or len(position_blue) == 0:
        raise ValueError("One or both trajectories have no points above the height threshold.")

    # Use a KD-tree to match each point in position_red to the closest point in position_blue
    kdtree_blue = cKDTree(position_blue)
    distances, indices = kdtree_blue.query(position_red)

    # Get the matched positions
    matched_red = position_red
    matched_blue = position_blue[indices]

    # Calculate point-to-point deviations
    deviations = np.linalg.norm(matched_red - matched_blue, axis=1)

    # Compute the RMS deviation
    rms_deviations = np.sqrt(np.mean(np.square(deviations)))

    return rms_deviations, matched_red, matched_blue

def check_positions_equal(position1:np.ndarray,positon2:np.ndarray)->bool:
    return position1[0] == positon2[0] and positon2[1] == position1[1] and position1[2] == positon2[2]
def check_in_the_bin(C: ry.Config, bot: ry.BotOp, bin_center, binxy_length, bin_height, cargo_height, ball_trajectory_array:list):
    start = time.time()
    prev_pos = np.array(C.getFrame("cargo").getPosition())
    bot.sync(C, .01)
    cargo_pos = np.array(C.getFrame("cargo").getPosition())
    
    # Wait for the cargo to stabilize or timeout
    while (not check_positions_equal(prev_pos,cargo_pos)) and (prev_pos[2]>=cargo_height and cargo_pos[2]>= cargo_height):
        # print(f"prev_pos = {prev_pos} vs. cargo_pos = {cargo_pos} | equal?{check_positions_equal(prev_pos,cargo_pos)}")
        prev_pos = cargo_pos
        bot.sync(C, .03)
        ball_trajectory_array.append(cargo_pos)
        cargo_pos = np.array(C.getFrame("cargo").getPosition())
        if time.time() - start > 8:
            break
    
    print(f"Final cargo position: {cargo_pos}")
    print(f"bin_center[0]-binxy_length <= cargo_pos[0] <= bin_center[0] + binxy_length = "
          f"{bin_center[0]-binxy_length} <= {cargo_pos[0]} <= {bin_center[0] + binxy_length}")
    print(f"bin_center[1]-binxy_length <= cargo_pos[1] <= bin_center[1] + binxy_length = "
          f"{bin_center[1]-binxy_length} <= {cargo_pos[1]} <= {bin_center[1] + binxy_length}")
    print(f"bin_center[2] <= cargo_pos[2] <= bin_height = {bin_center[2]} <= {cargo_pos[2]} <= {bin_center[2] + bin_height}")
    
    # Calculate deviation
    deviation = np.linalg.norm(cargo_pos - np.array(bin_center))
    print(f"Deviation from target position: {deviation:.4f} meters")

    if bin_center[0]-binxy_length <=cargo_pos[0] <= bin_center[0] + binxy_length and bin_center[1]-binxy_length <=cargo_pos[1] <= bin_center[1] + binxy_length\
        and bin_center[2]<=cargo_pos[2]<=bin_center[2] + bin_height:
        return True, deviation
    return False, deviation

def throw_object(C,bot,time_sleep,velocity, ball_trajectory_array):
    cargo_frame = C.getFrame("cargo")
    print(f"velocity is !!!: {velocity}")
    # new throw point calculation. Because in case of inversion the robot deviates a little
    def vel_komo():
        q0 = C.getJointState()
        komo = ry.KOMO(C, 1, 1, 1, True)
        # komo.addObjective([], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e-1], [4.2,-3.5,4],1)
        komo.addObjective([], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e-1], np.array(velocity),1)
        komo.addObjective([],ry.FS.positionDiff,["l_gripper","release_frame"],ry.OT.sos,[1e0],[0,0,0])
        ret2 = ry.NLP_Solver(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=0).solve()
        #komo.addObjective([1.],ry.FS.scalarProductXZ,["base","l_gripper"],ry.OT.eq,[1e1],[-1],0)
        #print(komo.report())
        print(f"ret!!!: {ret2}")
        return komo
    komo = vel_komo()
    gripper_frame = C.getFrame("l_gripper")
    # bot = ry.BotOp(C,False)
    # time.sleep(5)
    path = komo.getPath()
    print(f"path size: {path.size}")
    bot.move(path,[1.])
    print(f"bot initial end time:{bot.getTimeToEnd()}")
    time.sleep(time_sleep)
    bot.sync(C,0.001)
    print(f"bot end time:{bot.getTimeToEnd()}")
    bot.gripperMove(ry._left,width=1)
    C.addFrame("actual_release").setPosition(gripper_frame.getPosition()).setShape(ry.ST.marker,[.2]).setColor([1,1,0])
    print(f"bot end time:{bot.getTimeToEnd()}")

    while bot.getTimeToEnd()>0:
        ball_trajectory_array.append(cargo_frame.getPosition())
        bot.sync(C,.03)
def grasp_object(C:ry.Config, bot:ry.BotOp,object_name:str="cargo"):
    q0 = qHome = C.getJointState()
    komo_pre_grasp = pre_grasp_komo(C,"l_gripper",object_name,q0,qHome)
    path_pre_grasp = komo_pre_grasp.getPath()
    komo_post_grasp = post_grasp_komo(C,qHome)
    path_post_grasp = komo_post_grasp.getPath()
    shape = path_pre_grasp.shape[0]*0.1
    print(path_pre_grasp.shape)
    bot.move(path_pre_grasp, [1.])
    start = bot.getTimeToEnd()
    time.sleep(shape)
    end = bot.getTimeToEnd()
    while bot.getTimeToEnd() > 0:
        bot.sync(C, .1)
    bot.gripperClose(ry._left,width=0.06) 
    while not bot.gripperDone(ry._left):
        bot.sync(C, .1)
    bot.move(path_post_grasp, [3.])
    while bot.getTimeToEnd() > 0:
        bot.sync(C, .1)
    C.attach('l_gripper', 'cargo')
    time_deviation = shape/(start-end)
    return time_deviation
def pre_grasp_komo(C, gripper_name, grasp_frame_name, q0, qHome):
    komo = ry.KOMO(C, 3, 1, 0, True)
    # Suppose at some point you know positions:
    gripper_pos = C.getFrame("l_gripper").getPosition()
    bin_pos = C.getFrame("bin").getPosition()
    dx = bin_pos[0] - gripper_pos[0]
    dy = bin_pos[1] - gripper_pos[1]
    direction = np.array([dx, dy, 0])
    direction = direction / np.linalg.norm(direction)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, [1e1])
    komo.addObjective([], ry.FS.jointState, [], ry.OT.sos, [1e-1], q0)
    komo.addObjective([], ry.FS.jointState, [], ry.OT.sos, [1e-1], qHome)

    komo.addObjective([1., 3.], ry.FS.positionRel, [gripper_name, grasp_frame_name], ry.OT.eq, [1e1], [0, 0, 0])  
    komo.addObjective([1., 3.], ry.FS.scalarProductYX, [gripper_name, grasp_frame_name], ry.OT.eq, [1e2], [1]) 
    komo.addObjective([1., 3.], ry.FS.scalarProductZZ, [grasp_frame_name, gripper_name], ry.OT.eq, [1e1], [1]) 

    ret = ry.NLP_Solver(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=0).solve()
    print(ret)
    return komo
def post_grasp_komo(C:ry.Config, home)->ry.KOMO:
    # I assume that robot is facing 1 0 0 direction
    z_vector= np.array([0,0,1])
    robot_base = np.array(C.getFrame("l_panda_base").getPosition())
    initial_position  = np.array(C.getFrame("initial_position").getPosition())
    initial_position = initial_position-robot_base
    # initial_position = initial_position-robot_base
    C.addFrame("marker_coll3","l_panda_coll3").setShape(ry.ST.marker,[.3]).setColor([1,0,0])
    C.addFrame("marker_coll5","l_panda_coll5").setShape(ry.ST.marker,[.3]).setColor([0,1,0])
    C.addFrame("marker_coll7","l_panda_coll7").setShape(ry.ST.marker,[.4]).setColor([0,0,1])
    # orthogonal = np.cross(np.array([initial_position[0],initial_position[1],0]),z_vector)
    komo = ry.KOMO(C, 1, 1, 0, True)
    komo.addObjective([],ry.FS.vectorX,["l_panda_coll3"],ry.OT.eq,[initial_position[0],initial_position[1],0])
    komo.addObjective([],ry.FS.vectorX,["l_panda_coll5"],ry.OT.eq,[initial_position[0],initial_position[1],0])
    komo.addObjective([], ry.FS.scalarProductXZ, ["initial_position","l_panda_coll5"], ry.OT.eq, [1e1], [-1])
    komo.addObjective([], ry.FS.scalarProductXZ, ["initial_position","l_panda_coll3"], ry.OT.eq, [1e1], [-1])
    komo.addObjective([], ry.FS.scalarProductXX, ["initial_position","l_panda_coll7"], ry.OT.eq, [1e1], [1])
    # komo.addObjective([], ry.FS.scalarProductXX, ["initial_position","l_gripper"], ry.OT.eq, [1e1], [-1])
    ret = ry.NLP_Solver(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=0).solve()
    print(ret)
    return komo
def init_environment(C:ry.Config,bin_new_position:list,bin_shape):
    if bin_new_position[2]<0.08:
        bin_new_position[2] = 0.08 #just to ensure that bin is not immersed to the ground.
    qHome = C.getJointState()
    C.getFrame('cargo').unLink() # to remove a bug
    base_position = np.array(C.getFrame("l_panda_base").getPosition())
    bin_new_position = np.array(bin_new_position)
    new_quat = rotate_bin(bin_new_position,base_position)
    C.addFrame("throw").setPosition([0,1.8,1.2]).setShape(ry.ST.marker,[.3]).setQuaternion(new_quat)
    C.addFrame("release_frame").setPosition([0,0,0]).setShape(ry.ST.marker,[.4]).setColor([1,0,0]).setContact(0)
    if bin_new_position[2] >0.08: #create a support block to hold bin above the ground
        height = bin_new_position[2]-0.025
        C.addFrame("bin-support")\
            .setPosition([bin_new_position[0],bin_new_position[1],height/2]).setShape(ry.ST.ssBox,[bin_shape[0]+.001,bin_shape[1]+.001,height,0])\
            .setQuaternion(new_quat).setColor([0,0,0])
        center = np.array([bin_new_position[0],bin_new_position[1],height])
    C.getFrame('bin').setPosition(bin_new_position).setQuaternion(new_quat)
    C.view()



def rotate_bin(bin_position:np.ndarray, base_position:np.ndarray):
    vector_base_bin = bin_position-base_position
    xy_distance = np.sqrt(np.square(vector_base_bin[0])+np.square(vector_base_bin[1]))
    vector_base_bin_normalized = vector_base_bin/xy_distance
    bin_x_vector = np.array([vector_base_bin_normalized[0],vector_base_bin_normalized[1],0])
    bin_z_vector = np.array([0,0,1])
    #let's find the bin-y axis
    bin_y_vector = np.cross(bin_z_vector,bin_x_vector)

    #with them I can create rotation matrix
    rotation_matrix = np.column_stack((bin_x_vector, bin_y_vector, bin_z_vector)) # what it does is that it transposes each array and makes new matrix from these row vectors
    new_quaternion = rotation_matrix_to_quaternion(rotation_matrix)
    return new_quaternion




def generate_homogeneous_points(
    robo_base, carpet_center, carpet_len, range_limit, z_min, z_max, num_points, grid_resolution=10
):
    points = []
    x_min = robo_base[0] - range_limit
    x_max = robo_base[0] + range_limit
    y_min = robo_base[1] - range_limit
    y_max = robo_base[1] + range_limit

    # Generate grid boundaries
    x_bins = np.linspace(x_min, x_max, grid_resolution)
    y_bins = np.linspace(y_min, y_max, grid_resolution)

    # Calculate the number of points per grid cell
    total_cells = (grid_resolution - 1) ** 2
    points_per_cell = num_points // total_cells

    for i in range(len(x_bins) - 1):
        for j in range(len(y_bins) - 1):
            cell_points = 0
            max_attempts = 1000  # Limit attempts to prevent infinite loop
            attempts = 0

            while cell_points < points_per_cell and attempts < max_attempts:
                attempts += 1
                # Sample random x, y, and z within the current grid cell
                x = np.random.uniform(x_bins[i], x_bins[i + 1])
                y = np.random.uniform(y_bins[j], y_bins[j + 1])
                z = np.random.uniform(z_min, z_max)

                # Check if point is outside the blue carpet area
                if (
                    (x < carpet_center[0] - carpet_len / 2 or x > carpet_center[0] + carpet_len / 2)
                    or (y < carpet_center[1] - carpet_len / 2 or y > carpet_center[1] + carpet_len / 2)
                ):
                    points.append((x, y, z))
                    cell_points += 1

            if attempts == max_attempts:
                print(f"Warning: Could not generate enough points in cell ({i}, {j})")

    # Fill any remaining points due to rounding
    while len(points) < num_points:
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)
        z = np.random.uniform(z_min, z_max)
        if (
            (x < carpet_center[0] - carpet_len / 2 or x > carpet_center[0] + carpet_len / 2)
            or (y < carpet_center[1] - carpet_len / 2 or y > carpet_center[1] + carpet_len / 2)
        ):
            points.append((x, y, z))

    return points

#for testing this module
if __name__=="__main__":
    throw_sample([
           -1.681301052534445,
            0.1950987908494636,
            0.26589367752114873
        ],True,sleep_time=25)