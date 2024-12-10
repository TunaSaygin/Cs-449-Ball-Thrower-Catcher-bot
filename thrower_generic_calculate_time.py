import robotic as ry
import numpy as np
from my_utils import rotation_matrix_to_quaternion
import time
from velocity_finder import find_velocity, pick_last_object_if_valid
from my_utils import get_quat_from_velocity
import json
## this file is created to create generic bin position and get the expected results.

def simulate_throw_time(C, bot, velocity):
    """Simulate throw timing to calculate bot's movement time."""
    def vel_komo():
        komo = ry.KOMO(C, 1, 1, 1, True)
        komo.addObjective([], ry.FS.position, ["l_gripper"], ry.OT.eq, [1e-1], np.array(velocity), 1)
        komo.addObjective([], ry.FS.positionDiff, ["l_gripper", "release_frame"], ry.OT.sos, [1e0], [0, 0, 0])
        ry.NLP_Solver(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=0).solve()
        return komo

    komo = vel_komo()
    path = komo.getPath()
    bot.move(path, [1.0])
    while bot.getTimeToEnd() > 0:
        bot.sync(C, 0.1)
    # Return the total time for the motion
    return bot.getTimeToEnd()

def throw_sample(bin_new_position:list,isRender:bool,sleep_time:float = 20, bin_shape = [0.5,0.5]):
    C = ry.Config()
    C.addFile("throwing_bare.g")
    print(f"Initial bin pos:{C.getFrame('bin').getPosition()}")
    init_environment(C,bin_new_position,bin_shape)
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
    wanted_sleep = 0.75
    time_sleep = time_deviation * wanted_sleep
    throw_object(C,bot,time_sleep,release_velocity)
    cargo_height = C.getFrame("cargo").getSize()[2]
    result = check_in_the_bin(C,bot,bin_new_position,C.getFrame("side2").getSize()[0]/2,C.getFrame("side2").getSize()[2],cargo_height)

    print(f"Result:{result} for bin pos:{bin_new_position}")

    if isRender:
        C.view()
        time.sleep(sleep_time)
    del C
    del bot
    return result
def check_in_the_bin(C:ry.Config,bot:ry.BotOp,bin_center,binxy_length,bin_height,cargo_heigth):
    start = time.time()
    prev_pos = np.array(C.getFrame("cargo").getPosition())
    bot.sync(C,.01)
    cargo_pos = np.array(C.getFrame("cargo").getPosition())
    while not np.array_equal(prev_pos,cargo_pos) or (prev_pos[2]<cargo_heigth and cargo_pos[2]<cargo_heigth):
        prev_pos = cargo_pos
        bot.sync(C,.01)
        cargo_pos = np.array(C.getFrame("cargo").getPosition())
        if time.time() - start > 8:
            break
    print(cargo_pos)
    print(f"bin_center[0]-binxy_length <=cargo_pos[0] <= bin_center[0] + binxy_length = {bin_center[0]-binxy_length}<={cargo_pos[0]}<={bin_center[0] + binxy_length}")
    print(f"bin_center[1]-binxy_length <=cargo_pos[1] <= bin_center[1] + binxy_length = {bin_center[1]-binxy_length}<={cargo_pos[1]}<={bin_center[1] + binxy_length}")
    print(f"bin_center[2]<=cargo_pos[2]<=bin_height = {bin_center[2]}<={cargo_pos[2]}<={bin_height}")
    if bin_center[0]-binxy_length <=cargo_pos[0] <= bin_center[0] + binxy_length and bin_center[1]-binxy_length <=cargo_pos[1] <= bin_center[1] + binxy_length\
        and bin_center[2]<=cargo_pos[2]<=bin_center[2] + bin_height:
        return True
    return False

def throw_object(C,bot,time_sleep,velocity):
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
        bot.sync(C,.1)
def grasp_object(C:ry.Config, bot:ry.BotOp,isInverted:bool,deviation_arr=None,object_name:str="cargo"):
    q0 = qHome = C.getJointState()
    komo_pre_grasp = pre_grasp_komo(C,"l_gripper",object_name,q0,qHome)
    path_pre_grasp = komo_pre_grasp.getPath()
    komo_post_grasp = post_grasp_komo(C,isInverted,deviation_arr)
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
def post_grasp_komo(C,isInverted:bool,deviationarr=None)->ry.KOMO:

    komo = ry.KOMO(C, 1, 1, 0, True)
    komo.addObjective([], ry.FS.positionDiff, ["l_gripper","initial_position"], ry.OT.sos, [1e0], [0,0,0] if not isInverted else deviationarr)
    komo.addObjective([], ry.FS.scalarProductYZ, ["l_gripper","base"], ry.OT.eq, [1e1], [-1])
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
    
    throw_sample([-1,0.5,0.3],True,sleep_time=10)