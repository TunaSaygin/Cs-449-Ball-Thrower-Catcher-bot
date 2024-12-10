import robotic as ry
import numpy as np
from my_utils import rotation_matrix_to_quaternion
import time
from velocity_finder import find_velocity, pick_last_object_if_valid
from my_utils import get_quat_from_velocity
## this file is created to create generic bin position and get the expected results.


def throw_sample(bin_new_position:list,isRender:bool,sleep_time:float = 20, bin_shape = [0.5,0.5]):
    C = ry.Config()
    C.addFile("throwing_bare.g")
    print(f"Initial bin pos:{C.getFrame('bin').getPosition()}")
    init_environment(C,bin_new_position,bin_shape)
    if isRender:
        C.view()
        time.sleep(sleep_time)
    bot = ry.BotOp(C, useRealRobot=False)
    release_velocity = find_velocity(C)
    initial_position = pick_last_object_if_valid(C,C.getFrame("release_frame").getPosition(),release_velocity)
    grasp_object(C,bot)
    print(f"Final bin pos:{C.getFrame('bin').getPosition()}")
    if isRender:
        time.sleep(sleep_time)
    del C
def grasp_object(C:ry.Config, bot:ry.BotOp,object_name:str="cargo"):
    q0 = qHome = C.getJointState()
    komo_pre_grasp = pre_grasp_komo(C,"l_gripper",object_name,q0,qHome)
    path_pre_grasp = komo_pre_grasp.getPath()
    komo_post_grasp = post_grasp_komo(C,"l_gripper",object_name,q0,qHome)
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
def post_grasp_komo(C, gripper_name, grasp_frame_name, q0, qHome)->ry.KOMO:
    komo = ry.KOMO(C, 1, 1, 0, True)
    komo.addObjective([], ry.FS.positionDiff, ["l_gripper","initial_position"], ry.OT.eq, [1e1], [0,0,0])
    komo.addObjective([], ry.FS.scalarProductYZ, ["l_gripper","initial_position"], ry.OT.eq, [1e1], [-1])
    ret = ry.NLP_Solver(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=4).solve()
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


#for testing this module
if __name__=="__main__":
    throw_sample([3,3,0.9],True,sleep_time=10)