"""supervisor_controller controller."""

# You may need to import some classes of the controller module. Ex:

from controller import Supervisor
import sys
import random
import math

TIME_STEP = 16

supervisor = Supervisor()

board_x_max = 1.2
board_x_min = -1.2
board_z_max = 1.2
board_z_min = -1.2

start_square_size = 0.4
start_square_offset_z = 0.4

block_size = 0.5

steady_state_time = 10 # s
steady_state_thresh = 0.05 # m

def get_target_nodes(supervisor, namebase):
    root = supervisor.getRoot()
    children = root.getField("children")
    numChildren = children.getCount()

    blocks = []

    for i in range(numChildren):
        name_field = children.getMFNode(i).getField("name")
        if name_field is not None:
            if namebase in name_field.getSFString():
                blocks.append(children.getMFNode(i))

    return blocks

def get_robot_gps_nodes(supervisor, green_name, red_name):
    root = supervisor.getRoot()
    children = root.getField("children")
    numChildren = children.getCount()

    green_robot = None
    red_robot = None

    for i in range(numChildren):
        name_field = children.getMFNode(i).getField("name")
        if name_field is not None:
            if name_field.getSFString() == green_name:
                green_robot = children.getMFNode(i)
            elif name_field.getSFString() == red_name:
                red_robot = children.getMFNode(i)

    return green_robot, red_robot

def assign_random_block_positions(blocks, wall_buffer):
    for block in blocks:
        trans_field = block.getField("translation")
        x = 0
        z = start_square_offset_z
        while ((abs(x) < start_square_size/2) and
               (abs(z - start_square_offset_z) < start_square_size/2 or abs(z + start_square_offset_z) < start_square_size/2)):
            x = random.uniform(board_x_min + wall_buffer, board_x_max - wall_buffer)
            z = random.uniform(board_z_min + wall_buffer, board_z_max - wall_buffer)
        trans_field.setSFVec3f([x, 0.04, z])

def find_robot_gps(supervisor):
    robotDef = "IDPROBOT"
    gpsDef = "ROBOT_GPS"

    robot_node = supervisor.getFromDef(robotDef)
    if robot_node is None:
        print("couldnt find robot")
        return None

    gps_node = robot_node.getFromProtoDef(gpsDef)
    if gps_node is None:
        print("couldnt find gps!")
        return None

    return gps_node

def vector_distance(vec1, vec2):
    return math.sqrt((vec1[0] - vec2[0] ) ** 2 + (vec1[2] - vec2[2]) ** 2)

blocks = get_target_nodes(supervisor, "target")
assign_random_block_positions(blocks, 0.05)

red_robot, green_robot = get_robot_gps_nodes(supervisor, "green_robot", "red_robot")
red_trans = red_robot.getField("translation")
green_trans = green_robot.getField("translation")

red_trans_vec_last = red_trans.getSFVec3f()
green_trans_vec_last = green_trans.getSFVec3f()
red_trans_vec = red_trans_vec_last
green_trans_vec = green_trans_vec_last

steady_state_counter = 0

while supervisor.step(TIME_STEP) != -1:
    steady_state_counter += 1
    if (vector_distance(red_trans_vec_last, red_trans_vec) > steady_state_thresh and
        vector_distance(green_trans_vec_last, green_trans_vec) > steady_state_thresh):
        red_trans_vec_last = red_trans_vec
        green_trans_vec_last = green_trans_vec
        steady_state_counter = 0

    red_trans_vec = red_trans.getSFVec3f()
    green_trans_vec = green_trans.getSFVec3f()

    if (steady_state_counter > steady_state_time * 1000 / TIME_STEP):
        print("steady state reached")



