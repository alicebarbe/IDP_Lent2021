"""supervisor_controller controller."""

# You may need to import some classes of the controller module. Ex:

from controller import Supervisor, Receiver
import sys
import random
import math
import struct
import os
import pandas as pd

# constants ===
TIME_STEP = 16
board_x_max = 1.2
board_x_min = -1.2
board_z_max = 1.2
board_z_min = -1.2

start_square_size = 0.4
start_square_offset_z = 0.4

block_size = 0.5

steady_state_time = 10 # s
steady_state_thresh = 0.05 # m

DATA_SAVE_DIR = "./Data/Test/"
NUM_SIMULATIONS = 25
# ===

supervisor = Supervisor()
data = pd.DataFrame(columns=["TimeToSteadyState", "GreenBlocksIdentified", "RedBlocksIdentified", "GreenBlocksCollected", "RedBlocksCollected"])


# get webots objects ===
def setup_receiver(name):
    receiver = supervisor.getDevice(name)
    receiver.enable(TIME_STEP)
    return receiver

def receive_messages(receiver):
    messages = []
    while receiver.getQueueLength() > 0:
        messages.append(struct.unpack('<ddi', receiver.getData()))
        receiver.nextPacket()

    return messages

def get_node_by_name(supervisor, name):
    root = supervisor.getRoot()
    if root is None:
        print("root is none")
        return None
    children = root.getField("children")
    if children is None:
        print("Root has no children")
        return None
    numChildren = children.getCount()

    for i in range(numChildren):
        name_field = children.getMFNode(i).getField("name")
        if name_field is not None:
            if name_field.getSFString() == name:
                return children.getMFNode(i)

    return None

def get_target_nodes(supervisor):
    blocks = []
    target_names = ["target({})".format(i) for i in range(1, 10)]
    print(target_names)
    return list(filter(None, [get_node_by_name(supervisor, name) for name in target_names]))

def reset_simulation():
    supervisor.simulationReset()
    server.restartController()
    red_robot.restartController()
    green_robot.restartController()
# ===
# manipulate blocks ===

def assign_random_block_positions(blocks, wall_buffer):
    for block in blocks:
        print("working")
        trans_field = block.getField("translation")
        x = 0
        z = start_square_offset_z
        while ((abs(x) < start_square_size/2) and
               (abs(z - start_square_offset_z) < start_square_size/2 or abs(z + start_square_offset_z) < start_square_size/2)):
            x = random.uniform(board_x_min + wall_buffer, board_x_max - wall_buffer)
            z = random.uniform(board_z_min + wall_buffer, board_z_max - wall_buffer)
        trans_field.setSFVec3f([x, 0.04, z])

def count_blocks_in_correct_square(blocks):
    red_blocks_returned = 0
    green_blocks_returned = 0

    for block in blocks:
        trans_field = block.getField("translation")
        colour_field = block.getField("colour")
        x, _, z = trans_field.getSFVec3f()

    if ((abs(x) < start_square_size / 2) and
           (abs(z - start_square_offset_z) < start_square_size / 2) and
           colour_field.getSFColor()[0] == 1):
        # red block in red start_square
        red_blocks_returned += 1

    elif ((abs(x) < start_square_size / 2) and
           (abs(z + start_square_offset_z) < start_square_size / 2) and
           colour_field.getSFVec3f()[0] == 1):
        # red block in red start_square
        green_blocks_returned += 1

    return green_blocks_returned, red_blocks_returned
# ===
# Utilities ===

def vector_distance(vec1, vec2):
    return math.sqrt((vec1[0] - vec2[0] ) ** 2 + (vec1[2] - vec2[2]) ** 2)

# ===

for sim_num in range(NUM_SIMULATIONS):
    print("Restarting simulation ...")

    timeout = 0
    while supervisor.step(TIME_STEP) != -1 and timeout < 10:
        timeout += 1

    blocks = get_target_nodes(supervisor)
    print(blocks[0].getField("colour"))
    assign_random_block_positions(blocks, 0.05)

    receiver = setup_receiver("receiver")

    red_robot = get_node_by_name(supervisor, "red_robot")
    green_robot = get_node_by_name(supervisor, "green_robot")
    server = get_node_by_name(supervisor, "laptop")
    print(server)

    red_trans = red_robot.getField("translation")
    green_trans = green_robot.getField("translation")

    red_trans_vec_last = red_trans.getSFVec3f()
    green_trans_vec_last = green_trans.getSFVec3f()
    red_trans_vec = red_trans_vec_last
    green_trans_vec = green_trans_vec_last

    steady_state_counter = 0
    time_to_steady_state = 0

    green_blocks_identified = 0
    red_blocks_identified = 0

    timeout = 0
    while supervisor.step(TIME_STEP) != -1 and timeout < 20:
        timeout += 1

    supervisor.exportImage(os.path.join(DATA_SAVE_DIR, "{}_start_{}.jpg".format("test_image_export", sim_num)), 90)

    while supervisor.step(TIME_STEP) != -1:

        red_trans_vec = red_trans.getSFVec3f()
        green_trans_vec = green_trans.getSFVec3f()

        steady_state_counter += 1
        if (vector_distance(red_trans_vec_last, red_trans_vec) > steady_state_thresh or
            vector_distance(green_trans_vec_last, green_trans_vec) > steady_state_thresh):
            red_trans_vec_last = red_trans_vec
            green_trans_vec_last = green_trans_vec
            steady_state_counter = 0

        if (steady_state_counter > steady_state_time * 1000 / TIME_STEP):
            print("steady state reached")
            time_to_steady_state = supervisor.getTime() - steady_state_time
            supervisor.exportImage(os.path.join(DATA_SAVE_DIR, "{}_end_{}.jpg".format("test_image_export", sim_num)), 90)
            break

        for message in receive_messages(receiver):
            if message[2] == 130:
                green_blocks_identified += 1
            elif message[2] == 240:
                red_blocks_identified += 1

    green_blocks_returned, red_blocks_returned = count_blocks_in_correct_square(blocks)
    data.loc[sim_num] = [time_to_steady_state, green_blocks_identified, red_blocks_identified, green_blocks_returned, red_blocks_returned]
    reset_simulation()

supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
data.to_csv(os.path.join(DATA_SAVE_DIR, "test_csv_save.csv"))
del supervisor

