"""supervisor_controller controller."""

# You may need to import some classes of the controller module. Ex:

from controller import Supervisor, Receiver
import sys
import random
import math
import struct
import os
import pandas as pd

from utility import TestUtility

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
max_sim_time = 300 # s  stop if robots dont reach steady state by this time

DATA_SAVE_DIR = "./Data/FinalTest/"
NUM_SIMULATIONS = 100
# ===

supervisor = Supervisor()
testUtility = TestUtility(TIME_STEP, supervisor)
data = pd.DataFrame(columns=["TimeToSteadyState", "GreenBlocksIdentified", "RedBlocksIdentified", "GreenBlocksCollected", "RedBlocksCollected"])
lost_block_data = pd.DataFrame(columns=["GreenLostAfterTurn", "RedLostAfterTurn",
                                        "GreenRefoundAfterTurn", "RedRefoundAfterTurn",
                                        "GreenLostDuringTweaking", "RedLostDuringTweaking",
                                        "GreenCouldntMeasureColour", "RedCouldntMeasureColour",
                                        "GreenBlocksFoundAndMeasured", "RedBlocksFoundAndMeasured"])


def vector_distance(vec1, vec2):
    return math.sqrt((vec1[0] - vec2[0]) ** 2 + (vec1[2] - vec2[2]) ** 2)


for sim_num in range(NUM_SIMULATIONS):
    print("Restarting simulation ...")

    timeout = 0
    while supervisor.step(TIME_STEP) != -1 and timeout < 10:
        timeout += 1

    testUtility.assign_random_block_positions(0.05)

    red_trans, green_trans = testUtility.get_robot_fields("translation")

    red_trans_vec_last = red_trans.getSFVec3f()
    green_trans_vec_last = green_trans.getSFVec3f()
    red_trans_vec = red_trans_vec_last
    green_trans_vec = green_trans_vec_last

    steady_state_counter = 0
    time_to_steady_state = 0

    green_blocks_identified = 0
    red_blocks_identified = 0

    # losing block count
    red_cant_find_colour = 0
    green_cant_find_colour = 0
    red_lost_during_tweaking = 0
    green_lost_during_tweaking = 0
    red_lost_after_turn = 0
    green_lost_after_turn = 0
    red_refound_after_turn = 0
    green_refound_after_turn = 0
    red_total_blocks_found = 0
    green_total_blocks_found = 0

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

        if (steady_state_counter > steady_state_time * 1000 / TIME_STEP or supervisor.getTime() == max_sim_time):
            print("steady state reached")
            time_to_steady_state = supervisor.getTime() - steady_state_time
            supervisor.exportImage(os.path.join(DATA_SAVE_DIR, "{}_end_{}.jpg".format("test_image_export", sim_num)), 90)
            break

        for message in testUtility.receive_messages():
            if message[2] == 130:
                green_blocks_identified += 1
            elif message[2] == 240:
                red_blocks_identified += 1

            elif message[2] == 171:
                green_cant_find_colour += 1
            elif message[2] == 271:
                red_cant_find_colour += 1

            elif message[2] == 172:
                green_lost_during_tweaking += 1
            elif message[2] == 272:
                red_lost_during_tweaking += 1

            elif message[2] == 173:
                green_lost_after_turn += 1
            elif message[2] == 273:
                red_lost_after_turn += 1

            elif message[2] == 174:
                green_refound_after_turn += 1
            elif message[2] == 274:
                red_refound_after_turn += 1

            elif message[2] == 175:
                green_total_blocks_found += 1
            elif message[2] == 275:
                red_total_blocks_found += 1

    green_blocks_returned, red_blocks_returned = testUtility.count_blocks_in_correct_square()
    data.loc[sim_num] = [time_to_steady_state, green_blocks_identified, red_blocks_identified, green_blocks_returned, red_blocks_returned]
    lost_block_data.loc[sim_num] = [green_lost_after_turn, red_lost_after_turn,
                                    green_refound_after_turn, red_refound_after_turn,
                                    green_lost_during_tweaking, red_lost_during_tweaking,
                                    green_cant_find_colour, red_cant_find_colour,
                                    green_total_blocks_found, red_total_blocks_found]
    testUtility.reset_simulation()

supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
data.to_csv(os.path.join(DATA_SAVE_DIR, "test_csv_overall.csv"))
lost_block_data.to_csv(os.path.join(DATA_SAVE_DIR, "lost_block_data.csv"))
del supervisor

