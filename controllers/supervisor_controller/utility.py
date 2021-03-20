import struct
import random
import math

class TestUtility():

    def __init__(self, timestep, supervisor, robot_names = ["green_robot", "red_robot"], server_name = "laptop",
                 receiver_name = "receiver", board = [1.2, -1.2, 1.2, -1.2], start_square_offset_z = 0.4,
                 start_square_size = 0.4):
        self.supervisor = supervisor
        self.TIME_STEP = timestep

        self.board = board
        self.start_square_offset_z = start_square_offset_z
        self.start_square_size = start_square_size

        self.green_robot = self.get_node_by_name(robot_names[0])
        self.red_robot = self.get_node_by_name(robot_names[1])
        self.server = self.get_node_by_name(server_name)
        self.blocks = self.get_target_nodes()

        self.receiver = self.setup_receiver(receiver_name)

    def setup_receiver(self, name):
        receiver = self.supervisor.getDevice(name)
        receiver.enable(self.TIME_STEP)
        return receiver

    def receive_messages(self):
        messages = []
        while self.receiver.getQueueLength() > 0:
            messages.append(struct.unpack('<ddi', self.receiver.getData()))
            self.receiver.nextPacket()

        return messages

    def get_node_by_name(self, name):
        root = self.supervisor.getRoot()
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

    def get_target_nodes(self):
        target_names = ["target({})".format(i) for i in range(1, 10)]
        print(target_names)
        return list(filter(None, [self.get_node_by_name(name) for name in target_names]))

    def get_robot_fields(self, name):
        return self.red_robot.getField(name), self.green_robot.getField(name)

    def reset_simulation(self):
        self.supervisor.simulationReset()
        self.server.restartController()
        self.red_robot.restartController()
        self.green_robot.restartController()

    def assign_random_block_positions(self, wall_buffer):
        for block in self.blocks:
            print("working")
            trans_field = block.getField("translation")
            x = 0
            z = self.start_square_offset_z
            while ((abs(x) < self.start_square_size/2) and
                   (abs(z - self.start_square_offset_z) < self.start_square_size/2 or abs(z + self.start_square_offset_z) < self.start_square_size/2)):
                x = random.uniform(self.board[0] - wall_buffer, self.board[1] + wall_buffer)
                z = random.uniform(self.board[2] - wall_buffer, self.board[3] + wall_buffer)
            trans_field.setSFVec3f([x, 0.04, z])

    def count_blocks_in_correct_square(self):
        red_blocks_returned = 0
        green_blocks_returned = 0
        print("Counting blocks")

        for block in self.blocks:
            trans_field = block.getField("translation")
            colour_field = block.getField("colour")
            x, _, z = trans_field.getSFVec3f()

            print("block at x: {} z: {}".format(x, z))

            if ((abs(x) < self.start_square_size / 2) and
                   (abs(z - self.start_square_offset_z) < self.start_square_size / 2) and
                   colour_field.getSFColor()[0] == 1):
                # red block in red start_square
                print("Red block returned")
                red_blocks_returned += 1

            elif ((abs(x) < self.start_square_size / 2) and
                   (abs(z + self.start_square_offset_z) < self.start_square_size / 2) and
                   colour_field.getSFColor()[1] == 1):
                # green block in red start_square
                print("Green block returned")
                green_blocks_returned += 1

        return green_blocks_returned, red_blocks_returned
