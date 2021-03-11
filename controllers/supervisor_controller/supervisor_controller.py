"""supervisor_controller controller."""

# You may need to import some classes of the controller module. Ex:

from controller import Supervisor
import sys

TIME_STEP = 16

supervisor = Supervisor()

# do this once only
robot_node = supervisor.getFromDef("IDPROBOT")
gps_node = robot_node.getFromProtoDef("ROBOT_GPS")
root = supervisor.getRoot()
children = root.getField("children")
numChildren = children.getCount()

for i in range(numChildren):
    name_field = children.getMFNode(i).getField("name")
    if name_field is not None:
        print(name_field.getSFString())
        if name_field.getSFString() == "target(9)":
            print("found target node")
            target_node = children.getMFNode(i)
        
if target_node is None:
    print("could not get target")

trans_field = target_node.getField("translation")
trans_field.setSFVec3f([0, 0.025, 0])
target_node.resetPhysics()


if robot_node is None:
    print("Couldnt find robot!");
    sys.stderr.write("No DEF MY_ROBOT node found in the current world file\n")
    sys.exit(1)
    
if gps_node is None:
    print("couldnt find gps!");
trans_field = gps_node.getField("translation")

while supervisor.step(TIME_STEP) != -1:
    # this is done repeatedly
    values = trans_field.getSFVec3f()
    print("IDPROBOT is at position: %g %g %g" % (values[0], values[1], values[2]))