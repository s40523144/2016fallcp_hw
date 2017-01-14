from robolink import *    # API to communicate with RoboDK
from robodk import *      # basic matrix operations

# Any interaction with RoboDK must be done through
# Robolink()
RDK = Robolink()

# get the robot item:
robot = RDK.ItemUserPick('Select a robot for welding', ITEM_TYPE_ROBOT)
if not robot.Valid():
    raise Exception("Operation cancelled by user")
    
# get the home joints target
home = robot.JointsHome()

# get the reference welding target:
target = RDK.Item('Target')

# get the reference frame and set it to the robot
reference = target.Parent()
robot.setFrame(reference)

# get the pose of the target (4x4 matrix):
poseref = target.Pose()
pose_approach = poseref*transl(0,0,-100)

# move the robot to home, then to the center:
robot.MoveJ(home)
robot.MoveJ(pose_approach)
robot.MoveL(target)

# make an hexagon around the center:
for i in range(7):
    ang = i*2*pi/6 #angle: 0, 60, 120, ...
    posei = poseref*rotz(ang)*transl(200,0,0)*rotz(-ang)
    robot.MoveL(posei)

# move back to the center, then home:
robot.MoveL(target)
robot.MoveL(pose_approach)
robot.MoveJ(home)


    
    
    



