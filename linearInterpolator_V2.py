"""In this example we will take a look at the how to move in cartesian space. Please note we can move a different distance than the one specified. We will show how the following methods world:
 * delta_move_cartesian_translation()
 * delta_move_cartesian_rotation
 * move_cartesian_translation
 * move_cartesian_rotation

Lets take a look:"""

import math
import time
from robot import *
from poseInterpolator_4 import linear_pose_interp, quat2euler
from math import pi
from PyKDL import *

def cartesian_move(robotName):
    """Here is where we initialize the robot, in this demo we called the robot r, and we move the robot in castresian space accordingly.

    :param robotName: the name of the robot used """

    r = robot(robotName)

    [rot_limit_1_Z, rot_limit_1_Y, rot_limit_1_X] = quat2euler([-0.0473987537752, 0.516916337361, 0.854141731692, -0.0315081020154])
    [rot_limit_2_Z, rot_limit_2_Y, rot_limit_2_X] = quat2euler([0.0296427350675, 0.952103599283, -0.283398272039, -0.110929995474])

    # limit_1 = Frame(Rotation.Quaternion(-0.0473987537752, 0.516916337361, 0.854141731692, -0.0315081020154), Vector(-0.0152460794581, 0.00298808875685, -0.0507015737237))
    # limit_2 = Frame(Rotation.Quaternion(0.0296427350675, 0.952103599283, -0.283398272039, -0.110929995474), Vector(0.00789069117847, 0.011891057143, -0.0901080667402))

    r.move_cartesian(Vector(-0.0152460794581, 0.00298808875685, -0.0507015737237), interpolate = True)
    r.move_cartesian_rotation(Rotation.RPY(rot_limit_1_X,rot_limit_1_Y,rot_limit_1_Z))
    time.sleep(5)

    print 'Starting position:'
    print r.get_current_cartesian_position()
    
    start_pose = [-0.0152460794581, 0.00298808875685, -0.0507015737237, rot_limit_1_Z, rot_limit_1_Y, rot_limit_1_X]             # Starting position in [vx vy vz yaw pitch roll]
    end_pose = [0.00789069117847, 0.011891057143, -0.0901080667402, rot_limit_2_Z, rot_limit_2_Y, rot_limit_2_X]    # Ending position in [vx vy vz yaw pitch roll]
    T = 10000

    for ii in range(T):
        track = linear_pose_interp(start_pose, end_pose, (ii +1.0)/T)
        lin = track['lin']
        quat = track['rot']     # In the form [w x y z]

        vect = Vector(lin[0], lin[1], lin[2])
        rot = Rotation.Quaternion(quat[1], quat[2], quat[3], quat[0])  # function accepts the form (x, y, z, w)
        frame = Frame(rot, vect)

        r.move_cartesian_frame(frame, interpolate = False)
        # r.move_cartesian(vect , interpolate = True)
        time.sleep(0.01)
        # print 'Current position:'
        # print r.get_current_cartesian_position()
    
    # print "Limit 2: ", limit_2
    print "Current position: ", r.get_desired_cartesian_position()


if __name__ == '__main__':
    """Here will check if we are given the name of the robot arm or not, if we are we run cartesian_move()"""
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        cartesian_move(sys.argv[1])
