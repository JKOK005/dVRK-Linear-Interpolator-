import math
import time
from robot import *
from poseInterpolator_5 import linear_pose_interp, quat2euler
from math import pi
from PyKDL import *

def cartesian_move(robotName):
    """ Initialize the robot's arm using 'PSM1', 'PSM2'"""

    r = robot(robotName)

    r.move_cartesian(Vector(-0.0152460794581, 0.00298808875685, -0.0507015737237), interpolate = True)
    r.move_cartesian_rotation(Rotation.RPY('Roll_a', 'Pitch_a', 'Yaw_a')
    time.sleep(5)

    print 'Starting position:'
    print r.get_current_cartesian_position()
    
    start_pose = [-0.0152460794581, 0.00298808875685, -0.0507015737237, 'Roll_a', 'Pitch_a', 'Yaw_a']       # Starting position in [vx vy vz roll pitch yaw]
    end_pose = [0.00789069117847, 0.011891057143, -0.0901080667402, 'Roll_b', 'Pitch_b', 'Yaw_b']           # Ending position in [vx vy vz roll pitch yaw]
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
    
    print "Current position: " 
    print r.get_current_cartesian_position()


if __name__ == '__main__':
    """Here will check if we are given the name of the robot arm or not, if we are we run cartesian_move()"""
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        cartesian_move(sys.argv[1])
