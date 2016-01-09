# Function to test allowable linear and rotation limits of the robot's arm

import math
import time
from robot import *
from math import pi
from PyKDL import *

def cartesian_move(robotname):

	r = robot(robotname)

	pos = [-0.0152460794581, 0.00298808875685, -0.0507015737237]
	rot = ['Roll_a', 'Pitch_a', 'Yaw_a']

    r.move_cartesian(Vector(pos[0], pos[1], pos[2]), interpolate = True)	# Catersian coordiantes X, Y, Z
    time.sleep(1)

    r.move_cartesian_rotation(Rotation.RPY(rot[0], rot[1], rot[2])			# Roll, Pitch, Yaw
    time.sleep(1)

    print 'Current position:'
    print r.get_current_cartesian_position()


if __name__ == '__main__':
    """Here will check if we are given the name of the robot arm or not, if we are we run cartesian_move()"""
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        cartesian_move(sys.argv[1])

