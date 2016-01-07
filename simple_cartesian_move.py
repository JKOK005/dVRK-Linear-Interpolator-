"""In this example we will take a look at the how to move in cartesian space. Please note we can move a different distance than the one specified. We will show how the following methods world:
 * delta_move_cartesian_translation()
 * delta_move_cartesian_rotation
 * move_cartesian_translation
 * move_cartesian_rotation

Lets take a look:"""

from robot import *
from PyKDL import *
import math
from math import pi
import time

def cartesian_move(robotName):
    """Here is where we initialize the robot, in this demo we called the robot r, and we move the robot in castresian space accordingly.

    :param robotName: the name of the robot used """

    r = robot(robotName)
    print 'Starting position:'
    print r.get_current_cartesian_position()

    list_1 = [-0.02,  0.005,  -0.07]
    r.move_cartesian(list_1, interpolate = True)

    for ii in range(5):
        list_1 = [-0.02 + ii/1000.0,  0.005,  -0.07]
        # rot_1 = Rotation.RPY(pi,0,0)
        # frame = Frame(rot_1 , list_1)
        # print(frame)

        r.move_cartesian(list_1, interpolate = False)
        time.sleep(1)
        print r.get_current_cartesian_position()

if __name__ == '__main__':
    """Here will check if we are given the name of the robot arm or not, if we are we run cartesian_move()"""
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        cartesian_move(sys.argv[1])


    # list_1 = [0.01, 0.0, 0.0]
    # list_4 = [-0.01, 0.0, 0.0]

    # for i in range(5):

    #     r.delta_move_cartesian(list_1, interpolate = True)
    #     print r.get_current_cartesian_position()

    #     r.delta_move_cartesian(list_1, interpolate = True)
    #     print r.get_current_cartesian_position()

    #     r.delta_move_cartesian(list_1, interpolate = True)
    #     print r.get_current_cartesian_position()

    #     r.delta_move_cartesian(list_4, interpolate = True)
    #     print r.get_current_cartesian_position()

    #     r.delta_move_cartesian(list_4, interpolate = True)
    #     print r.get_current_cartesian_position()

    #     r.delta_move_cartesian(list_4, interpolate = True)
    #     print r.get_current_cartesian_position()

    #     time.sleep(1)