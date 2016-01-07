'''
Pose interpolator applies Quaternion SLERP to calculate intermediate poses given a starting point
and orientation and ending point and orientation.

Dependencies:
Q_Slerp.py - Script with function interpolate(...)
'''


from Q_Slerp import quaternion, interpolate
import numpy as np
import math
from numpy import array
from math import pi, cos, sin, acos, asin


def mat2euler(M, cy_thresh=None):
    ''' Discover Euler angle vector from 3x3 matrix

    Uses the conventions above.

    Parameters
    ----------
    M : array-like, shape (3,3)
    cy_thresh : None or scalar, optional
       threshold below which to give up on straightforward arctan for
       estimating x rotation.  If None (default), estimate from
       precision of input.

    Returns
    -------
    z : scalar
    y : scalar
    x : scalar
       Rotations in radians around z, y, x axes, respectively

    Notes
    -----
    If there was no numerical error, the routine could be derived using
    Sympy expression for z then y then x rotation matrix, which is::

      [                       cos(y)*cos(z),                       -cos(y)*sin(z),         sin(y)],
      [cos(x)*sin(z) + cos(z)*sin(x)*sin(y), cos(x)*cos(z) - sin(x)*sin(y)*sin(z), -cos(y)*sin(x)],
      [sin(x)*sin(z) - cos(x)*cos(z)*sin(y), cos(z)*sin(x) + cos(x)*sin(y)*sin(z),  cos(x)*cos(y)]

    with the obvious derivations for z, y, and x

       z = atan2(-r12, r11)
       y = asin(r13)
       x = atan2(-r23, r33)

    Problems arise when cos(y) is close to zero, because both of::

       z = atan2(cos(y)*sin(z), cos(y)*cos(z))
       x = atan2(cos(y)*sin(x), cos(x)*cos(y))

    will be close to atan2(0, 0), and highly unstable.

    The ``cy`` fix for numerical instability below is from: *Graphics
    Gems IV*, Paul Heckbert (editor), Academic Press, 1994, ISBN:
    0123361559.  Specifically it comes from EulerAngles.c by Ken
    Shoemake, and deals with the case where cos(y) is close to zero:

    See: http://www.graphicsgems.org/

    The code appears to be licensed (from the website) as "can be used
    without restrictions".
    '''
    M = np.asarray(M)
    if cy_thresh is None:
        try:
            cy_thresh = np.finfo(M.dtype).eps * 4
        except ValueError:
            cy_thresh = _FLOAT_EPS_4
    r11, r12, r13, r21, r22, r23, r31, r32, r33 = M.flat
    # cy: sqrt((cos(y)*cos(z))**2 + (cos(x)*cos(y))**2)
    cy = math.sqrt(r33*r33 + r23*r23)
    if cy > cy_thresh: # cos(y) not close to zero, standard form
        z = math.atan2(-r12,  r11) # atan2(cos(y)*sin(z), cos(y)*cos(z))
        y = math.atan2(r13,  cy) # atan2(sin(y), cy)
        x = math.atan2(-r23, r33) # atan2(cos(y)*sin(x), cos(x)*cos(y))
    else: # cos(y) (close to) zero, so x -> 0.0 (see above)
        # so r21 -> sin(z), r22 -> cos(z) and
        z = math.atan2(r21,  r22)
        y = math.atan2(r13,  cy) # atan2(sin(y), cy)
        x = 0.0
    return z, y, x


def quat2euler(q):
    ''' Return Euler angles corresponding to quaternion `q`

    Parameters
    ----------
    q : 4 element sequence
       w, x, y, z of quaternion

    Returns
    -------
    z : scalar
       Rotation angle in radians around z-axis (performed first)
    y : scalar
       Rotation angle in radians around y-axis
    x : scalar
       Rotation angle in radians around x-axis (performed last)

    Notes
    -----
    It's possible to reduce the amount of calculation a little, by
    combining parts of the ``quat2mat`` and ``mat2euler`` functions, but
    the reduction in computation is small, and the code repetition is
    large.
    '''
    # delayed import to avoid cyclic dependencies
    import nibabel.quaternions as nq
    return mat2euler(nq.quat2mat(q))


def euler2quat(z=0, y=0, x=0):
    z = z/2.0
    y = y/2.0
    x = x/2.0
    cz = cos(z)
    sz = sin(z)
    cy = cos(y)
    sy = sin(y)
    cx = cos(x)
    sx = sin(x)
    return np.array([
             cx*cy*cz - sx*sy*sz,
             cx*sy*sz + cy*cz*sx,
             cx*cz*sy - sx*cy*sz,
             cx*cy*sz + sx*cz*sy])


def linear_translation(A, B, T):
    '''
    Interpolates between 2 start points A an dB linearly

    Input:
    A - Start point
    B - Ending point
    T - intermediate points within a range from 0 - 1, 0 representing point A and 1 representing point B

    Output
    C - intermediate pose as an array
    '''
    V_AB = B -A
    C = A + T*(V_AB)
    return C



def linear_pose_interp(start_A, end_B, T):
    '''
    Pose interpolator that calculates intermediate poses of a vector connecting 2 points
    Interpolation is done linearly for both translation and rotation
    Translation is done assuming that point A is rotationally invariant.
    Rotation is done about the point A. Quaternion SLERP rotation is used to give the quickest rotation from A -> B
    ** "Roll" or twisting of the arm is not taken into account in this calculation. Separate interpolations have to be done for the roll angle

    Input:
    Starting points start_A [X,Y,Z,yaw,pitch,roll] list
    Ending points end_B in [X',Y',Z',yaw',pitch',roll'] list
        Yaw, Pitch, Roll calculated in radians within bounds [0, 2*pi]
    T = no of intermediate poses

    Output:
    list of positions and rotations stored into the variable track
        track is a dictionary with keys 'lin' and 'rot'

    track['lin'] - Linear interpolation of interval T of starting positions from A -> B
    track['rot'] - Slerp interpolation of quaternion of interval T, arranged as a list in [w x y z]
    # track['rot'] - Intermediate Yaw-Pitch-Roll poses of interval T, in sequence YPR

    '''

    track = {'lin': [], 'rot': []}

    ya = start_A[3]; pa = start_A[4]; ra = start_A[5]  # Yaw/pitch/Roll for A and B
    yb = end_B[3]; pb = end_B[4]; rb = end_B[5]

    A = array(start_A[:3]); B = array(end_B[:3])
    [wa, vxa, vya, vza] = euler2quat(ya, pa, ra)        # Quaternion representation of start and end points
    [wb, vxb, vyb, vzb] = euler2quat(yb, pb, rb)

    QA = quaternion(wa, vxa, vya, vza)
    QB = quaternion(wb, vxb, vyb, vzb)

    track['lin'] = linear_translation(A, B, T).tolist()
    q = interpolate(QA, QB, T)
    # track['rot'] = quat2euler([q.s] + (q.v).tolist()[0])    # Sequence is [Yaw, Pitch, Roll]
    track['rot'] = [q.s] + (q.v).tolist()[0]    # List of quaternion [w x y z]

    # print("Quaternion: ", track['rot'], "Y-P-R: ", quat2euler(track['rot']))
    return track


if __name__ == "__main__":
    A = [0, 0, 0, 0, 0, 0]
    B = [1, 1, 1, -pi/2, pi/2, -pi/2]

    track = linear_pose_interp(A, B, 1)
    # print(track)

