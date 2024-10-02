import numpy as np

def RpToTrans(R, p):
    """
        Converts a rotation matrix and a position vector into homogeneous
        transformation matrix
        :param R: A 3x3 rotation matrix
        :param p: A 3-vector
        :return: A homogeneous transformation matrix corresponding to the inputs
        Example Input:
            R = np.array([[1, 0,  0],
                          [0, 0, -1],
                          [0, 1,  0]])
            p = np.array([1, 2, 5])
        Output:
            np.array([[1, 0,  0, 1],
                      [0, 0, -1, 2],
                      [0, 1,  0, 5],
                      [0, 0,  0, 1]])
        """
    return np.r_[np.c_[R, p], [[0, 0, 0, 1]]]

def TransToRp(T):
    """
    Converts a homogeneous transformation matrix into a rotation matrix
    and position vector
    :param T: A homogeneous transformation matrix
    :return R: The corresponding rotation matrix,
    :return p: The corresponding position vector.
    Example Input:
        T = np.array([[1, 0,  0, 0],
                      [0, 0, -1, 0],
                      [0, 1,  0, 3],
                      [0, 0,  0, 1]])
    Output:
        (np.array([[1, 0,  0],
                   [0, 0, -1],
                   [0, 1,  0]]),
         np.array([0, 0, 3]))
    """
    T = np.array(T)
    return T[0:3, 0:3], T[0:3, 3]

def TransInv(T):
    """
    Inverts a homogeneous transformation matrix
    :param T: A homogeneous transformation matrix
    :return: The inverse of T
    Uses the structure of transformation matrices to avoid taking a matrix
    inverse, for efficiency.
    Example input:
        T = np.array([[1, 0,  0, 0],
                      [0, 0, -1, 0],
                      [0, 1,  0, 3],
                      [0, 0,  0, 1]])
    Output:
        np.array([[1,  0, 0,  0],
                  [0,  0, 1, -3],
                  [0, -1, 0,  0],
                  [0,  0, 0,  1]])
    """
    R, p = TransToRp(T)
    Rt = np.array(R).T
    return np.r_[np.c_[Rt, -np.dot(Rt, p)], [[0, 0, 0, 1]]]


def Adjoint(T):
    """
    Computes the adjoint representation of a homogeneous transformation
    matrix
    :param T: A homogeneous transformation matrix
    :return: The 6x6 adjoint representation [AdT] of T
    Example Input:
        T = np.array([[1, 0,  0, 0],
                      [0, 0, -1, 0],
                      [0, 1,  0, 3],
                      [0, 0,  0, 1]])
    Output:
        np.array([[1, 0,  0, 0, 0,  0],
                  [0, 0, -1, 0, 0,  0],
                  [0, 1,  0, 0, 0,  0],
                  [0, 0,  3, 1, 0,  0],
                  [3, 0,  0, 0, 0, -1],
                  [0, 0,  0, 0, 1,  0]])
    """
    R, p = TransToRp(T)
    return np.r_[np.c_[R, np.zeros((3, 3))], np.c_[np.dot(VecToso3(p), R), R]]

def VecToso3(omg):
    """
    Converts a 3-vector to an so(3) representation
    :param omg: A 3-vector
    :return: The skew symmetric representation of omg
    Example Input:
        omg = np.array([1, 2, 3])
    Output:
        np.array([[ 0, -3,  2],
                  [ 3,  0, -1],
                  [-2,  1,  0]])
    """
    return np.array([[0, -omg[2], omg[1]], [omg[2], 0, -omg[0]], [-omg[1], omg[0], 0]])

def RPY(roll, pitch, yaw):
    """
    Creates a Roll, Pitch, Yaw Transformation Matrix
    :param roll: roll component of matrix
    :param pitch: pitch component of matrix
    :param yaw: yaw component of matrix
    :return: The transformation matrix
    Example Input:
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
    Output:
        np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    """
    Roll = np.array(
        [
            [1, 0, 0, 0],
            [0, np.cos(roll), -np.sin(roll), 0],
            [0, np.sin(roll), np.cos(roll), 0],
            [0, 0, 0, 1],
        ]
    )
    Pitch = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch), 0],
            [0, 1, 0, 0],
            [-np.sin(pitch), 0, np.cos(pitch), 0],
            [0, 0, 0, 1],
        ]
    )
    Yaw = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0, 0],
            [np.sin(yaw), np.cos(yaw), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )
    return np.matmul(np.matmul(Roll, Pitch), Yaw)


def RotateTranslate(rotation, position):
    """
    Creates a Transformation Matrix from a Rotation, THEN, a Translation
    :param rotation: pure rotation matrix
    :param translation: pure translation matrix
    :return: The transformation matrix
    """
    trans = np.eye(4)
    trans[0, 3] = position[0]
    trans[1, 3] = position[1]
    trans[2, 3] = position[2]

    return np.dot(rotation, trans)

def TransformVector(xyz_coord, rotation, translation):
    """
    Transforms a vector by a specified Rotation THEN Translation Matrix
    :param xyz_coord: the vector to transform
    :param rotation: pure rotation matrix
    :param translation: pure translation matrix
    :return: The transformed vector
    """
    xyz_vec = np.append(xyz_coord, 1.0)

    Transformed = np.dot(RotateTranslate(rotation, translation), xyz_vec)
    return Transformed[:3]

def quaternion_to_euler(q):
    x, y, z, w = q

    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(2 * (w * y - z * x))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    return roll, pitch, yaw


def quaternion_from_euler(a):
    ai, aj, ak = a
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q


def quat_from_angle_axis(aa):
    angle = aa[3]
    axis = np.array(aa[:3])
    norm_axis = axis / np.linalg.norm(axis)
    half_angle = 0.5 * angle
    sin_half_angle = np.sin(half_angle)

    x = sin_half_angle * norm_axis[0]
    y = sin_half_angle * norm_axis[1]
    z = sin_half_angle * norm_axis[2]
    w = np.cos(half_angle)

    return [x, y, z, w]


def diff_quat(q2, q1):
    # Calculate the product of q2 and the inverse of q1
    q1_inv = [-q1[0], -q1[1], -q1[2], q1[3]]

    x1, y1, z1, w1 = q1_inv
    x2, y2, z2, w2 = q2

    x = x2 * w1 + y2 * z1 - z2 * y1 + w2 * x1
    y = -x2 * z1 + y2 * w1 + z2 * x1 + w2 * y1
    z = x2 * y1 - y2 * x1 + z2 * w1 + w2 * z1
    w = w2 * w1 - x2 * x1 - y2 * y1 - z2 * z1

    return [x, y, z, w]