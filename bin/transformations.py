
import numpy as np


def __atan4(a, b, c, w):
    y = 2 * (w * a + b * c)
    x = 1 - 2 * (a ** 2 + b ** 2)
    return np.arctan2(y, x)

def euler_from_quaternion(q):
    x, y, z, w = q.x, q.y, q.z, q.w

    roll = __atan4(x, y, z, w)
    pitch = np.arcsin(2 * (w * y - z * x))
    yaw = __atan4(z, y, x, w)

    return roll, pitch, yaw

