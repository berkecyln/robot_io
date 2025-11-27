from robot_io.utils.utils import euler_to_quat, scipy_quat_to_np_quat
from franky import Affine
import numpy as np


def to_affine(target_pos, target_orn):
    """
    Converts position and orientation to Franky Affine format.
    Args:
        target_pos: [x, y, z] list or array
        target_orn: [x, y, z, w] quaternion or [r, p, y] euler angles
    Returns:
        franky.Affine object
    """
    if len(target_orn) == 3:
        target_orn = euler_to_quat(target_orn)
    target_orn = scipy_quat_to_np_quat(target_orn)

    transilation = np.array(target_pos).reshape(3,1)
    # franky requires [x, y, z, w] format
    quaternion  = np.array([target_orn.x, target_orn.y, target_orn.z, target_orn.w]).reshape(4,1)
    return Affine(transilation, quaternion)