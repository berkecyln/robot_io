import numpy as np

from franky import Affine # No used here but required for  to_affine
from ikfast_franka_panda import get_ik
from robot_io.utils.franky_utils import to_affine
from robot_io.utils.utils import pos_orn_to_matrix, matrix_to_pos_orn


class IKFrankY:
    def __init__(self, nullspace_joint_id, nullspace_joint_value, ll, ul,):
        self.null_space_joint_id = nullspace_joint_id
        self.null_space_joint_value = nullspace_joint_value
        self.ll = np.array(ll)
        self.ul = np.array(ul)
        # self.EE_T_NE = np.linalg.inv(np.array([[1, 0, 0, 0],
        #                                         [0, -1, 0, 0],
        #                                         [0, 0, -1, 0],
        #                                         [0, 0, 0, 1]]))
        self.F_T_EEold = np.array([[ 0.7071,  0.7071,  0.    ,  0.    ],
                                   [ 0.7071, -0.7071,  0.    ,  0.    ],
                                   [ 0.    ,  0.    , -1.    ,  0.    ],
                                   [ 0.    ,  0.    ,  0.    ,  1.    ]])
        self.NE_T_F = np.linalg.inv(np.array([[ 0.7071,  0.7071,  0.    ,  0.    ],
                                                [ -0.7071, 0.7071,  0.    ,  0.    ],
                                                [ 0.    ,  0.    ,  1.    ,  0.1034],
                                                [ 0.    ,  0.    ,  0.    ,  1.    ]]))

    def inverse_kinematics(self, target_pos, target_orn, current_joint_state):
        R_T_EE = pos_orn_to_matrix(target_pos, target_orn)
        _target_pos, _target_orn = matrix_to_pos_orn(R_T_EE @ self.NE_T_F @ self.F_T_EEold)
        #q_new = Kinematics.inverse(to_affine(_target_pos, _target_orn).vector(), current_joint_state)#, self.null_space)
        target_affine = to_affine(_target_pos, _target_orn) # Franky Affine
        target_matrix = target_affine.matrix
        translation = target_matrix[:3, 3].tolist()
        rotation = target_matrix[:3, :3].tolist()
        ik_solutions = get_ik(translation, rotation, [self.null_space_joint_value])

        if not ik_solutions:
            print("No IK solution found")
            return current_joint_state
        
        valid_solutions = []
        for sol in ik_solutions:
            q_new = np.array(sol)
            q_new[np.where(q_new > 0)] %= (2 * np.pi)
            q_new[np.where(q_new < 0)] %= (-2 * np.pi)
            
            if np.all(q_new >= self.ll) and np.all(q_new <= self.ul):
                valid_solutions.append(q_new)
        
        if not valid_solutions:
            print("Solution exceeds joint limits")
            return current_joint_state
        
        distances = [np.linalg.norm(sol - current_joint_state) for sol in valid_solutions]
        closest_solution = valid_solutions[np.argmin(distances)]
        
        return closest_solution
