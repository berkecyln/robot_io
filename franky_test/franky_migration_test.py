from franky import Robot, Gripper, Affine, CartesianMotion, ReferenceType
# from robot_io.utils.franky_utils import to_affine
# from robot_io.ik.ik_franky import IKFrankY

import time
import numpy as np

ROBOT_IP = "172.16.0.2"


def test_robot_connection():
    try:
        robot = Robot(ROBOT_IP)
        state = robot.state
        print(f"Successfully connected to robot at {ROBOT_IP}.\nState: {state.q}")

    except Exception as e:
        print(f"Failed to connect to robot at {ROBOT_IP}: {e}")

def move_gripper():
    try:
        gripper = Gripper(ROBOT_IP)
        gripper.move(0.01, 0.1)
        print("Gripper is closing...")
        
        time.sleep(1)
        
        gripper.open(0.1)
        print("Gripper is opening...")

    except Exception as e:
        print(f"Failed to move gripper: {e}")

def move_arm():
    robot = Robot(ROBOT_IP)
    robot.relative_dynamics_factor = 0.05
     
    try:
        # Movement: [X,Y,Z]
        # Movements are based on gripper, in this setup gri
        # X - Forward/Backward
        # Y - Left/Right
        # Z - Up/Down
        target = Affine([0.0, 0.1, 0.0])
        motion = CartesianMotion(target=target, reference_type=ReferenceType.Relative)
        robot.move(motion)
        print("Arm moved successfully. To left 10 cm.")    

        time.sleep(1)

        target = Affine([0.0, -0.1, 0.0])
        motion = CartesianMotion(target=target, reference_type=ReferenceType.Relative)
        robot.move(motion)
        print("Arm moved successfully. To right 10 cm.")

        time.sleep(2)

    except Exception as e:
        print(f"Failed to move arm: {e}")


# ==================== Migration Tests ====================
# ===== Test for robot_io/utils/franky_utils.py =====
# def test_to_affine():
    
#     # Test 1: Quaternion input (identity)
#     pos1 = [0.1, 0.2, 0.3]
#     quat_identity = [0.0, 0.0, 0.0, 1.0]  # (x, y, z, w) - identity quaternion
    
#     try:
#         affine1 = to_affine(pos1, quat_identity)
        
#         assert isinstance(affine1, Affine)
#         print(f"Type check passed: {type(affine1).__name__}")
        
#         print("Test 1")
#         translation1 = affine1.translation
#         assert np.allclose(translation1.flatten(), pos1, atol=1e-6)
#         print(f"Translation: {pos1} -> {translation1.flatten()}")
        
#         quaternion1 = affine1.quaternion
#         assert np.allclose(quaternion1.flatten(), quat_identity, atol=1e-6)
#         print(f"Quaternion: {quat_identity} -> {quaternion1.flatten()}")
        
#         print("Test 2")
#         # Test 2: Quaternion input (90 degree rotation around Z-axis)
#         pos2 = [0.5, -0.2, 0.4]
#         quat_90z = [0.0, 0.0, 0.7071068, 0.7071068]  # 90 degree around Z (x, y, z, w)
        
#         affine2 = to_affine(pos2, quat_90z)
#         translation2 = affine2.translation
#         quaternion2 = affine2.quaternion
        
#         assert np.allclose(translation2.flatten(), pos2, atol=1e-6)
#         print(f"Translation: {pos2} -> {translation2.flatten()}")
#         assert np.allclose(quaternion2.flatten(), quat_90z, atol=1e-5)
#         print(f"Quaternion: {quat_90z} -> {quaternion2.flatten()}")
        
#         print("Test 3")
#         # Test 3: Euler angles input (90 degree rotation around Z-axis)
#         pos3 = [0.3, 0.0, 0.5]
#         euler_90z = [0.0, 0.0, np.pi/2]  # (roll, pitch, yaw)
        
#         affine3 = to_affine(pos3, euler_90z)
#         translation3 = affine3.translation
#         quaternion3 = affine3.quaternion
        
#         assert isinstance(affine3, Affine)
#         assert np.allclose(translation3.flatten(), pos3, atol=1e-6)
#         print(f"Translation: {pos3} -> {translation3.flatten()}")
        
#         # Expected: [0, 0, sin(pi/4), cos(pi/4)] = [0, 0, 0.7071, 0.7071]
#         expected_quat_from_euler = [0.0, 0.0, np.sin(np.pi/4), np.cos(np.pi/4)]
#         assert np.allclose(quaternion3.flatten(), expected_quat_from_euler, atol=1e-5)
#         print(f"Euler: {euler_90z} -> Quaternion {quaternion3.flatten()}")
        
#         print("\nAll to_affine() tests passed")
#         return True

#     except Exception as e:
#         print(f"\n to_affine() test failed: {e}")

# # ===== Test for robot_io/ik/ik_franky.py =====
# def test_ik_franky():
#     try:
#         # Panda joint limits
#         ll = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
#         ul = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        
#         # Create IK solver
#         ik_solver = IKFrankY(
#             nullspace_joint_id=6,
#             nullspace_joint_value=0.0,
#             ll=ll.tolist(),
#             ul=ul.tolist()
#         )
#         print("IK solver initialized")
        
#         # Test with a known good configuration
#         current_q = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
#         print(f"Current joint state: {current_q}")
        
#         # Target pose (reachable position)
#         target_pos = [0.5, 0.0, 0.4]
#         target_orn = [0.0, 0.0, 0.0, 1.0]  # identity quaternion
#         print(f"Target position: {target_pos}")
#         print(f"Target orientation: {target_orn}")
        
#         # Compute IK
#         q_solution = ik_solver.inverse_kinematics(target_pos, target_orn, current_q)
        
#         # Verify solution
#         assert isinstance(q_solution, np.ndarray), "Solution should be numpy array"
#         assert len(q_solution) == 7, "Solution should have 7 joints"
#         print(f"IK solution found: {q_solution}")
        
#         # Check joint limits
#         within_limits = np.all(q_solution >= ll) and np.all(q_solution <= ul)
#         print(f"Within joint limits: {within_limits}")
#         assert within_limits, "Solution exceeds joint limits"
        
#         # Test with unreachable position
#         unreachable_pos = [2.0, 2.0, 2.0]  # aperantly franke reach is aroun 1m so I gave 2m
#         q_fallback = ik_solver.inverse_kinematics(unreachable_pos, target_orn, current_q)
        
#         if np.allclose(q_fallback, current_q):
#             print(f"Returnd result: {q_fallback}")
#             print("Correctly handles unreachable positions")
        
#         print("\nAll IK tests passed")
#         return True

    except Exception as e:
        print(f"\n IK test failed: {e}")

if __name__ == "__main__":
    print("===== Test Started =====")
    #test_robot_connection()
    #move_arm()
    move_gripper()

    # franky_utils to_affine test
    #test_to_affine()

    # ik_franky test
    #test_ik_franky()

    print("===== Test completed =====")