"""
FrankyY Migration Test for panda_franky_interface.py

EXPECTED ROBOT BEHAVIOR:
- Gripper will open and close 
- Robot will make small movements (1-2cm up/down, testing motions)
- One motion will be intentionally stopped mid-execution (testing abort)

"""

import time
import numpy as np
from franky import (
    Robot, Gripper, Affine, RelativeDynamicsFactor,
    CartesianMotion, CartesianWaypointMotion, CartesianWaypoint,
    JointMotion, CartesianImpedanceMotion,
    ReferenceType
)


ROBOT_IP = "172.16.0.2"


def test_robot_initialization():
    "Test Step 2: Robot initialization without urdf_path, using state property."
    print("\n=== Test: Robot Initialization ===")
    try:
        # Test Robot creation without urdf_path
        robot = Robot(ROBOT_IP)
        robot.recover_from_errors()
        
        # Test state property access
        state = robot.state
        print(f"Robot state accessed: joint positions = {state.q}")
        
        # Test F_T_NE access from state
        F_T_NE = state.F_T_NE.matrix.T
        print(f"F_T_NE transformation obtained: shape = {F_T_NE.shape}")
        
        print(" Robot initialization test PASSED\n")
        return True
        
    except Exception as e:
        print(f"Robot initialization test FAILED: {e}\n")
        return False


def test_relative_dynamics_factor():
    "Test Step 3: Setting relative_dynamics_factor as tuple."
    print("\n=== Test: Relative Dynamics Factor ===")
    try:
        robot = Robot(ROBOT_IP)
        
        # Test RelativeDynamicsFactor object
        robot.relative_dynamics_factor = RelativeDynamicsFactor(0.3, 0.3, 0.2)
        print(f"Object assignment: {robot.relative_dynamics_factor}")
        
        # Test different RelativeDynamicsFactor values
        robot.relative_dynamics_factor = RelativeDynamicsFactor(0.5, 0.5, 0.3)
        print(f"Different values: {robot.relative_dynamics_factor}")
        
        # Test scalar (uniform factor)
        robot.relative_dynamics_factor = 0.1
        print(f"Scalar assignment: {robot.relative_dynamics_factor}")
        
        print("Relative dynamics factor test PASSED\n")
        return True
        
    except Exception as e:
        print(f"Relative dynamics factor test FAILED: {e}\n")
        return False


def test_gripper_operations():
    "Test Step 3: Gripper open/close with franky API."
    print("\n=== Test: Gripper Operations ===")
    try:
        gripper = Gripper(ROBOT_IP)
        
        # Test width property
        current_width = gripper.width
        print(f"Current gripper width: {current_width:.4f}m")
        
        # Test open with speed parameter
        print("Testing gripper.open(speed)...")
        gripper.open(speed=0.1)
        time.sleep(0.5)
        print(f"Gripper opened: width = {gripper.width:.4f}m")
        
        # Test grasp with force control
        print("Testing gripper.grasp(width=0.0, speed, force, epsilon)...")
        success = gripper.grasp(
            width=0.0,
            speed=0.1,
            force=20.0,
            epsilon_outer=0.04
        )
        time.sleep(0.5)
        print(f"Gripper grasped: success = {success}, width = {gripper.width:.4f}m")
        
        # Test async operations
        print("Testing async operations...")
        future = gripper.open_async(speed=0.1)
        success = future.wait(timeout=2.0)
        print(f"Async open: success = {success}")
        
        print("Gripper operations test PASSED\n")
        return True
        
    except Exception as e:
        print(f"Gripper operations test FAILED: {e}\n")
        return False


def test_state_reading():
    "Test Step 4: State reading with property access."
    print("\n=== Test: State Reading ===")
    try:
        robot = Robot(ROBOT_IP)
        
        # Test robot.state property
        state = robot.state
        print(f"State accessed as property (not method)")
        
        # Test state attributes
        joint_pos = np.array(state.q)
        print(f"Joint positions: {joint_pos}")
        
        # Test K_F_ext_hat_K (force/torque)
        force_torque = np.array(state.K_F_ext_hat_K)
        print(f"External forces: shape = {force_torque.shape}")
        
        # Test cartesian_contact
        contact = np.array(state.cartesian_contact)
        print(f"Cartesian contact: shape = {contact.shape}")
        
        # Test current_pose property
        current_pose = robot.current_pose
        print(f"Current pose: {current_pose.end_effector_pose.translation.flatten()}")
        
        # Test gripper width property
        gripper = Gripper(ROBOT_IP)
        width = gripper.width 
        print(f"Gripper width: {width:.4f}m")
        
        print("State reading test PASSED\n")
        return True
        
    except Exception as e:
        print(f"State reading test FAILED: {e}\n")
        return False


def test_is_in_control():
    "Test Step 5: Check robot.is_in_control property."
    print("\n=== Test: is_in_control Property ===")
    try:
        robot = Robot(ROBOT_IP)
        robot.relative_dynamics_factor = RelativeDynamicsFactor(0.05, 0.05, 0.05)
        
        # Check is_in_control when idle
        in_control = robot.is_in_control
        print(f"is_in_control (idle): {in_control}")
        
        # Start asynchronous motion - move up 1cm
        motion = CartesianMotion(Affine([0.0, 0.0, 0.01]), ReferenceType.Relative)
        robot.move(motion, asynchronous=True)
        
        time.sleep(0.1)
        in_control = robot.is_in_control
        print(f"is_in_control (moving): {in_control}")
        
        # Wait for completion
        robot.join_motion()
        in_control = robot.is_in_control
        print(f"is_in_control (after motion): {in_control}")
        
        # Move back to original position
        motion = CartesianMotion(Affine([0.0, 0.0, -0.01]), ReferenceType.Relative)
        robot.move(motion)
        print("Returned to starting position")
        
        print("is_in_control test PASSED\n")
        return True
        
    except Exception as e:
        print(f"is_in_control test FAILED: {e}\n")
        return False


def test_asynchronous_motion():
    "Test Step 5: Asynchronous motion with robot.move(..., asynchronous=True)."
    print("\n=== Test: Asynchronous Motion ===")
    try:
        
        
        robot = Robot(ROBOT_IP)
        robot.relative_dynamics_factor = RelativeDynamicsFactor(0.05, 0.05, 0.05)
        
        # Test Cartesian motion with asynchronous=True - move up 2cm
        print("Testing CartesianMotion async (up 2cm)...")
        motion = CartesianMotion(Affine([0.0, 0.0, 0.02]), ReferenceType.Relative)
        robot.move(motion, asynchronous=True)
        print(f"Async motion started, is_in_control = {robot.is_in_control}")
        
        # Wait for completion
        robot.join_motion(timeout=5.0)
        print(f"Motion completed, is_in_control = {robot.is_in_control}")
        
        # Test stopping motion - start moving up 2cm then stop
        print("Testing robot.stop()...")
        motion = CartesianMotion(Affine([0.0, 0.0, 0.02]), ReferenceType.Relative)
        robot.move(motion, asynchronous=True)
        time.sleep(0.2)
        robot.stop()
        try:
            robot.join_motion()
        except Exception as e:
            # Stopping motion mid-execution causes "Move command preempted!" -this is expected so I desgined test to catch it
            if "preempted" in str(e).lower():
                print(f"Motion stopped (preempted as expected), is_in_control = {robot.is_in_control}")
            else:
                raise
        robot.recover_from_errors()  # Clear the preemption error
        
        # Return to original position - move down 4cm
        print("Returning to starting position...")
        motion = CartesianMotion(Affine([0.0, 0.0, -0.04]), ReferenceType.Relative)
        robot.move(motion)
        print("Returned to start position")
        
        print("Asynchronous motion test PASSED\n")
        return True
        
    except Exception as e:
        print(f"Asynchronous motion test FAILED: {e}\n")
        return False


def test_motion_classes():
    """Test Step 5: New franky motion classes."""
    print("\n=== Test: Motion Classes ===")
    robot = None
    try:
        robot = Robot(ROBOT_IP)
        robot.relative_dynamics_factor = RelativeDynamicsFactor(0.05, 0.05, 0.05)
        
        # Save starting position
        print("Saving starting position...")
        start_q = robot.state.q
        
        # Test CartesianWaypointMotion - move up 1cm
        print("Testing CartesianWaypointMotion (move up 1cm)...")
        print("  Creating waypoint motion...")
        waypoint_motion = CartesianWaypointMotion([
            CartesianWaypoint(Affine([0.0, 0.0, 0.01]), ReferenceType.Relative)
        ])
        print("  Executing motion...")
        robot.move(waypoint_motion)
        print("CartesianWaypointMotion executed")
        
        time.sleep(0.3)
        
        # Move back down 1cm to return to start
        print("Returning to starting position...")
        print("  Creating return motion...")
        waypoint_motion = CartesianWaypointMotion([
            CartesianWaypoint(Affine([0.0, 0.0, -0.01]), ReferenceType.Relative)
        ])
        print("  Executing return motion...")
        robot.move(waypoint_motion)
        print("Returned to start position")
        
        time.sleep(0.3)
        
        # Test JointMotion - go to exact starting position
        print("Testing JointMotion (confirm start position)...")
        print("  Creating joint motion...")
        joint_motion = JointMotion(start_q)
        print("  Executing joint motion...")
        robot.move(joint_motion)
        print("JointMotion executed - robot at starting position")
        
        time.sleep(0.3)
        
        # Test CartesianImpedanceMotion - move up 1cm with impedance control
        print("Testing CartesianImpedanceMotion (move up 1cm, compliant)...")
        from franky import Duration
        current_pose = robot.current_pose.end_effector_pose
        target_pose = current_pose * Affine([0.0, 0.0, 0.01])
        impedance_motion = CartesianImpedanceMotion(
            target_pose,
            Duration(2000),  # Duration in milliseconds (2000ms = 2s)
            translational_stiffness=600,
            rotational_stiffness=200
        )
        print("  Executing impedance motion...")
        robot.move(impedance_motion)
        print("CartesianImpedanceMotion executed")
        
        time.sleep(0.3)
        
        # Move back down to start
        print("Returning to starting position...")
        joint_motion = JointMotion(start_q)
        robot.move(joint_motion)
        print("Returned to start position")
        
        print("Motion classes test PASSED\n")
        return True
        
    except Exception as e:
        print(f"Motion classes test FAILED: {e}\n")
        import traceback
        traceback.print_exc()
        if robot:
            try:
                robot.stop()
                robot.recover_from_errors()
            except:
                pass
        return False


def test_affine_operations():
    """Test Step 1: Affine creation with numpy arrays."""
    print("\n=== Test: Affine Operations ===")
    try:
        # Test Affine with translation and quaternion arrays
        translation = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
        quaternion = np.array([0.0, 1.0, 0.0, 0.0]).reshape(4, 1)  # 180° around Y
        affine = Affine(translation=translation, quaternion=quaternion)
        print(f"Affine created with numpy arrays")
        
        # Test Affine multiplication
        offset = Affine([0.1, 0.0, 0.0])
        result = affine * offset
        print(f"Affine multiplication works")
        
        # Test accessing translation and quaternion
        trans = result.translation
        quat = result.quaternion
        print(f"Translation: {trans.flatten()}")
        print(f"Quaternion: {quat.flatten()}")
        
        print("Affine operations test PASSED\n")
        return True
        
    except Exception as e:
        print(f"Affine operations test FAILED: {e}\n")
        return False


def run_all_tests():
    """Run all migration tests."""
    print("\n" + "="*60)
    print("FRANKY MIGRATION TESTS")
    print("="*60)
    
    tests = [
        ("Affine Operations", test_affine_operations),
        ("Robot Initialization", test_robot_initialization),
        ("Relative Dynamics Factor", test_relative_dynamics_factor),
        ("Gripper Operations", test_gripper_operations),
        ("State Reading", test_state_reading),
        ("is_in_control Property", test_is_in_control),
        ("Asynchronous Motion", test_asynchronous_motion),
        ("Motion Classes", test_motion_classes),
    ]
    
    results = []
    for name, test_func in tests:
        try:
            result = test_func()
            results.append((name, result))
            # Add small delay between tests for robot stability
            time.sleep(0.5)
        except Exception as e:
            print(f"{name} crashed: {e}\n")
            results.append((name, False))
            # Try to recover robot after crash
            try:
                robot = Robot(ROBOT_IP)
                robot.stop()
                robot.recover_from_errors()
                time.sleep(1.0)
            except:
                pass
    
    # Summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for name, result in results:
        status = "PASSED" if result else "FAILED"
        print(f"{name:.<50} {status}")
    
    print("="*60)
    print(f"Total: {passed}/{total} tests passed")
    print("="*60)
    
    return passed == total


if __name__ == "__main__":
    success = run_all_tests()
    exit(0 if success else 1)
