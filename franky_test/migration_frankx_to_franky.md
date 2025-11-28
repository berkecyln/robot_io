# FrankX -> FrankY Migration
There are 3 main files that use frankx import these are `frankx_utils.py `, `ik_frankx.py` and `panda_frankx_interface.py`
### 1. frankx_utils.py -> franky_utils.py
**Changes:**
- Updated Affine constructor from 7 scalar arguments to 2 numpy arrays (translation, quaternion) since franky expects like that.
- Changed quaternion order from `[w,x,y,z]` to `[x,y,z,w]` again franky expects diffrent ordering.
- Renamed the file as `franky_utils.py`

**Test Result:**

Test script is: `test_to_affine()` in `franky_migration_test.py` 
```
Type check passed: Affine
Test 1
Translation: [0.1, 0.2, 0.3] -> [0.1 0.2 0.3]
Quaternion: [0.0, 0.0, 0.0, 1.0] -> [0. 0. 0. 1.]
Test 2
Translation: [0.5, -0.2, 0.4] -> [ 0.5 -0.2  0.4]
Quaternion: [0.0, 0.0, 0.7071068, 0.7071068] -> [0.         0.         0.7071068  0.70710676]
Test 3
Translation: [0.3, 0.0, 0.5] -> [0.3 0.  0.5]
Euler: [0.0, 0.0, 1.5707963267948966] -> Quaternion [0.         0.         0.70710678 0.70710678]

All to_affine() tests passed
```

### 2. ik_frankx.py -> ik_franky.py
**Changes:**

- Replaced `frankx.Kinematics.inverse()` with `ikfast_franka_panda.get_ik()` since franky does not have any knematics method
- Removed `NullSpaceHandling` class since nullspace now passed as parameter to `get_ik()`
- Added filtering for multiple IK solutions since IKFast returns all solutions, not just one. So added abasic logic which picks closest to current joint state (this is my idea it can be changed I am not sure how will it work in real settings)
- Renamed file as `ik_franky.py`
- Modified `robot_io/conf/robot/ik/ik_frankx.yaml` to `robot_io/conf/robot/ik/ik_franky.yaml` with `updated _target_ path`

**Note:** However I belice this class not used by defult since defult robot config uses IKfast.py directly. In any case I tried to update accuratly as possible. 

**Test Result:**
Test script is: `test_ik_franky()` in `franky_migration_test.py`
```
IK solver initialized
Current joint state: [ 0.    -0.785  0.    -2.356  0.     1.571  0.785]
Target position: [0.5, 0.0, 0.4]
Target orientation: [0.0, 0.0, 0.0, 1.0]
IK solution found: [ 1.69287024 -1.24187679 -1.28751981 -2.17977636 -1.14122507  1.53860136
  0.        ]
Within joint limits: True
No IK solution found
Returnd result: [ 0.    -0.785  0.    -2.356  0.     1.571  0.785]
Correctly handles unreachable positions

All IK tests passed
```

### 3. panda_frankx_interface.py

**Note:** Only remaining replacement is file and function renaming which i will do in lab computer and do additional test to be safe

**Note:** franky uses property access rather than method calls so most of old methods replaced via equivalent access

- All frankx related imports are removed and related franky ones are added:
  ```
  from franky import (
      Robot, Gripper, Affine, RelativeDynamicsFactor, Duration,
      JointMotion, JointWaypointMotion,
      CartesianMotion, CartesianWaypointMotion, CartesianWaypoint,
      CartesianImpedanceMotion, ExponentialImpedanceMotion,
      NetworkException, ReferenceType as FrankyReferenceType
  )
  ```

- Frame translation part changed to translation and quaternion parts since Franky requires numpy arrays with specific shapes (3x1 and 4x1), not scalars.
- In robot initialization several things are changed:
  - `Robot()` not use urdf_path anymore since franky has build in panda franka model inside.
  - `self.robot.set_default_behavior()` is commented out since in franky this methid does not exist.
  - `Gripper()` not take any parameter to in it so I decided to take params as `self.gripper_params = gripper_params` and use where is needed.
  - > Note: There are several changes in Gripper's open and grab please see gripper bulletpoint.
- In state access `.read_once()` changed to `.state` and state needs matrix property not flattened ones so `.matrix` added.
- `frankx_params.velocity_rel`, `frankx_params.acceleration_rel`, `frankx_params.jerk_rel` is now not separate variables but they constructed via `RelativeDynamicsFactor` since Franky requires explicit object creation for type safety.
- All `WaypointMotion([Waypoint(target_pose)])` are changed to `CartesianWaypointMotion([CartesianWaypoint(target_pose)])`
- All `move_async(motion)`replaced with `move(motion, asynchronous=True)` since franky uses unified API for move.
- `current_motion.stop()` changed to `.robot.stop()` this throws  "Move command preempted!" exception which is expected however to recover from error state `robot.recover_from_errors()`  is need.
- `self.motion_thread.join()` in async case is removed since franky does handle this internally.
- Changed `.current_pose()` to `end_effector_pose` which directly return Affine object.
- Gripper has several changes:
  - Franky does not accept blocking or speed parameters to `.open()` method.
  - To replicate old behaviour control blocking behaviour via parameter, if blocking is True this means its sync move so ve use `.open(speed)` however if its False this means its async so we use `async_open(speed)`
  - However franky not accept timeout to gripper so currently gripper open has no option for timeout.
  - > NEED REVIEW: if needed we need to add timeout logic separately.
  - In close method franky uses `closing_threshold` from config as `epsilon_inner/epsilon_outer` for grasp detection since it uses force-controlled grasping.
    - As far as I searched  width=0.0 with force limit safely closes until object detected.
- Impedance motion update between 264-282 has some problems requires fi
  further review.
  - frankx's `ImpedanceMotion` can be changed with franky's `ExponentialImpedanceMotion` or `CartesianImpedanceMotion`.
  - However when we use `ExponentialImpedanceMotion` it creates C++ crash which I controlled and it is an know issue and still open as [bug](https://github.com/pantor/frankx/issues/21) so I used `CartesianImpedanceMotion` but it requires `Duration` object in **milliseconds**, I have added duration parameter to configs impedance params.
  -  > NEED REVIEW: if dont want to use Duration we need to find a different way to replace frankx's `ImpedanceMotion`.
- Lastly `motion_thread.is_alive()` is replaced via `robot.is_in_control`

**Test Results:**

Tests are in seperate file under franky_test: `test_panda_interface_migration.py`
```
============================================================
FRANKY MIGRATION TESTS
============================================================

=== Test: Affine Operations ===
Affine created with numpy arrays
Affine multiplication works
Translation: [-0.1  0.   0. ]
Quaternion: [0. 1. 0. 0.]
Affine operations test PASSED


=== Test: Robot Initialization ===
Robot state accessed: joint positions = [-0.15199173 -0.46833004  0.13901612 -2.50383386 -0.05916813  2.10873654
  0.89431431]
F_T_NE transformation obtained: shape = (4, 4)
 Robot initialization test PASSED


=== Test: Relative Dynamics Factor ===
Object assignment: <franky._franky.RelativeDynamicsFactor object at 0x7376b26394b0>
Different values: <franky._franky.RelativeDynamicsFactor object at 0x7376b26394b0>
Scalar assignment: <franky._franky.RelativeDynamicsFactor object at 0x7376b26394b0>
Relative dynamics factor test PASSED


=== Test: Gripper Operations ===
Current gripper width: 0.0783m
Testing gripper.open(speed)...
Gripper opened: width = 0.0784m
Testing gripper.grasp(width=0.0, speed, force, epsilon)...
Gripper grasped: success = True, width = 0.0006m
Testing async operations...
Async open: success = True
Gripper operations test PASSED


=== Test: State Reading ===
State accessed as property (not method)
Joint positions: [-0.15198787 -0.46833004  0.13902334 -2.5038291  -0.05916081  2.10871929
  0.89429253]
External forces: shape = (6,)
Cartesian contact: shape = (6,)
Current pose: [ 0.41419988 -0.01850611  0.34954133]
Gripper width: 0.0783m
State reading test PASSED


=== Test: is_in_control Property ===
is_in_control (idle): False
is_in_control (moving): True
is_in_control (after motion): False
Returned to starting position
is_in_control test PASSED


=== Test: Asynchronous Motion ===
Testing CartesianMotion async (up 2cm)...
Async motion started, is_in_control = True
Motion completed, is_in_control = False
Testing robot.stop()...
Motion stopped (preempted as expected), is_in_control = False
Returning to starting position...
Returned to start position
Asynchronous motion test PASSED


=== Test: Motion Classes ===
Saving starting position...
Testing CartesianWaypointMotion (move up 1cm)...
  Creating waypoint motion...
  Executing motion...
CartesianWaypointMotion executed
Returning to starting position...
  Creating return motion...
  Executing return motion...
Returned to start position
Testing JointMotion (confirm start position)...
  Creating joint motion...
  Executing joint motion...
JointMotion executed - robot at starting position
Testing CartesianImpedanceMotion (move up 1cm, compliant)...
  Executing impedance motion...
CartesianImpedanceMotion executed
Returning to starting position...
Returned to start position
Motion classes test PASSED


============================================================
TEST SUMMARY
============================================================
Affine Operations................................. PASSED
Robot Initialization.............................. PASSED
Relative Dynamics Factor.......................... PASSED
Gripper Operations................................ PASSED
State Reading..................................... PASSED
is_in_control Property............................ PASSED
Asynchronous Motion............................... PASSED
Motion Classes.................................... PASSED
============================================================
Total: 8/8 tests passed
============================================================
(robotio) ceylanb@knoppers:~/robot/robot_io$
```
