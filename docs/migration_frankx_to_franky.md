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

### 3. panda_frankx_interface.py Migration

File will be renamed after migration fully finishes/

### Imports (Line 8-14)
- `frankx` → `franky` - New library package name
- Added `Duration` - Required for `CartesianImpedanceMotion` duration parameter
- Removed: `LinearMotion`, `PathMotion`, `LinearRelativeMotion`, `StopMotion` - Not available in franky
- Added: `CartesianMotion`, `CartesianWaypointMotion`, `CartesianWaypoint` - Franky's Cartesian motion classes
- Added: `CartesianImpedanceMotion`, `ExponentialImpedanceMotion` - Franky's impedance control classes
- `Gripper` now from `franky`, not `frankx.gripper` - Module restructured
- `NetworkException` from `franky`, not `_frankx` - Exception handling updated
- Added `ReferenceType as FrankyReferenceType` - Aliased to avoid conflict with `robot_io.utils.utils.ReferenceType`

### Import Path Fix (Line 16)
- Old: `robot_io.robot_io.utils.franky_utils`
- New: `robot_io.utils.franky_utils`
- **Reason:** Corrected double module path error

### Frame Transform (Line 23-26)
- Old: `Affine(0, 0, 0, 0, 0, np.pi)` - 6 scalars (x, y, z, roll, pitch, yaw)
- New: `Affine(translation=np.array([0.0, 0.0, 0.0]).reshape(3, 1), quaternion=np.array([0.0, 1.0, 0.0, 0.0]).reshape(4, 1))`
- Quaternion `[0, 1, 0, 0]` in `[x, y, z, w]` format = 180° rotation around Y-axis
- **Reason:** Franky requires numpy arrays with specific shapes (3x1 and 4x1), not scalars

### Robot Initialization (Line 73)
- Old: `Robot(fci_ip, urdf_path=...)`
- New: `Robot(fci_ip)`
- **Reason:** Franky has built-in Panda model; `urdf_path` parameter removed from API

### Default Behavior (Line 75)
- Old: `self.robot.set_default_behavior()`
- New: (commented out/removed)
- **Reason:** Method doesn't exist in franky API

### Gripper Initialization (Line 89)
- Old: `Gripper(fci_ip, **gripper_params)`
- New: `Gripper(fci_ip)`
- **Reason:** Franky Gripper only accepts hostname; configuration applied in method calls

### State Access (Line 93)
- Old: `self.robot.read_once().F_T_NE`
- New: `self.robot.state.F_T_NE.matrix.T`
- **Critical Fix:** `F_T_NE` returns `Affine` object, must use `.matrix` property to get 4x4 transformation matrix
- **Reason:** Franky uses `.state` property (not `.read_once()` method), and state attributes are Affine objects

### Relative Dynamics Factor (Line 109-113)
- Old: Separate properties `velocity_rel`, `acceleration_rel`, `jerk_rel`
- New: `self.robot.relative_dynamics_factor = RelativeDynamicsFactor(velocity_rel, acceleration_rel, jerk_rel)`
- **Important:** Must use `RelativeDynamicsFactor` object (tuple auto-conversion has type checking issues)
- **Reason:** Franky requires explicit object creation for type safety

### Cartesian Linear Motion (Line 149)
- Old: `WaypointMotion([Waypoint(target_pose)])`
- New: `CartesianWaypointMotion([CartesianWaypoint(target_pose)])`
- **Reason:** Franky renamed motion classes with "Cartesian" prefix for clarity

### Asynchronous Motion (Line 169, 260, 292)
- Old: `self.robot.move_async(motion)` returns thread object
- New: `self.robot.move(motion, asynchronous=True)`
- **Reason:** Franky unified API - same `move()` method with `asynchronous` parameter instead of separate method

### Abort Motion (Line 187)
- Old: `self.current_motion.stop()`
- New: `self.robot.stop()`
- **Note:** Stopping motion mid-execution throws "Move command preempted!" exception (expected behavior)
- Call `robot.recover_from_errors()` after stop to clear error state
- **Reason:** Stop command on robot object, not motion object in franky

### Motion Thread Handling (Line 189-190)
- Old: `self.motion_thread.join()` to wait for completion
- New: No join needed; franky handles internally
- **Reason:** Franky's asynchronous execution doesn't require manual thread management

### Robot State Reading (Line 198)
- Old: `self.robot.read_once()`
- New: `self.robot.state`
- **Reason:** Property access instead of method call in franky

### Gripper Width (Line 206)
- Old: `self.gripper.width()`
- New: `self.gripper.width`
- **Reason:** Property, not method in franky

### TCP Pose (Line 210-218)
- Old: `self.robot.current_pose()` - Method call
- New: `self.robot.end_effector_pose` - Property access
- Returns `Affine` object directly (no need for `.end_effector_pose` attribute)
- Also fixed `pose.translation()` → `pose.translation.flatten()` (property, not method)
- **Reason:** Franky uses property access pattern consistently

### Open Gripper (Line 225-227)
- Old: Blocking handled via parameter
- New: `gripper.open(speed)` for blocking, `gripper.open_async(speed)` for non-blocking
- Parameters: Only `speed` from config (no timeout support)
- **Reason:** Franky has explicit `_async` variants instead of blocking parameter

### Close Gripper (Line 230-248)
- Old: `gripper.close(blocking=...)`
- New: 
  - Blocking: `gripper.grasp(width=0.0, speed, force, epsilon_inner, epsilon_outer)`
  - Non-blocking: `gripper.grasp_async(...)`
- Uses `closing_threshold` from config as `epsilon_inner/epsilon_outer` for grasp detection
- **Reason:** Franky uses force-controlled grasping; no dedicated `close()` method. Width=0.0 with force limit safely closes until object detected.

### Impedance Motion (Line 264-282) - CRITICAL UPDATE
- Old: `ImpedanceMotion(translational_stiffness, rotational_stiffness, nullspace_stiffness, q_d_nullspace, damping_xi)`
- New: `CartesianImpedanceMotion(target_pose, Duration(int(duration * 1000)), translational_stiffness=..., rotational_stiffness=...)`
- **Critical Changes:**
  1. Changed from `ExponentialImpedanceMotion` to `CartesianImpedanceMotion` (ExponentialImpedanceMotion has C++ crash bug in franky)
  2. First two arguments MUST be positional: `target_pose` and `duration`
  3. `Duration` requires integer in **milliseconds** (e.g., `Duration(5000)` for 5 seconds)
  4. Config has `duration: 5.0` (seconds), convert with `int(duration * 1000)`
  5. Added `duration` parameter to `impedance_params` in config file
- **Reason:** CartesianImpedanceMotion requires explicit duration, ExponentialImpedanceMotion is buggy

### Motion Active Check (Line 310)
- Old: `self.motion_thread.is_alive()`
- New: `self.robot.is_in_control`
- **Reason:** Franky provides robot-level property to check control state

## Configuration Changes:

### Added to impedance_params (panda_frankx_interface_policy.yaml)
```yaml
impedance_params:
  translational_stiffness: 600
  rotational_stiffness: 200
  duration: 5.0  # Duration in SECONDS (converted to milliseconds in code)
  # ... existing params
```

### Config Parameters NOT Used:
- `timeout` in gripper_params - Franky doesn't support timeout parameter
- `opening_threshold` - Not used by franky's `open()` method
- Nullspace parameters (`damping_xi`, `use_nullspace`, etc.) - Different implementation in franky

## Known Issues:

1. **ExponentialImpedanceMotion C++ Crash:** Causes segmentation fault in franky library. Workaround: Use `CartesianImpedanceMotion` instead (requires duration parameter).

2. **Motion Preemption Exception:** Calling `robot.stop()` during async motion throws "Move command preempted!" - this is expected behavior. Always call `robot.recover_from_errors()` after stop.

3. **F_T_NE Matrix Access:** State attributes return Affine objects. Must use `.matrix` property to get numpy array, then transpose with `.T`.

## Key Architectural Differences:

1. **No Dynamic Motion Updates:** Franky doesn't support `set_target()` or `set_next_waypoint()` - must create new motion and call `move()` again
2. **Unified Async API:** Single `move()` method with `asynchronous` parameter instead of separate `move_async()`
3. **Property-Based API:** Extensive use of properties instead of methods (`.state`, `.width`, `.is_in_control`, `.current_pose`, `.end_effector_pose`)
4. **Explicit Type Safety:** Requires `RelativeDynamicsFactor` and `Duration` objects instead of tuples/floats
5. **Built-in Robot Model:** Eliminates need for URDF path parameter
6. **Motion Class Naming:** More explicit names (CartesianWaypointMotion vs WaypointMotion)

## Test Results:

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
(robotio) ceylanb@knoppers:~/robot/robot_io$ ^C
