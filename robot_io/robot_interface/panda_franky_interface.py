import time

import cv2
import numpy as np
import hydra.utils

from robot_io.control.rel_action_control import RelActionControl
from robot_io.robot_interface.base_robot_interface import BaseRobotInterface
from franky import (
    Robot, Gripper, Affine, RelativeDynamicsFactor, Duration,
    JointMotion, JointWaypointMotion, JointWaypoint, JointState,
    CartesianMotion, CartesianWaypointMotion, CartesianWaypoint,
    CartesianImpedanceMotion, ExponentialImpedanceMotion,
    NetworkException, CommandException, ReferenceType as FrankyReferenceType
)

from robot_io.utils.franky_utils import to_affine
from robot_io.utils.utils import pos_orn_to_matrix, get_git_root, ReferenceType
import logging
log = logging.getLogger(__name__)

# Franky uses continuous end-effector control in Cartesian space.
# Transform TCP orientation so z-axis faces down for consistency with other robot interfaces.
NE_T_EE = EE_T_NE = Affine(
    translation=np.array([0.0, 0.0, 0.0]).reshape(3, 1),
    quaternion=np.array([0.0, 0.0, 1.0, 0.0]).reshape(4, 1) # 180 deg around Z-axis
)

# align the output of force torque reading with the EE frame
WRENCH_FRAME_CONV = np.diag([-1, 1, 1, -1, 1, 1])  # np.eye(6)


class PandaFrankYInterface(BaseRobotInterface):
    """
    Robot control interface for Franka Emika Panda robot to be used on top of this Franky fork

    Args:
        fci_ip: IPv4 address of Franka Control Interface (FCI).
        urdf_path: URDF of panda robot (change default config e.g. when mounting different fingers).
        neutral_pose: Joint angles in rad.
        ll: Lower joint limits in rad.
        ul: Upper joint limits in rad.
        ik: Config of the inverse kinematic solver.
        workspace_limits: Workspace limits defined as a bounding box or as hollow cylinder.
        libfranka_params: DictConfig of params for libfranka.
        use_impedance: If True, use impedance control whenever it is possible.
        franky_params: DictConfig of general params for Franky.
        impedance_params: DictConfig of params for Franky impedance motion.
        rel_action_params: DictConfig of params for relative action control.
        gripper_params: DictConfig of params for Franky gripper.
        waypoint_motion_params: DictConfig of params for Cartesian waypoint motions.
    """
    def __init__(self,
                 fci_ip,
                 urdf_path,
                 neutral_pose,
                 ll,
                 ul,
                 ik,
                 workspace_limits,
                 libfranka_params,
                 use_impedance,
                 franky_params,
                 impedance_params,
                 rel_action_params,
                 gripper_params,
                 waypoint_motion_params):
        self.name = "panda"
        self.neutral_pose = neutral_pose
        self.ll = ll
        self.ul = ul

        # robot
        self.robot = Robot(fci_ip) # franky does not need urdf path
        self.robot.recover_from_errors()
        #self.robot.set_default_behavior()
        self.libfranka_params = libfranka_params
        self.set_robot_params(libfranka_params, franky_params)

        # impedance
        self.use_impedance = use_impedance
        self.impedance_params = impedance_params
        
        # waypoint motion
        self.waypoint_motion_params = waypoint_motion_params

        self.rel_action_converter = RelActionControl(ll=ll, ul=ul, workspace_limits=workspace_limits,
                                                     **rel_action_params)

        self.motion_thread = None
        self.current_motion = None
        self.last_motion_time = 0.0
        self.min_motion_interval = 0.15

        self.gripper = Gripper(fci_ip)
        self.gripper_params = gripper_params
        self.gripper.homing()
        self.open_gripper(blocking=True)

        # F_T_NE is the transformation from nominal end-effector (NE) frame to flange (F) frame.
        F_T_NE = self.robot.state.F_T_NE.matrix
        self.ik_solver = hydra.utils.instantiate(ik, F_T_NE=F_T_NE)

        self.reference_type = ReferenceType.ABSOLUTE
        super().__init__(ll=ll, ul=ul)

    def __del__(self):
        self.abort_motion()

    def set_robot_params(self, libfranka_params, franky_params):
        # params of libfranka
        self.robot.set_collision_behavior(libfranka_params.contact_torque_threshold,
                                          libfranka_params.collision_torque_threshold,
                                          libfranka_params.contact_force_threshold,
                                          libfranka_params.collision_force_threshold)
        self.robot.set_joint_impedance(libfranka_params.franka_joint_impedance)

        # params of franky
        self.robot.relative_dynamics_factor = RelativeDynamicsFactor(
            franky_params.velocity_rel,
            franky_params.acceleration_rel,
            franky_params.jerk_rel
        )

    def move_to_neutral(self):
        return self.move_joint_pos(self.neutral_pose)

    def move_cart_pos_abs_ptp(self, target_pos, target_orn):
        self.reference_type = ReferenceType.ABSOLUTE
        # if self.use_impedance:
        #     log.warning("Impedance motion is not available for synchronous motions. Not using impedance.")
        q_desired = self._inverse_kinematics(target_pos, target_orn)
        return self.move_joint_pos(q_desired)

    def move_cart_pos_rel_ptp(self, rel_target_pos, rel_target_orn):
        target_pos, target_orn = self.rel_action_converter.to_absolute(rel_target_pos, rel_target_orn, self.get_state(), self.reference_type)
        self.reference_type = ReferenceType.RELATIVE

        q_desired = self._inverse_kinematics(target_pos, target_orn)
        self.abort_motion()
        self.robot.move(JointMotion(q_desired))

    def move_cart_pos_rel_lin(self, rel_target_pos, rel_target_orn):
        target_pos, target_orn = self.rel_action_converter.to_absolute(rel_target_pos, rel_target_orn, self.get_state(), self.reference_type)
        self.reference_type = ReferenceType.RELATIVE
        self.abort_motion()
        target_pose = to_affine(target_pos, target_orn) * NE_T_EE
        self.current_motion = CartesianMotion(target_pose, FrankyReferenceType.Absolute, self.robot.relative_dynamics_factor)
        self.robot.move(self.current_motion)
        self.current_motion = None

    def move_async_cart_pos_rel_lin(self, rel_target_pos, rel_target_orn):
        target_pos, target_orn = self.rel_action_converter.to_absolute(rel_target_pos, rel_target_orn, self.get_state(), self.reference_type)
        self.reference_type = ReferenceType.RELATIVE
        self._franky_async_impedance_motion(target_pos, target_orn)

    def move_async_cart_pos_abs_ptp(self, target_pos, target_orn):
        self.reference_type = ReferenceType.ABSOLUTE
        if self.use_impedance:
            log.warning("Impedance motion for cartesian PTP is currently not implemented. Not using impedance.")
        
        q_desired = self._inverse_kinematics(target_pos, target_orn)
        self.move_async_joint_pos(q_desired)

    def move_cart_pos_abs_lin(self, target_pos, target_orn):
        self.reference_type = FrankyReferenceType.Absolute
        # if self.use_impedance:
        #     log.warning("Impedance motion for cartesian LIN is currently not implemented. Not using impedance.")
        self.abort_motion()
        target_pose = to_affine(target_pos, target_orn) * NE_T_EE
        self.current_motion = CartesianMotion(target_pose, self.reference_type, self.robot.relative_dynamics_factor)
        self.robot.move(self.current_motion)
        self.current_motion = None

    def move_cart_waypoints(self, waypoints_pos, waypoints_orn):
        """
        Move through multiple Cartesian waypoints in one smooth motion.
        
        Args:
            waypoints_pos: List of positions [(x,y,z), (x,y,z), ...]
            waypoints_orn: List of orientations [quat1, quat2, ...] where quat = (x,y,z,w)
        """
        self.reference_type = ReferenceType.ABSOLUTE
        
        # Convert all waypoints to CartesianWaypoint objects
        cartesian_waypoints = []
        for pos, orn in zip(waypoints_pos, waypoints_orn):
            target_pose = to_affine(pos, orn) * NE_T_EE
            cartesian_waypoints.append(CartesianWaypoint(target_pose))
        
        # Execute motion
        self.abort_motion()
        self.current_motion = CartesianWaypointMotion(
            cartesian_waypoints,
            relative_dynamics_factor=self.waypoint_motion_params.relative_dynamics_factor
        )
        self.robot.move(self.current_motion)
        self.current_motion = None
        

    def move_async_cart_pos_abs_lin(self, target_pos, target_orn):
        self.reference_type = ReferenceType.ABSOLUTE
        if self.use_impedance:
            self._franky_async_impedance_motion(target_pos, target_orn)
        else:
            self._franky_async_lin_motion(target_pos, target_orn)

    def move_async_joint_pos(self, joint_positions):
        self.current_motion = JointWaypointMotion([JointWaypoint(JointState(joint_positions))], self.robot.relative_dynamics_factor, return_when_finished=False)
        self.motion_thread = self.robot.move(self.current_motion, asynchronous=True)

    def move_joint_pos(self, joint_positions):
        self.reference_type = ReferenceType.JOINT
        self.abort_motion()
        success = self.robot.move(JointMotion(JointState(joint_positions), relative_dynamics_factor=self.robot.relative_dynamics_factor))
        if not success:
            self.robot.recover_from_errors()
        return success

    def abort_motion(self):
        if self.current_motion is not None:
            try:
                self.robot.stop()
            except (CommandException, NetworkException):
                pass  # reflex or network issue during stop
            self.current_motion = None
        if self.motion_thread is not None:
            self.motion_thread = None
        while 1:
            try:
                self.robot.recover_from_errors()
                break
            except NetworkException:
                time.sleep(0.01)
                continue

    def get_state(self):
        # if self.current_motion is None:
        #     _state = self.robot.state
        # else:
        #     _state = self.current_motion.get_robot_state()
        _state = self.robot.state

        pos, orn = self.get_tcp_pos_orn()

        state = {"tcp_pos": pos,
                 "tcp_orn": orn,
                 "joint_positions": np.array(_state.q),
                 "gripper_opening_width": self.gripper.width,
                 "force_torque": WRENCH_FRAME_CONV @ np.array(_state.K_F_ext_hat_K),
                 "contact": np.array(_state.cartesian_contact)}
        return state

    def get_tcp_pos_orn(self):
        # if self.current_motion is None:
        #     pose = self.robot.current_pose.end_effector_pose * EE_T_NE
        # else:
        #     pose = self.current_motion.end_effector_pose * EE_T_NE
        #     while np.all(pose.translation == 0):
        #         pose = self.current_motion.end_effector_pose * EE_T_NE
        #         time.sleep(0.01)
        pose = self.robot.current_pose.end_effector_pose * EE_T_NE

        pos, orn = np.array(pose.translation).flatten(), np.array(pose.quaternion).flatten()
        return pos, orn

    def get_tcp_pose(self):
        return pos_orn_to_matrix(*self.get_tcp_pos_orn())

    def open_gripper(self, blocking=False):
        if blocking:
            self.gripper.open(self.gripper_params.speed)
        else:
            self.gripper.open_async(self.gripper_params.speed)
    
    def close_gripper(self, blocking=False):
        if blocking:
            self.gripper.grasp(
                width=0.0,
                speed=self.gripper_params.speed,
                force=self.gripper_params.force,
                epsilon_inner=self.gripper_params.closing_threshold,
                epsilon_outer=self.gripper_params.closing_threshold
            )
        else:
            self.gripper.grasp_async(
                width=0.0,
                speed=self.gripper_params.speed,
                force=self.gripper_params.force,
                epsilon_inner=self.gripper_params.closing_threshold,
                epsilon_outer=self.gripper_params.closing_threshold
            )

    def _franky_async_impedance_motion(self, target_pos, target_orn):
        """
        Start new async impedance motion. Do not call this directly.

        Args:
            target_pos: (x,y,z)
            target_orn: quaternion (x,y,z,w) | euler_angles (α,β,γ)
        """
        # Rate limit motion commands
        current_time = time.time()
        time_since_last_motion = current_time - self.last_motion_time
        
        # Only send new motion command if enough time has passed or robot not in control
        if time_since_last_motion < self.min_motion_interval and self.robot.is_in_control:
            return
        
        target_pose = to_affine(target_pos, target_orn) * NE_T_EE
        self.current_motion = self._new_impedance_motion(target_pose)
        self.robot.move(self.current_motion, asynchronous=True)
        self.last_motion_time = current_time

    def _new_impedance_motion(self, target_pose):
        """
        Create new franky impedance motion with the params specified in config file.
    
        Args:
            target_pose: Target Affine pose
    
        Returns:
            Impedance motion object.
        """
        # return ExponentialImpedanceMotion(
        #     target_pose,
        #     FrankyReferenceType.Absolute,
        #     self.impedance_params.translational_stiffness,
        #     self.impedance_params.rotational_stiffness,
        #     None, # force_constraints
        #     self.impedance_params.exponantial_decay
        # )
        return CartesianMotion(target_pose, FrankyReferenceType.Absolute, self.robot.relative_dynamics_factor)


    def _franky_async_lin_motion(self, target_pos, target_orn):
        """
        Start new Waypaint motion without impedance. Do not call this directly.

        Args:
            target_pos: (x,y,z)
            target_orn: quaternion (x,y,z,w) | euler_angles (α,β,γ)
        """
        # Rate limit motion commands
        current_time = time.time()
        time_since_last_motion = current_time - self.last_motion_time
        
        # Only send new motion command if enough time has passed or robot not in control
        if time_since_last_motion < self.min_motion_interval and self.robot.is_in_control:
            return
        
        target_pose = to_affine(target_pos, target_orn) * NE_T_EE
        self.current_motion = CartesianWaypointMotion([CartesianWaypoint(target_pose), ], return_when_finished=False)
        self.motion_thread = self.robot.move(self.current_motion, asynchronous=True)
        self.last_motion_time = current_time

    def _inverse_kinematics(self, target_pos, target_orn):
        """
        Find inverse kinematics solution with the ik solver specified in config file.

        Args:
            target_pos: cartesian target position (x,y,z).
            target_orn: cartesian target orientation, quaternion (x,y,z,w) | euler_angles (α,β,γ).

        Returns:
            Target joint angles in rad.
        """
        current_q = self.get_state()['joint_positions']
        new_q = self.ik_solver.inverse_kinematics(target_pos, target_orn, current_q)
        return new_q

    def _is_active(self, motion):
        """Returns True if there is a currently active motion with the same type as motion."""
        return self.current_motion is not None and isinstance(self.current_motion, motion) and self.robot.is_in_control

    def visualize_external_forces(self, canvas_width=500):
        """
        Display the external forces (x,y,z) and torques (a,b,c) of the tcp frame.

        Args:
            canvas_width: Display width in pixel.

        """
        canvas = np.ones((300, canvas_width, 3))
        forces = self.get_state()["force_torque"]
        contact = np.array(self.libfranka_params.contact_force_threshold)
        collision = np.array(self.libfranka_params.collision_force_threshold)
        left = 10
        right = canvas_width - left
        width = right - left
        height = 30
        y = 10
        for i, (lcol, lcon, f, ucon, ucol) in enumerate(zip(-collision, -contact, forces, contact, collision)):
            cv2.rectangle(canvas, [left, y], [right, y + height], [0, 0, 0], thickness=2)
            force_bar_pos = int(left + width * (f - lcol) / (ucol - lcol))
            cv2.line(canvas, [force_bar_pos, y], [force_bar_pos, y + height], thickness=4, color=[0, 0, 1])
            ucon_bar_pos = int(left + width * (ucon - lcol) / (ucol - lcol))
            cv2.line(canvas, [ucon_bar_pos, y], [ucon_bar_pos, y + height], thickness=2, color=[1, 0, 0])
            lcon_bar_pos = int(left + width * (lcon - lcol) / (ucol - lcol))
            cv2.line(canvas, [lcon_bar_pos, y], [lcon_bar_pos, y + height], thickness=2, color=[1, 0, 0])

            y += height + 10
        cv2.imshow("external_forces", canvas)
        cv2.waitKey(1)


@hydra.main(config_path="../conf", config_name="panda_teleop.yaml")
def main(cfg):
    robot = hydra.utils.instantiate(cfg.robot)
    robot.move_to_neutral()
    robot.close_gripper()
    time.sleep(1)
    print(robot.get_tcp_pose())
    exit()
    # print(robot.get_state()["gripper_opening_width"])
    # time.sleep(2)
    # robot.open_gripper()
    # time.sleep(1)
    # exit()
    # pos, orn = robot.get_tcp_pos_orn()
    # pos[0] += 0.2
    # pos[2] -= 0.1
    # # pos[2] -= 0.05
    # print("move")
    # robot.move_cart_pos_abs_ptp(pos, orn)
    # time.sleep(5)
    # print("done!")


if __name__ == "__main__":
    main()
