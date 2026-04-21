import numpy as np

from gello.agents.gello_agent import GelloAgent

GRIPPER_OPEN = 1
GRIPPER_CLOSE = -1

FRANKA_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO4UAS-if00-port0"


class GelloInput:
    """
    GELLO leader arm input device for Franka Panda teleoperation.

    Wraps GelloAgent (gello_software) which handles Dynamixel communication,
    joint offsets, signs, and gripper normalization via PORT_CONFIG_MAP.

    Args:
        port: USB serial port for GELLO. Must be in PORT_CONFIG_MAP.
        start_joints: 8D array (7 arm + 1 gripper) matching the robot pose
                      at initialization time. Used for 2*pi disambiguation.
        gripper_close_threshold: Gripper value in [0,1] above which gripper
                                 is considered closed. 0=open, 1=closed.
        gripper_hysteresis: Dead band around the threshold. Gripper state only
                            changes when the value moves outside [threshold ±
                            hysteresis]. Prevents rapid toggling near the boundary.
    """

    def __init__(
        self,
        port=FRANKA_PORT,
        start_joints=None,
        gripper_close_threshold=0.5,
        gripper_hysteresis=0.1,
    ):
        self.gripper_close_threshold = gripper_close_threshold
        self.gripper_hysteresis = gripper_hysteresis
        self._gripper_state = GRIPPER_OPEN
        self._agent = GelloAgent(port=port, start_joints=np.array(start_joints) if start_joints is not None else None)

    def get_action(self):
        """
        Read current GELLO state and return a robot action dict.

        Returns:
            action: dict with keys:
                "motion"  - np.ndarray (7,) calibrated arm joint positions in radians
                "gripper" - GRIPPER_OPEN (1) or GRIPPER_CLOSE (-1)
                "ref"     - "joint"
            record_info: empty dict (recording is keyboard-controlled in teleop script)
        """
        joints = self._agent.act({})          # shape (8,): arm[0:7] + gripper[7] in [0,1]
        arm_joints = joints[:7]
        gripper_raw = joints[7]

        # Hysteresis: only switch state when crossing threshold ± hysteresis band.
        # Prevents the gripper from toggling when the raw value oscillates near the threshold.
        if self._gripper_state == GRIPPER_OPEN and gripper_raw > self.gripper_close_threshold + self.gripper_hysteresis:
            self._gripper_state = GRIPPER_CLOSE
        elif self._gripper_state == GRIPPER_CLOSE and gripper_raw < self.gripper_close_threshold - self.gripper_hysteresis:
            self._gripper_state = GRIPPER_OPEN
        gripper = self._gripper_state

        action = {"motion": arm_joints, "gripper": gripper, "ref": "joint"}
        return action, {}

    def close(self):
        """Stop the Dynamixel background thread and close the serial port."""
        self._agent._robot._driver.close()
