"""
GELLO teleoperation + trajectory recording for Franka Panda.

Usage:
    conda activate robotio
    cd /home/ceylanb/robot/robot_io
    python robot_io/examples/gello_teleop.py

Controls (cv2 window must be focused):
    r  - toggle recording ON/OFF
    q  - quit and save trajectory

Output:
    trajectory_YYYYMMDD_HHMMSS.npy  - list of dicts with tcp_pos, tcp_orn,
                                       joint_positions, gripper, timestamp.
    Used by GelloMover for drema data gathering, and by simulation replay.
"""

import time
from datetime import datetime

import cv2
import hydra
import numpy as np
from franky import NetworkException

from robot_io.input_devices.gello_input import (
    GRIPPER_CLOSE,
    GRIPPER_OPEN,
    GelloInput,
)
from robot_io.utils.utils import FpsController


@hydra.main(config_path="../conf", config_name="gello_teleop", version_base=None)
def main(cfg):
    robot = hydra.utils.instantiate(cfg.robot)
    cam_manager = hydra.utils.instantiate(cfg.cams)

    print("Moving to neutral pose...")
    robot.move_to_neutral()
    robot.open_gripper(blocking=True)
    print("Done. Physically match GELLO arm to robot pose, then press Enter or Space in the camera window.")

    # Show live camera feed
    while True:
        img_obs = cam_manager.get_images()
        rgb = img_obs.get("rgb_gripper")
        if rgb is not None:
            cv2.imshow("Gripper Cam", rgb[:, :, ::-1])
        canvas = np.zeros((80, 420, 3), dtype=np.uint8)
        cv2.putText(canvas, "Match GELLO to robot, press Enter to start",
                    (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
        cv2.imshow("GELLO Teleop", canvas)
        key = cv2.waitKey(1) & 0xFF
        if key in (13, ord(' ')):  # Enter or Space
            break

    # start_joints: neutral_pose (7) + gripper=0.0
    # DynamixelRobot uses this to resolve 2*pi ambiguity in servo readings.
    current_q = robot.get_state()["joint_positions"]
    start_joints = np.append(current_q, 0.0)

    gello = GelloInput(
        port=cfg.input.port,
        start_joints=start_joints,
        gripper_close_threshold=cfg.input.gripper_close_threshold,
        gripper_hysteresis=cfg.input.gripper_hysteresis,
    )

    # Measure exact per-joint offset at startup and apply every frame.
    # Corrects any residual physical assembly offset not capturable by pi/2-step calibration.
    gello_q_init, _ = gello.get_action()
    robot_q_init = robot.get_state()["joint_positions"]
    q_offset = robot_q_init - gello_q_init["motion"]
    print(f"Startup offsets (rad): {np.round(q_offset, 4)}")

    ll = np.array(cfg.robot.ll)
    ul = np.array(cfg.robot.ul)

    recording = False
    trajectory = []
    prev_gripper = GRIPPER_OPEN
    fps = FpsController(freq=cfg.freq)

    print("Teleop running. Focus the cv2 window: [r] record, [q] quit.")

    try:
        while True:
            # --- Read GELLO ---
            action, _ = gello.get_action()
            q = np.clip(action["motion"] + q_offset, ll, ul)  # offset + clamp to joint limits
            gripper = action["gripper"]       # GRIPPER_OPEN or GRIPPER_CLOSE

            # --- Command arm ---
            try:
                robot.move_async_joint_pos(q)
            except NetworkException:
                robot.recover_from_errors()

            # --- Command gripper (only on state change) ---
            if gripper != prev_gripper:
                if gripper == GRIPPER_CLOSE:
                    robot.close_gripper(blocking=False)
                else:
                    robot.open_gripper(blocking=False)
                prev_gripper = gripper

            # --- Record if active (get_state only when needed) ---
            if recording:
                state = robot.get_state()
                trajectory.append({
                    "tcp_pos":         state["tcp_pos"].copy(),
                    "tcp_orn":         state["tcp_orn"].copy(),
                    "joint_positions": state["joint_positions"].copy(),
                    "gripper":         gripper,
                    "timestamp":       time.time(),
                })

            # --- Gripper camera live view ---
            img_obs = cam_manager.get_images()
            rgb = img_obs.get("rgb_gripper")
            if rgb is not None:
                cv2.imshow("Gripper Cam", rgb[:, :, ::-1])

            # --- cv2 status window + keyboard ---
            canvas = np.zeros((120, 420, 3), dtype=np.uint8)
            rec_str = "REC ON " if recording else "REC OFF"
            rec_col = (0, 0, 200) if recording else (150, 150, 150)
            grp_str = "GRIP: CLOSE" if gripper == GRIPPER_CLOSE else "GRIP: OPEN "
            grp_col = (0, 200, 200) if gripper == GRIPPER_CLOSE else (150, 150, 150)
            cv2.putText(canvas, rec_str, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, rec_col, 2)
            cv2.putText(canvas, grp_str, (220, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, grp_col, 2)
            cv2.putText(canvas, f"Frames: {len(trajectory)}", (10, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)
            cv2.imshow("GELLO Teleop", canvas)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('r'):
                recording = not recording
                print(f"Recording {'ON' if recording else 'OFF'} - {len(trajectory)} frames so far")
            elif key == ord('q'):
                break

            fps.step()

    finally:
        gello.close()
        robot.abort_motion()
        cv2.destroyAllWindows()

        if trajectory:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"trajectory_{ts}.npy"
            np.save(filename, trajectory)
            print(f"Saved {len(trajectory)} frames to {filename}")
        else:
            print("No trajectory recorded.")


if __name__ == "__main__":
    main()
