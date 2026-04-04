import hydra
import numpy as np


# GELLO calibration pose for Franka Panda.
# Move the robot to this configuration, then physically match your GELLO arm
# to the same pose before running gello_get_offset.py to compute the joint offsets.
GELLO_CALIB_JOINTS = np.array([0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.0])


@hydra.main(config_path="../conf", config_name="panda_teleop")
def main(cfg):
    robot = hydra.utils.instantiate(cfg.robot)

    print("Moving to GELLO calibration pose...")
    print(f"Target joints (rad): {GELLO_CALIB_JOINTS}")
    robot.move_joint_pos(GELLO_CALIB_JOINTS)
    print("Done.")
    print("Now match your GELLO arm to this pose, then run gello_get_offset.py.")


if __name__ == "__main__":
    main()
