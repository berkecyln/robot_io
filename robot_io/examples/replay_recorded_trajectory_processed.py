from pathlib import Path

import hydra
import numpy as np
from robot_io.utils.utils import to_relative, to_relative_action_dict

N_DIGITS = 6
MAX_REL_POS = 0.02
MAX_REL_ORN = 0.05


def get_ep_start_end_ids(path):
    return np.sort(np.load(Path(path) / "ep_start_end_ids.npy"), axis=0)


def get_frame(path, i):
    filename = Path(path) / f"episode_{i:0{N_DIGITS}d}.npz"
    return np.load(filename, allow_pickle=True)


def reset(env, path, i):
    data = get_frame(path, i)
    robot_state = data["robot_obs"]
    gripper_state = "open" if robot_state[6] > 0.07 else "closed"
    env.reset(
        target_pos=robot_state[:3],
        target_orn=robot_state[3:6],
        gripper_state=gripper_state,
    )


def get_action(path, i, use_rel_actions=False, control_frame="tcp"):
    frame = get_frame(path, i)
    if use_rel_actions:
        if control_frame == "tcp":
            action = frame["rel_actions_gripper"]
        else:
            action = frame["rel_actions_world"]
        print(action)
        pos = action[:3] * MAX_REL_POS
        orn = action[3:6] * MAX_REL_ORN

        new_action = {"motion": (pos, orn, action[-1]), "ref": "rel"}
    else:
        action = frame["actions"]
        new_action = {"motion": (action[:3], action[3:6], action[-1]), "ref": "abs"}
    return new_action

@hydra.main(config_path="../conf", config_name="replay_recorded_trajectory")
def main(cfg):
    """
    Replay a recorded trajectory, either with absolute actions or relative actions.

    Args:
        cfg: Hydra config
    """
    robot = hydra.utils.instantiate(cfg.robot)
    env = hydra.utils.instantiate(cfg.env, robot=robot)

    use_rel_actions = cfg.use_rel_actions
    ep_start_end_ids = get_ep_start_end_ids(cfg.load_dir)

    env.reset()
    for idx, (start_idx, end_idx) in enumerate(ep_start_end_ids):
        reset(env, cfg.load_dir, start_idx)
        for i in range(start_idx + 1, end_idx + 1):
            action = get_action(cfg.load_dir, i, use_rel_actions,
                                cfg.robot.rel_action_params.relative_action_control_frame)
            obs, _, _, _ = env.step(action)
            env.render()


if __name__ == "__main__":
    main()
