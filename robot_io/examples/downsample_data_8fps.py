from copy import deepcopy
import json
import os
from pathlib import Path
import shutil
import numpy as np
import hydra
from omegaconf import OmegaConf
from tqdm import tqdm
from robot_io.utils.utils import to_relative_all_frames

MAX_REL_POS = 0.08
MAX_REL_ORN = 0.20

# Copied from robot_io/examples/preprocess_data.py
def compute_rel_action(tcp_pos, tcp_orn, next_tcp_pos, next_tcp_orn, gripper_action):
    rel_pos_orn_dct = to_relative_all_frames(
        tcp_pos, tcp_orn, next_tcp_pos, next_tcp_orn
    )
    rel_pos_orn_dct_new = {}
    for frame, (rel_pos, rel_orn) in rel_pos_orn_dct.items():
        # clipped_rel_pos = np.clip(rel_pos, -MAX_REL_POS, MAX_REL_POS) / MAX_REL_POS
        # clipped_rel_orn = np.clip(rel_orn, -MAX_REL_ORN, MAX_REL_ORN) / MAX_REL_ORN
        clipped_rel_pos, clipped_rel_orn = rel_pos / MAX_REL_POS, rel_orn / MAX_REL_ORN
        if clipped_rel_pos[0] > 1 or clipped_rel_pos[1] > 1 or clipped_rel_pos[2] > 1:
            print("clipped_rel_pos: ", clipped_rel_pos)
        if clipped_rel_orn[0] > 1 or clipped_rel_orn[1] > 1 or clipped_rel_orn[2] > 1:
            print("clipped_rel_orn: ", clipped_rel_orn)
        rel_action = np.concatenate(
            [clipped_rel_pos, clipped_rel_orn, [gripper_action]]
        )
        rel_pos_orn_dct_new[frame] = rel_action
    return rel_pos_orn_dct_new


@hydra.main(config_path="../conf", config_name="downsample_data")
def main(cfg):
    src_path = Path(cfg.dataset_root)
    assert src_path.is_dir(), "The path of the src dataset must be a dir"

    dest_path = Path(cfg.output_dir)
    dest_path.mkdir(parents=True, exist_ok=True)

    # Iterate subdirs inside src dir
    old_to_new_ids = {}
    subdirs = [f for f in src_path.iterdir() if f.is_dir()]
    subdirs = ["."] if len(subdirs) == 0 else subdirs
    offsets = [0, 1] if cfg.repeat_info else [0]
    for subdir in subdirs:
        # Create dest dir
        (dest_path / subdir).mkdir(parents=True, exist_ok=True)
        # Transfer transformed files to dest dir
        new_i = 0
        new_ep_lens = []
        new_ep_start_end_ids = []
        ep_start_end_ids = np.load(src_path / subdir / "ep_start_end_ids.npy")
        for start, end in tqdm(ep_start_end_ids):
            episode_frames = {}
            for offset in offsets:
                new_start = new_i
                for old_i in tqdm(range(start + 2 + offset, end, 4)):
                    if old_i + 2 <= end:
                        if not cfg.high_memory_mode:
                            episode_frames = {}
                        # Load following frames if they have not been loaded
                        if old_i - 2 not in episode_frames:
                            aux_data = np.load(src_path / subdir / f"episode_{old_i - 2:06d}.npz", allow_pickle=True)
                            episode_frames[old_i - 2] = dict(aux_data)
                        if old_i not in episode_frames:
                            aux_data = np.load(src_path / subdir / f"episode_{old_i:06d}.npz", allow_pickle=True)
                            episode_frames[old_i] = dict(aux_data)
                        if old_i + 2 not in episode_frames:
                            aux_data = np.load(src_path / subdir / f"episode_{old_i + 2:06d}.npz", allow_pickle=True)
                            episode_frames[old_i + 2] = dict(aux_data)

                        # Get rel_action from current_state -> current_state + 2
                        data = deepcopy(episode_frames[old_i])
                        data["actions"] = episode_frames[old_i + 2]["actions"]
                        rel_actions = compute_rel_action(
                            tcp_pos=episode_frames[old_i - 2]["actions"][:3],
                            tcp_orn=episode_frames[old_i - 2]["actions"][3:6],
                            next_tcp_pos=episode_frames[old_i + 2]["actions"][:3],
                            next_tcp_orn=episode_frames[old_i + 2]["actions"][3:6],
                            gripper_action=episode_frames[old_i + 2]["actions"][-1],
                        )
                        data["rel_actions_world"] = rel_actions["world_frame"]
                        data["rel_actions_gripper"] = rel_actions["gripper_frame"]
                        np.savez_compressed(dest_path / subdir / f"episode_{new_i:06d}.npz", **data)
                        old_to_new_ids[old_i] = new_i
                        new_i += 1
                new_end = new_i - 1
                new_ep_len = new_end - new_start + 1
                new_ep_start_end_ids.append((new_start, new_end))
                new_ep_lens.append(new_ep_len)

        np.save(dest_path / subdir / "ep_lens.npy", new_ep_lens)
        np.save(dest_path / subdir / "ep_start_end_ids.npy", new_ep_start_end_ids)
        if (src_path / subdir / "statistics.yaml").is_file():
            shutil.copy(src_path / subdir / "statistics.yaml", dest_path / subdir)
        if (src_path / subdir / ".hydra").is_dir():
            os.makedirs(dest_path / subdir / ".hydra")
            shutil.copytree(src_path / subdir / ".hydra", dest_path / subdir / ".hydra", dirs_exist_ok=True)
            cfg = OmegaConf.load(src_path / subdir / ".hydra/merged_config.yaml")
            cfg.env.control_freq = 15
            OmegaConf.save(cfg, dest_path / subdir / ".hydra/merged_config.yaml")
        with open(dest_path / subdir / "old_to_new_ids.json", "w") as file:
            json.dump(old_to_new_ids, file)


if __name__ == "__main__":
    main()
