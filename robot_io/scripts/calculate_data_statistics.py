import os
import hydra
import numpy as np
import yaml
from tqdm import tqdm
from pathlib import Path

N_DIGITS = 6
FILENAME = "episode"

def get_frame(path, i):
    filename = Path(path) / f"{FILENAME}_{i:0{N_DIGITS}d}.npz"
    return np.load(filename, allow_pickle=True)

def calculate_stats(dataset_root, obs_keys, act_keys, target_name):
    ep_start_end_ids_all = np.load(Path(dataset_root) / "ep_start_end_ids.npy")

    act_tracker = {}
    for key in act_keys:
        act_tracker.update({f"{key}" : [np.ones(7) * np.inf, -np.ones(7) * np.inf]})
    
    for start_idx, end_idx in tqdm(ep_start_end_ids_all):
        for i in range(start_idx, end_idx + 1):
            data = get_frame(dataset_root, i)
            for key in act_keys:
                key_min, key_max = act_tracker[key]
                act_tracker[key][0] = np.minimum(data[key], key_min)
                act_tracker[key][1] = np.maximum(data[key], key_max)
    
    # Write to a yaml file where {key}_min and {key}_max are the min and max values of the key
    with open(Path(dataset_root) / f"{target_name}.yaml", "w") as f:
        yaml.dump(act_tracker, f)

    for k, v in act_tracker.items():
        print(k, v)

@hydra.main(config_path="../conf", config_name="calculate_data_statistics")
def main(cfg):
    dataset_root = cfg.dataset_root
    obs_keys = cfg.obs_keys
    act_keys = cfg.act_keys
    target_name = cfg.target_name

    calculate_stats(dataset_root, obs_keys, act_keys, target_name)


if __name__ == "__main__":
    main()