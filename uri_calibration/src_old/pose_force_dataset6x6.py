import json
import glob
import os
import torch
from torch.utils.data import Dataset, random_split


class PoseForceDataset(Dataset):
    """
    Loads all record_pose_and_force_*.json files from logs_dir.
    Each step is one sample: x = wrench_tcp (6,), y = delta (6,).
    """

    def __init__(self, logs_dir: str):
        self.samples: list[tuple[torch.Tensor, torch.Tensor]] = []

        pattern = os.path.join(logs_dir, "record_pose_and_force_*.json")
        files = sorted(glob.glob(pattern))
        if not files:
            raise FileNotFoundError(f"No matching files in {logs_dir}")

        for path in files:
            with open(path) as f:
                steps = json.load(f)
            for step in steps:
                x = torch.tensor(step["ayal_wrench_tcp"], dtype=torch.float32)
                y = torch.tensor(step["ayal_delta"], dtype=torch.float32)
                self.samples.append((x, y))

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int) -> tuple[torch.Tensor, torch.Tensor]:
        return self.samples[idx]

    def split(self, train_ratio: float = 0.8, seed: int = 42):
        n_train = int(len(self) * train_ratio)
        n_val = len(self) - n_train
        return random_split(self, [n_train, n_val],
                            generator=torch.Generator().manual_seed(seed))
