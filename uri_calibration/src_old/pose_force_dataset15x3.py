import json
import glob
import math
import os

import torch
from torch.utils.data import Dataset, random_split


def _force_mag(wrench_tcp):
    fx, fy, fz = wrench_tcp[0], wrench_tcp[1], wrench_tcp[2]
    return math.sqrt(fx * fx + fy * fy + fz * fz)


class PoseForceDataset(Dataset):
    """Loads step logs as (x, y) samples for the dual-arm peg controller.

    x (6,): ayal_wrench_tcp
    y (6,): `label_key` (full 6-dim pose delta: translation + rotation).

    `path` may be a single .json file or a directory; when it's a directory,
    files are picked up via `pattern` (default matches numbered
    auto_peg_cycle_*.json, skipping the bare auto_peg_cycle.json).

    `flip_pull_sign`: if True, negate the label for any step whose `phase`
    starts with "pull" (covers `pull`, `pull_straighten`, `pull_backoff`).
    `min_wrench`: drop samples where |F_ayal| < this (Newtons, force only,
    not torque). 0 disables filtering.
    """

    def __init__(
        self,
        path: str,
        pattern: str = "auto_peg_cycle_*.json",
        label_key: str = "ayal_delta_to_mate",
        flip_pull_sign: bool = True,
        min_wrench: float = 1.0,
    ):
        self.samples: list[tuple[torch.Tensor, torch.Tensor]] = []
        self.n_dropped_low_wrench = 0
        self.n_flipped = 0

        if os.path.isfile(path):
            files = [path]
        else:
            files = sorted(glob.glob(os.path.join(path, pattern)))
        if not files:
            raise FileNotFoundError(f"No matching files for {path!r} (pattern {pattern!r})")

        for fpath in files:
            with open(fpath) as f:
                steps = json.load(f)
            for step in steps:
                ayal_w = step["ayal_wrench_tcp"]

                if min_wrench > 0:
                    if _force_mag(ayal_w) < min_wrench:
                        self.n_dropped_low_wrench += 1
                        continue

                x = torch.tensor(list(ayal_w), dtype=torch.float32)

                y_vals = list(step[label_key])
                if flip_pull_sign and str(step.get("phase", "")).startswith("pull"):
                    y_vals = [-v for v in y_vals]
                    self.n_flipped += 1
                y = torch.tensor(y_vals, dtype=torch.float32)
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
