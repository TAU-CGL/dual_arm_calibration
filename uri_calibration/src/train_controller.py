import math
import os
import torch
import argparse
import json
import glob
import uri_if
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset, random_split

# --- config ---
DEFAULT_DATA = uri_if._RMP_LAB_ROOTos.path.normpath(
    os.path.join(os.path.dirname(__file__), "../../dual_arm_peg/output")
)
LOGS_DIR = os.path.join(os.path.dirname(__file__), "../logs/auto_record_v1")
OUT_DIR = os.path.join(os.path.dirname(__file__), "../logs")
EPOCHS = 2000
BATCH_SIZE = 32
LR = 1e-3
TRAIN_RATIO = 0.8
SEED = 42
# --------------

# -------------- 6X6 dataset and training (force(6) -> delta_pose(6)) --------------
class PoseForceDataset6x6(Dataset):
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

def train_lqr_6x6():
    torch.manual_seed(SEED)

    ds = PoseForceDataset6x6(LOGS_DIR)
    train_ds, val_ds = ds.split(TRAIN_RATIO, SEED)

    train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=len(val_ds))

    # LQR gain: delta = K @ wrench_tcp
    model = nn.Linear(6, 6, bias=False)
    optimizer = torch.optim.Adam(model.parameters(), lr=LR)
    loss_fn = nn.MSELoss()

    best_val_loss = float("inf")
    best_K = None

    for epoch in range(1, EPOCHS + 1):
        model.train()
        for x, y in train_loader:
            optimizer.zero_grad()
            loss_fn(model(x), y).backward()
            optimizer.step()

        model.eval()
        with torch.no_grad():
            x_val, y_val = next(iter(val_loader))
            val_loss = loss_fn(model(x_val), y_val).item()

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            best_K = model.weight.detach().clone()

        if epoch % 50 == 0:
            print(f"epoch {epoch:4d}  val_mse={val_loss:.6f}")

    out_path = os.path.join(OUT_DIR, "lqr_gain_K.pt")
    torch.save(best_K, out_path)
    print(f"\nBest val MSE: {best_val_loss:.6f}")
    print(f"K matrix saved to {out_path}")
    print(f"\nK =\n{best_K}")

# -------------- 15X3 dataset and training ({force_uri(6),force_ayal(6),distance(1),manipulabilities(2)} -> delta_orientation(6)) --------------
def _force_mag(wrench_tcp):
    fx, fy, fz = wrench_tcp[0], wrench_tcp[1], wrench_tcp[2]
    return math.sqrt(fx * fx + fy * fy + fz * fz)

class PoseForceDataset15x3(Dataset):
    """Loads step logs as (x, y) samples for the dual-arm peg controller.

    x (15,): uri_wrench_tcp (6) + ayal_wrench_tcp (6) + distance (1) + manipulabilities (2)
    y (3,): `label_key` (full 3-dim orientation delta: rotation only).

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

def train_lqr_15x3():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "data",
        nargs="?",
        default=DEFAULT_DATA,
        help="Path to a single JSON log or a directory of logs.",
    )
    parser.add_argument("--pattern", default="auto_peg_cycle_*.json")
    parser.add_argument("--label-key", default="ayal_delta_to_mate")
    parser.add_argument(
        "--no-flip-pull",
        action="store_true",
        help="Don't negate the rotation label for pull-phase steps (default: do flip).",
    )
    parser.add_argument(
        "--min-wrench",
        type=float,
        default=1.0,
        help="Drop samples where max(|F_ayal|, |F_uri|) < this (N). 0 disables.",
    )
    args = parser.parse_args()

    torch.manual_seed(SEED)

    ds = PoseForceDataset15x3(
        args.data,
        pattern=args.pattern,
        label_key=args.label_key,
        flip_pull_sign=not args.no_flip_pull,
        min_wrench=args.min_wrench,
    )
    print(
        f"loaded {len(ds)} samples"
        f" (dropped {ds.n_dropped_low_wrench} below min_wrench={args.min_wrench} N,"
        f" flipped {ds.n_flipped} pull-phase rotation labels)"
    )
    train_ds, val_ds = ds.split(TRAIN_RATIO, SEED)

    train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=len(val_ds))

    # LQR gain: rotation_delta(3) = K @ x(15)
    # x = [ayal_wrench_tcp(6), uri_wrench_tcp(6), rel_distance(1),
    #      ayal_manipulability(1), uri_manipulability(1)]
    model = nn.Linear(15, 3, bias=False)
    optimizer = torch.optim.Adam(model.parameters(), lr=LR)
    loss_fn = nn.MSELoss()

    best_val_loss = float("inf")
    best_K = None

    for epoch in range(1, EPOCHS + 1):
        model.train()
        for x, y in train_loader:
            optimizer.zero_grad()
            loss_fn(model(x), y).backward()
            optimizer.step()

        model.eval()
        with torch.no_grad():
            x_val, y_val = next(iter(val_loader))
            val_loss = loss_fn(model(x_val), y_val).item()

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            best_K = model.weight.detach().clone()

        if epoch % 50 == 0:
            print(f"epoch {epoch:4d}  val_mse={val_loss:.6f}")

    out_path = os.path.join(OUT_DIR, "lqr_gain_K.pt")
    torch.save(best_K, out_path)
    print(f"\nBest val MSE: {best_val_loss:.6f}")
    print(f"K matrix saved to {out_path}")
    print(f"\nK =\n{best_K}")

def train_lqr(lqr_type="6x6"):
    if lqr_type == "6x6":
        train_lqr_6x6()
    elif lqr_type == "15x3":
        train_lqr_15x3()
    else:
        raise ValueError(f"Unsupported LQR type: {lqr_type}")

if __name__ == "__main__":
    train_lqr("15x3")