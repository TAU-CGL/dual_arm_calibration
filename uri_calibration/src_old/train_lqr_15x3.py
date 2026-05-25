import argparse
import os
import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from uri_calibration.src.pose_force_dataset import PoseForceDataset

# --- config ---
DEFAULT_DATA = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "../../dual_arm_peg/output")
)
OUT_DIR = os.path.join(os.path.dirname(__file__), "../logs")
EPOCHS = 2000
BATCH_SIZE = 32
LR = 1e-3
TRAIN_RATIO = 0.8
SEED = 42
# --------------

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

ds = PoseForceDataset(
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
