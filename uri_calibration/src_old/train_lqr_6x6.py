import os
import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from uri_calibration.src.pose_force_dataset import PoseForceDataset

# --- config ---
LOGS_DIR = os.path.join(os.path.dirname(__file__), "../logs/auto_record_v1")
# LOGS_DIR = os.path.join(os.path.dirname(__file__), "../../dual_arm_peg/output")
OUT_DIR = os.path.join(os.path.dirname(__file__), "../logs")
EPOCHS = 2000
BATCH_SIZE = 32
LR = 1e-3
TRAIN_RATIO = 0.8
SEED = 42
# --------------

torch.manual_seed(SEED)

ds = PoseForceDataset(LOGS_DIR)
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
