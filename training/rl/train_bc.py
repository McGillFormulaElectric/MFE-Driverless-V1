"""
Behavioural Cloning (BC) trainer.

Loads expert demonstrations from ~/mfe_models/rl/demos.npz and trains a
DrivingPolicy to imitate the pure pursuit controller.

Usage:
    python3 training/rl/train_bc.py

Outputs:
    ~/mfe_models/rl/bc_policy.pt   — best validation-loss checkpoint
    ~/mfe_models/rl/bc_loss.png    — training / validation loss curve
"""

import os
import sys

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset, random_split

# Allow running from repo root or from training/rl/
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from policy import DrivingPolicy

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
DEMO_PATH   = os.path.expanduser('~/mfe_models/rl/demos.npz')
MODEL_PATH  = os.path.expanduser('~/mfe_models/rl/bc_policy.pt')
LOSS_PLOT   = os.path.expanduser('~/mfe_models/rl/bc_loss.png')

# ---------------------------------------------------------------------------
# Hyperparameters
# ---------------------------------------------------------------------------
EPOCHS      = 50
BATCH_SIZE  = 256
LR          = 1e-3
TRAIN_RATIO = 0.8
DEVICE      = 'cuda' if torch.cuda.is_available() else 'cpu'


def load_dataset(path: str):
    if not os.path.exists(path):
        raise FileNotFoundError(
            f'Demo file not found: {path}\n'
            'Run collect_demos.py with pure pursuit driving first.'
        )
    data = np.load(path)
    obs = torch.tensor(data['obs'], dtype=torch.float32)
    actions = torch.tensor(data['actions'], dtype=torch.float32)
    print(f'Loaded {len(obs)} demos from {path}')
    return TensorDataset(obs, actions)


def make_loaders(dataset):
    n_train = int(len(dataset) * TRAIN_RATIO)
    n_val   = len(dataset) - n_train
    train_ds, val_ds = random_split(
        dataset, [n_train, n_val],
        generator=torch.Generator().manual_seed(42)
    )
    train_loader = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True,  num_workers=2)
    val_loader   = DataLoader(val_ds,   batch_size=BATCH_SIZE, shuffle=False, num_workers=2)
    print(f'Train: {n_train}  Val: {n_val}')
    return train_loader, val_loader


def train():
    os.makedirs(os.path.expanduser('~/mfe_models/rl'), exist_ok=True)

    dataset = load_dataset(DEMO_PATH)
    train_loader, val_loader = make_loaders(dataset)

    model = DrivingPolicy().to(DEVICE)
    optimizer = torch.optim.Adam(model.parameters(), lr=LR)
    criterion = nn.MSELoss()

    best_val_loss = float('inf')
    train_losses, val_losses = [], []

    header = f'{"Epoch":>6}  {"Train Loss":>12}  {"Val Loss":>12}  {"Best":>6}'
    print('\n' + header)
    print('-' * len(header))

    for epoch in range(1, EPOCHS + 1):
        # --- Training ---
        model.train()
        total_train = 0.0
        for obs_b, act_b in train_loader:
            obs_b, act_b = obs_b.to(DEVICE), act_b.to(DEVICE)
            optimizer.zero_grad()
            pred = model(obs_b)
            loss = criterion(pred, act_b)
            loss.backward()
            optimizer.step()
            total_train += loss.item() * len(obs_b)
        avg_train = total_train / len(train_loader.dataset)

        # --- Validation ---
        model.eval()
        total_val = 0.0
        with torch.no_grad():
            for obs_b, act_b in val_loader:
                obs_b, act_b = obs_b.to(DEVICE), act_b.to(DEVICE)
                pred = model(obs_b)
                loss = criterion(pred, act_b)
                total_val += loss.item() * len(obs_b)
        avg_val = total_val / len(val_loader.dataset)

        train_losses.append(avg_train)
        val_losses.append(avg_val)

        is_best = avg_val < best_val_loss
        if is_best:
            best_val_loss = avg_val
            model.save(MODEL_PATH)

        marker = '  *' if is_best else ''
        print(f'{epoch:>6}  {avg_train:>12.6f}  {avg_val:>12.6f}{marker}')

    print(f'\nBest val loss: {best_val_loss:.6f} — saved to {MODEL_PATH}')

    # --- Plot loss curve ---
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        epochs = list(range(1, EPOCHS + 1))
        plt.figure(figsize=(8, 5))
        plt.plot(epochs, train_losses, label='Train Loss')
        plt.plot(epochs, val_losses,   label='Val Loss')
        plt.xlabel('Epoch')
        plt.ylabel('MSE Loss')
        plt.title('Behavioural Cloning — Loss Curve')
        plt.legend()
        plt.tight_layout()
        plt.savefig(LOSS_PLOT, dpi=150)
        plt.close()
        print(f'Loss plot saved to {LOSS_PLOT}')
    except ImportError:
        print('matplotlib not available — skipping loss plot.')


if __name__ == '__main__':
    train()
