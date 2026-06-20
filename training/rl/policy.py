"""
MLP policy network for the MFE driverless car.

Architecture:
  Linear(42 → 256) → ReLU
  Linear(256 → 256) → ReLU
  Linear(256 → 128) → ReLU
  Linear(128 → 2)   → Tanh

Output is in [-1, 1]² matching the action space:
  out[0]: steering_norm
  out[1]: throttle_brake_norm
"""

import os
import torch
import torch.nn as nn


class DrivingPolicy(nn.Module):
    """MLP policy for behavioural-cloning and PPO fine-tuning."""

    def __init__(self, obs_dim: int = 42, act_dim: int = 2, hidden=(256, 256, 128)):
        super().__init__()

        layers = []
        in_dim = obs_dim
        for h in hidden:
            layers.append(nn.Linear(in_dim, h))
            layers.append(nn.ReLU())
            in_dim = h
        layers.append(nn.Linear(in_dim, act_dim))
        layers.append(nn.Tanh())

        self.net = nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)

    def save(self, path: str) -> None:
        """Save model weights to *path*."""
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        torch.save(self.state_dict(), path)
        print(f'Policy saved to {path}')

    @classmethod
    def load(cls, path: str, device: str = 'cpu') -> 'DrivingPolicy':
        """Load and return a DrivingPolicy from a saved state-dict file."""
        path = os.path.expanduser(path)
        model = cls()
        model.load_state_dict(torch.load(path, map_location=device))
        model.eval()
        return model
