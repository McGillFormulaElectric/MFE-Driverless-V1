"""
PPO fine-tuning using stable-baselines3.

Loads the behavioural-cloning checkpoint and fine-tunes in the Gazebo
simulation using Proximal Policy Optimisation.

Usage:
    python3 training/rl/train_ppo.py

Outputs:
    ~/mfe_models/rl/ppo_policy.zip  — SB3 PPO checkpoint
    ~/mfe_models/rl/tb_logs/        — TensorBoard logs
"""

import os
import sys

import torch as th

# Allow running from repo root or from training/rl/
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from policy import DrivingPolicy
from mfe_env import MFEDrivingEnv

try:
    from stable_baselines3 import PPO
except ImportError as e:
    raise ImportError(
        'stable-baselines3 is required for PPO training.\n'
        'Install with: pip install stable-baselines3'
    ) from e

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
BC_PATH   = os.path.expanduser('~/mfe_models/rl/bc_policy.pt')
PPO_PATH  = os.path.expanduser('~/mfe_models/rl/ppo_policy')
TB_LOGS   = os.path.expanduser('~/mfe_models/rl/tb_logs/')

# ---------------------------------------------------------------------------
# BC → SB3 weight transfer helper
# ---------------------------------------------------------------------------

def transfer_bc_to_sb3(bc_policy: DrivingPolicy, sb3_model: PPO) -> None:
    """
    Copy weights from the BC DrivingPolicy into the SB3 PPO actor network.

    Architecture note / known mismatch
    -----------------------------------
    DrivingPolicy (this repo):
        net.0  Linear(42, 256)
        net.2  Linear(256, 256)
        net.4  Linear(256, 128)
        net.6  Linear(128, 2)   + Tanh

    SB3 MlpPolicy with net_arch=[256, 256, 128]:
        mlp_extractor.policy_net.0  Linear(42, 256)
        mlp_extractor.policy_net.2  Linear(256, 256)
        mlp_extractor.policy_net.4  Linear(256, 128)
        action_net                  Linear(128, 2)

    The output activation differs (SB3 uses a Gaussian dist on top of
    action_net, not Tanh), so only shared hidden layers are copied.

    TODO: Verify index alignment when SB3 or this policy changes.
    """
    bc_sd = bc_policy.state_dict()
    sb3_sd = sb3_model.policy.state_dict()

    # Mapping: (bc key) → (sb3 key)
    # Indices in bc net: 0=Linear, 1=ReLU, 2=Linear, 3=ReLU, 4=Linear, 5=ReLU, 6=Linear, 7=Tanh
    mapping = {
        'net.0.weight': 'mlp_extractor.policy_net.0.weight',
        'net.0.bias':   'mlp_extractor.policy_net.0.bias',
        'net.2.weight': 'mlp_extractor.policy_net.2.weight',
        'net.2.bias':   'mlp_extractor.policy_net.2.bias',
        'net.4.weight': 'mlp_extractor.policy_net.4.weight',
        'net.4.bias':   'mlp_extractor.policy_net.4.bias',
        # NOTE: net.6 (output layer) is NOT copied — SB3 action_net feeds a
        # Normal distribution; BC output has Tanh baked in, shapes may differ.
    }

    copied = 0
    for bc_key, sb3_key in mapping.items():
        if bc_key in bc_sd and sb3_key in sb3_sd:
            if bc_sd[bc_key].shape == sb3_sd[sb3_key].shape:
                sb3_sd[sb3_key] = bc_sd[bc_key].clone()
                copied += 1
            else:
                print(
                    f'[transfer] Shape mismatch {bc_key}: '
                    f'{bc_sd[bc_key].shape} vs {sb3_sd[sb3_key].shape} — skipped.'
                )
        else:
            print(f'[transfer] Key not found: bc={bc_key!r} or sb3={sb3_key!r} — skipped.')

    sb3_model.policy.load_state_dict(sb3_sd)
    print(f'[transfer] Copied {copied}/{len(mapping)} layer pairs from BC to SB3.')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    os.makedirs(TB_LOGS, exist_ok=True)
    os.makedirs(os.path.dirname(PPO_PATH) or '.', exist_ok=True)

    print('Initialising MFE Driving Environment...')
    env = MFEDrivingEnv()

    print('Creating PPO model...')
    model = PPO(
        'MlpPolicy',
        env,
        verbose=1,
        policy_kwargs=dict(net_arch=[256, 256, 128]),
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        learning_rate=3e-4,
        ent_coef=0.01,
        tensorboard_log=TB_LOGS,
    )

    # --- Load BC weights into SB3 actor ---
    if os.path.exists(BC_PATH):
        print(f'Loading BC policy from {BC_PATH}...')
        bc_policy = DrivingPolicy.load(BC_PATH)
        with th.no_grad():
            # manual weight transfer — see transfer_bc_to_sb3 docstring for
            # details on the architectural mismatch between DrivingPolicy and
            # the SB3 MlpPolicy actor.  Output layer is intentionally skipped.
            transfer_bc_to_sb3(bc_policy, model)
        print('BC weights transferred to SB3 PPO actor.')
    else:
        print(
            f'Warning: BC policy not found at {BC_PATH}. '
            'Training PPO from scratch (no BC initialisation).'
        )

    # --- Fine-tune with PPO ---
    print(f'Starting PPO training (500 000 steps)...')
    model.learn(
        total_timesteps=500_000,
        tb_log_name='ppo_mfe',
    )

    model.save(PPO_PATH)
    print(f'PPO policy saved to {PPO_PATH}.zip')

    env.close()


if __name__ == '__main__':
    main()
