"""
PPO training for MFE driverless — supports parallel Gazebo instances.

Usage:
    # Single env, from scratch
    python3 training/rl/train_ppo.py

    # 4 parallel envs, from scratch, 2M steps
    python3 training/rl/train_ppo.py --n-envs 4 --timesteps 2000000

    # With BC warm-start
    python3 training/rl/train_ppo.py --bc

    # Resume from last checkpoint
    python3 training/rl/train_ppo.py --resume

Monitor training live:
    tensorboard --logdir ~/mfe_models/rl/tb_logs/
    # Then open http://localhost:6006 in browser.
    # Key metrics:
    #   rollout/ep_rew_mean  — average episode reward (should increase)
    #   rollout/ep_len_mean  — average episode length (should increase as car drives longer)
    #   train/entropy_loss   — policy entropy (higher = more exploration)

Prerequisites:
    - N Gazebo sims running, one per env:
        bash scripts/start_rl_envs.sh <N>
    - Install deps:
        pip install stable-baselines3 tensorboard
"""

import argparse
import os
import sys

import torch as th

sys.path.insert(0, os.path.dirname(__file__))
from mfe_env import MFEDrivingEnv
from policy import DrivingPolicy

try:
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
    from stable_baselines3.common.callbacks import (
        EvalCallback, CheckpointCallback, CallbackList
    )
    from stable_baselines3.common.monitor import Monitor
except ImportError as e:
    raise ImportError('pip install stable-baselines3 tensorboard') from e

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
BC_PATH          = os.path.expanduser('~/mfe_models/rl/bc_policy.pt')
PPO_PATH         = os.path.expanduser('~/mfe_models/rl/ppo_policy')
CHECKPOINT_DIR   = os.path.expanduser('~/mfe_models/rl/checkpoints/')
TB_LOGS          = os.path.expanduser('~/mfe_models/rl/tb_logs/')
EVAL_LOG_DIR     = os.path.expanduser('~/mfe_models/rl/eval_logs/')


# ---------------------------------------------------------------------------
# BC → SB3 weight transfer
# ---------------------------------------------------------------------------

def transfer_bc_to_sb3(bc_path: str, sb3_model: PPO) -> None:
    """Copy BC hidden-layer weights into the SB3 PPO actor network."""
    if not os.path.exists(bc_path):
        print(f'[transfer] BC weights not found at {bc_path} — skipping.')
        return

    bc = DrivingPolicy.load(bc_path)
    bc_sd  = bc.state_dict()
    sb3_sd = sb3_model.policy.state_dict()

    mapping = {
        'net.0.weight': 'mlp_extractor.policy_net.0.weight',
        'net.0.bias':   'mlp_extractor.policy_net.0.bias',
        'net.2.weight': 'mlp_extractor.policy_net.2.weight',
        'net.2.bias':   'mlp_extractor.policy_net.2.bias',
        'net.4.weight': 'mlp_extractor.policy_net.4.weight',
        'net.4.bias':   'mlp_extractor.policy_net.4.bias',
        # Output layer skipped: SB3 uses Gaussian dist, BC uses Tanh
    }

    copied = 0
    for bc_key, sb3_key in mapping.items():
        if bc_key in bc_sd and sb3_key in sb3_sd:
            if bc_sd[bc_key].shape == sb3_sd[sb3_key].shape:
                sb3_sd[sb3_key] = bc_sd[bc_key].clone()
                copied += 1
    sb3_model.policy.load_state_dict(sb3_sd)
    print(f'[transfer] Copied {copied}/{len(mapping)} layers from BC → SB3.')


# ---------------------------------------------------------------------------
# Env factory (each rank → separate ROS_DOMAIN_ID + Gazebo instance)
# ---------------------------------------------------------------------------

def make_env(rank: int):
    """Return a callable that creates a monitored env for the given rank."""
    def _init():
        env = MFEDrivingEnv(rank=rank)
        env = Monitor(env, os.path.join(EVAL_LOG_DIR, f'rank_{rank}'))
        return env
    return _init


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--n-envs',    type=int,   default=1,
                        help='Number of parallel Gazebo environments (default: 1)')
    parser.add_argument('--timesteps', type=int,   default=500_000,
                        help='Total PPO training timesteps (default: 500000)')
    parser.add_argument('--bc',        action='store_true',
                        help='Warm-start from BC weights (requires bc_policy.pt)')
    parser.add_argument('--resume',    action='store_true',
                        help='Resume from last PPO checkpoint in checkpoints/')
    parser.add_argument('--eval-freq', type=int,   default=10_000,
                        help='Evaluate policy every N steps (default: 10000)')
    args = parser.parse_args()

    for d in (CHECKPOINT_DIR, TB_LOGS, EVAL_LOG_DIR):
        os.makedirs(d, exist_ok=True)

    # --- Build vectorised environment ---
    print(f'[train] Launching {args.n_envs} environment(s)...')
    if args.n_envs == 1:
        # DummyVecEnv runs in the same process — simpler for debugging
        vec_env = DummyVecEnv([make_env(0)])
    else:
        # SubprocVecEnv forks N processes; each sets its own ROS_DOMAIN_ID
        vec_env = SubprocVecEnv([make_env(i) for i in range(args.n_envs)],
                                start_method='fork')

    # Separate single env for evaluation (rank = n_envs so it gets its own domain)
    eval_env = DummyVecEnv([make_env(args.n_envs)])

    # --- Build or resume model ---
    last_ckpt = _find_latest_checkpoint(CHECKPOINT_DIR)

    if args.resume and last_ckpt:
        print(f'[train] Resuming from checkpoint: {last_ckpt}')
        model = PPO.load(last_ckpt, env=vec_env, tensorboard_log=TB_LOGS)
    else:
        print('[train] Creating new PPO model...')
        model = PPO(
            'MlpPolicy',
            vec_env,
            verbose=1,
            policy_kwargs=dict(net_arch=[256, 256, 128]),
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            learning_rate=3e-4,
            ent_coef=0.01,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            tensorboard_log=TB_LOGS,
        )

        if args.bc:
            print(f'[train] Loading BC weights from {BC_PATH}...')
            with th.no_grad():
                transfer_bc_to_sb3(BC_PATH, model)

    # --- Callbacks ---
    # EvalCallback: runs the current policy on eval_env every eval_freq steps,
    # saves the best policy, and logs mean reward to TensorBoard automatically.
    eval_cb = EvalCallback(
        eval_env,
        best_model_save_path=os.path.join(CHECKPOINT_DIR, 'best/'),
        log_path=EVAL_LOG_DIR,
        eval_freq=max(args.eval_freq // args.n_envs, 1),
        n_eval_episodes=3,
        deterministic=True,
        verbose=1,
    )

    # CheckpointCallback: saves a .zip every 50k steps so training can be resumed
    checkpoint_cb = CheckpointCallback(
        save_freq=max(50_000 // args.n_envs, 1),
        save_path=CHECKPOINT_DIR,
        name_prefix='ppo_mfe',
        verbose=1,
    )

    callbacks = CallbackList([eval_cb, checkpoint_cb])

    # --- Train ---
    print(f'[train] Starting PPO — {args.timesteps} steps across {args.n_envs} env(s).')
    print(f'[train] Monitor live: tensorboard --logdir {TB_LOGS}')
    model.learn(
        total_timesteps=args.timesteps,
        callback=callbacks,
        tb_log_name='ppo_mfe',
        reset_num_timesteps=not args.resume,
    )

    model.save(PPO_PATH)
    print(f'[train] Final policy saved to {PPO_PATH}.zip')

    vec_env.close()
    eval_env.close()


def _find_latest_checkpoint(ckpt_dir: str) -> str | None:
    """Return path to the most recent .zip checkpoint, or None."""
    if not os.path.isdir(ckpt_dir):
        return None
    zips = [f for f in os.listdir(ckpt_dir) if f.endswith('.zip') and 'ppo_mfe' in f]
    if not zips:
        return None
    zips.sort()  # filenames include step count, so lexicographic sort works
    return os.path.join(ckpt_dir, zips[-1])


if __name__ == '__main__':
    main()
