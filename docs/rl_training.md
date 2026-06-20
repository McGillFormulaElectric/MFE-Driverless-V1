# RL Training — End-to-End Imitation + Reinforcement Learning

End-to-end driving policy for the MFE Formula Student car.
Branch: `neil-rl` | Worktree: `~/Develop/MFE-RL`

---

## Architecture

**Input (42-dim observation):**
- 10 nearest left (blue) cones in car frame: (dx, dy) × 10 → 20 dims
- 10 nearest right (yellow) cones in car frame: (dx, dy) × 10 → 20 dims
- Car speed (m/s) → 1 dim
- Yaw rate (rad/s) → 1 dim

All in car frame (rotated by -yaw). Zero-padded if fewer than 10 cones visible.

**Output (2-dim continuous action):**
- `action[0]` → `steering_norm` ∈ [-1, 1]
- `action[1]` → positive: throttle, negative: brake magnitude

**Policy network (MLP):**
```
Linear(42→256) → ReLU → Linear(256→256) → ReLU → Linear(256→128) → ReLU → Linear(128→2) → Tanh
```

---

## Reward Function

| Term | Value | Trigger |
|------|-------|---------|
| Forward progress | `+progress × 2.0` | Every step |
| Speed bonus | `+speed × 0.05` | Every step |
| Boundary penalty | `-excess × 5.0` | Continuous when past 70% of track half-width |
| Cone hit | `-100.0` + terminate | Car within 0.40 m of any cone |
| Off-track | `-30.0` + terminate | No blue AND no yellow cone within 12 m |

Cone hit detection uses `/ground_truth/track_colored` (full static map) — not the visibility-filtered `/planning/cones`. A hit is detected regardless of whether the cone is behind the car or outside the planning window.

---

## Files

| File | Purpose |
|------|---------|
| `training/rl/mfe_env.py` | `gymnasium.Env` wrapping Gazebo/ROS2 |
| `training/rl/policy.py` | `DrivingPolicy` MLP (standalone PyTorch) |
| `training/rl/collect_demos.py` | Record expert demos while pure pursuit drives |
| `training/rl/train_bc.py` | Behavioural cloning trainer |
| `training/rl/train_ppo.py` | PPO fine-tuning (supports N parallel envs) |
| `scripts/start_rl_envs.sh` | Launch N independent Gazebo instances |
| `ros2/src/mfe_control/mfe_control/rl_policy_node.py` | Deploy trained policy at 20 Hz |

---

## Workflow

### Option A — Pure RL (recommended)

No demonstrations needed. The shaped reward is dense enough to learn from scratch.
SAC converges faster than PPO for pure RL (~500k steps vs ~2M).

```bash
# 1. Launch N Gazebo sims
bash scripts/start_rl_envs.sh 4 peanut

# 2. Train
python3 training/rl/train_ppo.py --n-envs 4 --timesteps 2000000

# 3. Monitor (open http://localhost:6006)
tensorboard --logdir ~/mfe_models/rl/tb_logs/
```

### Option B — Behavioural Cloning → PPO fine-tuning

BC gives PPO a warm start — converges ~4× faster but requires expert data.

```bash
# 1. Collect expert demonstrations (pure pursuit drives, you record)
bash scripts/docker_run.sh peanut no_perception nogui 0   # sim in one terminal
python3 training/rl/collect_demos.py                       # record in another
# Ctrl+C after ~10 laps. Saves to ~/mfe_models/rl/demos.npz every 30s.

# 2. Train BC policy
python3 training/rl/train_bc.py
# 50 epochs, MSE loss, saves bc_policy.pt + loss curve plot

# 3. PPO fine-tune from BC warm-start
bash scripts/start_rl_envs.sh 4 peanut
python3 training/rl/train_ppo.py --n-envs 4 --bc --timesteps 500000
```

---

## Parallel Environments

Each parallel instance needs its own Gazebo sim. `start_rl_envs.sh` handles this:

```bash
bash scripts/start_rl_envs.sh <N> [track]
```

| Instance | ROS_DOMAIN_ID | GAZEBO_PORT |
|----------|--------------|-------------|
| 0 | 42 | 11350 |
| 1 | 43 | 11351 |
| 2 | 44 | 11352 |
| N | 42+N | 11350+N |

`MFEDrivingEnv(rank=i)` sets `ROS_DOMAIN_ID=42+i` before `rclpy.init()` so each subprocess connects to its own isolated sim. `SubprocVecEnv` forks N processes — each fork is fully isolated.

**GPU / CPU requirements per instance:**
- Gazebo (nogui): ~2 CPU cores, ~1.5 GB RAM
- ROS2 stack: ~1 CPU core, ~500 MB RAM
- 4 envs: needs ~12 CPU cores and ~8 GB RAM minimum

---

## Training Arguments

```bash
python3 training/rl/train_ppo.py \
  --n-envs 4          # parallel environments (default: 1)
  --timesteps 2000000 # total training steps (default: 500000)
  --bc                # warm-start from bc_policy.pt
  --resume            # resume from latest checkpoint in ~/mfe_models/rl/checkpoints/
  --eval-freq 10000   # evaluate every N steps (default: 10000)
```

---

## Monitoring with TensorBoard

```bash
tensorboard --logdir ~/mfe_models/rl/tb_logs/
# Open http://localhost:6006
```

**What to watch:**

| Metric | What it means | Good sign |
|--------|---------------|-----------|
| `rollout/ep_rew_mean` | Average episode reward | Trending up |
| `rollout/ep_len_mean` | Average episode length (steps) | Getting longer — car survives more |
| `eval/mean_reward` | Reward on deterministic eval runs every 10k steps | Matches or beats rollout reward |
| `train/entropy_loss` | Policy entropy | Slowly decreasing (more confident) |
| `train/policy_gradient_loss` | PPO gradient | Stable, not exploding |

If `ep_len_mean` plateaus early and reward doesn't improve, the car is hitting cones or going off-track consistently — check the boundary penalty weight or reduce max speed.

---

## Checkpoints and Resuming

Checkpoints are saved every 50k steps to `~/mfe_models/rl/checkpoints/`.
The best-ever policy (by eval reward) is saved to `checkpoints/best/best_model.zip`.

```bash
# Resume after crash or interruption
python3 training/rl/train_ppo.py --n-envs 4 --resume
```

Final policy: `~/mfe_models/rl/ppo_policy.zip`

---

## Deploying the Trained Policy

Replaces pure pursuit with the RL policy node:

```bash
# BC policy (faster to train, good baseline)
ros2 run mfe_control rl_policy_node \
  --ros-args -p policy_path:=~/mfe_models/rl/bc_policy.pt

# PPO fine-tuned policy
ros2 run mfe_control rl_policy_node \
  --ros-args -p policy_path:=~/mfe_models/rl/checkpoints/best/best_model.zip \
             -p use_ppo:=true
```

The node subscribes to `/planning/cones` and `/ekf/output` (configurable via `pose_topic`), runs the policy at 20 Hz, and publishes to `/control/command`.

---

## BC → SB3 Weight Transfer

BC and SB3's `MlpPolicy` have different output heads:
- BC: `Linear(128→2) + Tanh`
- SB3: `action_net` feeds a `Normal` distribution (Gaussian)

Only the 3 hidden layers are copied. The output layer is re-initialised randomly.
This means the first few PPO updates after transfer may be noisy — this is expected.

---

## Comparing RL vs Pure Pursuit

Run the lap validator on both and compare the CSV:

```bash
# Pure pursuit baseline (neil-fix branch)
bash scripts/docker_run.sh peanut no_perception nogui 5
# CSV: ~/mfe_logs/validation_<timestamp>.csv

# RL policy
ros2 run mfe_control rl_policy_node --ros-args -p policy_path:=~/mfe_models/rl/bc_policy.pt
# Same validator runs alongside — same CSV format, same cone hit detection
```

CSV columns: `lap, lap_time_s, avg_speed_ms, max_speed_ms, avg_deviation_m, max_deviation_m, cones_hit`
