# MFE RL Training — Behavioural Cloning → PPO

End-to-end pipeline to train a neural network controller for the MFE Formula
Student driverless car using behavioural cloning (BC) from the pure pursuit
expert, followed by PPO reinforcement learning fine-tuning in Gazebo.

---

## 1. Collect Expert Demonstrations

Launch the simulator and pure pursuit controller in the normal way, then in a
separate terminal run the demo collector:

```bash
python3 training/rl/collect_demos.py
```

The node subscribes to `/planning/cones`, `/ground_truth/state`, and
`/control/command`, records (observation, action) pairs whenever pure pursuit
publishes a command, and saves them to `~/mfe_models/rl/demos.npz` every 30 s
(and on Ctrl+C).

Aim for at least 10 000 samples (a few laps). Watch the stats printout — it
shows the sample count every 5 s.

---

## 2. Train Behavioural Cloning

```bash
python3 training/rl/train_bc.py
```

- Loads `~/mfe_models/rl/demos.npz`
- 80 / 20 train / validation split
- 50 epochs, batch size 256, Adam lr=1e-3, MSE loss
- Saves the best-validation-loss checkpoint to `~/mfe_models/rl/bc_policy.pt`
- Saves a loss-curve plot to `~/mfe_models/rl/bc_loss.png`

---

## 3. Fine-tune with PPO

Ensure Gazebo is running with the `/mfe_rl/reset_episode` service available,
then:

```bash
python3 training/rl/train_ppo.py
```

- Creates a `stable-baselines3` PPO agent with the same hidden architecture
  `[256, 256, 128]`
- Transfers BC weights to the SB3 actor (see note below)
- Runs 500 000 environment steps
- Saves the final policy to `~/mfe_models/rl/ppo_policy.zip`
- TensorBoard logs go to `~/mfe_models/rl/tb_logs/`

View training progress:
```bash
tensorboard --logdir ~/mfe_models/rl/tb_logs/
```

---

## 4. Deploy the Trained Policy

```bash
ros2 run mfe_control rl_policy_node
```

Parameters (pass with `--ros-args -p key:=value`):

| Parameter     | Default                           | Description                            |
|---------------|-----------------------------------|----------------------------------------|
| `policy_path` | `~/mfe_models/rl/bc_policy.pt`   | Path to BC or PPO exported `.pt` file  |
| `use_ppo`     | `false`                           | Reserved for future SB3 loading        |
| `pose_topic`  | `/ekf/output`                     | nav_msgs/Odometry pose source          |

Example — deploy BC policy on `/ekf/output`:
```bash
ros2 run mfe_control rl_policy_node \
    --ros-args -p policy_path:=/home/neil/mfe_models/rl/bc_policy.pt
```

---

## 5. BC → SB3 Weight Transfer — Known Limitation

`DrivingPolicy` (this repo) and the SB3 `MlpPolicy` actor share the same
hidden-layer architecture (`Linear 42→256→256→128`) but differ at the output:

- **DrivingPolicy** ends with `Linear(128, 2) → Tanh`
- **SB3 MlpPolicy** uses `Linear(128, 2)` feeding a diagonal Gaussian
  distribution (no Tanh); a separate `log_std` parameter is also present

`transfer_bc_to_sb3()` in `train_ppo.py` copies the three hidden-layer
weight/bias pairs and deliberately skips the output layer to avoid the
activation mismatch. The PPO actor therefore starts with a good feature
representation but learns its own output mapping from the RL signal.

If you change the hidden architecture in `DrivingPolicy`, update the
`mapping` dict in `transfer_bc_to_sb3()` accordingly.
