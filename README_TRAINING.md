# Robodog - train a command-following dog (MuJoCo + PPO), v2

## 1. Install (Linux, once)
    python3 -m venv ~/dogenv && source ~/dogenv/bin/activate
    pip install torch --index-url https://download.pytorch.org/whl/cpu
    pip install mujoco numpy gymnasium stable-baselines3
(later, in a new terminal:  source ~/dogenv/bin/activate)

## 2. Choose WHAT the dog trains on
Edit the table `TASKS` at the top of `dog_env.py`:  "forward": (weight, mode, vx, vy, wz)
 weight 0 = never trained. vx forward/back m/s, vy sideways m/s, wz turn rad/s (+ = left).
 mode 0 walk/stand, 1 sit (experimental, off), 2 lie down. Floor = infinite flat plane.

## 3. Train
    python train.py --name dog_v1 --steps 5000000 --envs 8
 Ctrl+C saves. Output in models/<name>/ : final.zip, vecnormalize.pkl, policy.npz, ckpt_*.zip
 Watch ep_len_mean (toward 1000 = not falling).

## 4. Check and watch
    python run_policy.py models/dog_v1/policy.npz --eval    # speeds, foot slip, feet lifted, falls
    python run_policy.py models/dog_v1/policy.npz           # window; type f b l r s lie + Enter in the terminal
    python run_policy.py models/dog_v1/policy.npz --demo    # automatic sequence
 Works with policy.npz or final.zip (any existing model of THIS version).

## 5. Export / continue
 policy.npz = the exported model (numpy only). Continue training:
    python train.py --name dog_v2 --resume models/dog_v1/final.zip --steps 5000000

## 6. Against "skating" / leg vibration (v2 reward)
 The dog gets a gait clock (sin/cos, 2.5 Hz) and is rewarded for a real trot: diagonal feet touch and leave the
 ground in turn, swinging feet are lifted ~3.5 cm. Sliding feet, jerky/vibrating leg commands and fast joint
 motion are penalised, and actions are low-pass filtered. Tune: REWARD, GAIT_PERIOD, STANCE_DUTY, SWING_HEIGHT.
 --eval prints "foot slip" (should be ~0) and "feet lifted" (should be high when walking, ~0 standing).
 NOTE: models trained with v1 (48 inputs) do not work with v2 (50 inputs) - train from scratch.

## 7. Add a new gesture
 Add a reference pose to POSES (test that it stands still in MuJoCo first), a row in TASKS with a new mode,
 grow the mode one-hot in the observation, and give it a reward in DogEnv.step.
