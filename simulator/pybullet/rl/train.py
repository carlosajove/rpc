import os
import sys
import numpy as np
import datetime
import time

import gymnasium as gym

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecNormalize

from env import DracoEnv
cwd = os.getcwd()
sys.path.append(cwd)

model_dir = cwd + "/rl_model/PPO"



if __name__ == "__main__":
    env = DracoEnv(render=True)
    #env = VecNormalize(not_norm_env, norm_reward=False, clip_obs=50)
    new_model = True
    
    ## train model
    if new_model:
        tensorboard_dir = cwd + "/ppo_rl_log/"
        model = PPO("MlpPolicy", env, verbose=1, n_steps = 256, batch_size=64, tensorboard_log=tensorboard_dir) #policy_kwargs=dict(net_arch=[64,64, dict(vf=[], pi=[])]), 
        startTime = time.time()
        iters = 0
        TIMESTEPS = 2*768
    else:
        startTime = time.time()
        model_path = f"{model_dir}/reduced_input/78336_"
        model = PPO.load(model_path, env=env)
        TIMESTEPS = 768
        iters = 102

    last_saved_iter = 102
    while(True):
        iters = last_saved_iter
        try:
            iters += 1
            model.learn(total_timesteps=TIMESTEPS, progress_bar=True, reset_num_timesteps=False, tb_log_name="reduced_input_v1")
            endTime = time.time()
            print("Model train time: "+str(datetime.timedelta(seconds=endTime-startTime)))
            ## save the model
            save_path = '{}/{}'.format(model_dir, f"/reduced_input/{TIMESTEPS*iters}_")
            model.save(save_path)
            last_saved_iter = iters
        except Exception as e:
            print(f"An error occurred during training: {e}")
            endTime = time.time()
            model_path = f"{model_dir}/" + f"{TIMESTEPS*last_saved_iter}"
            model = PPO.load(model_path, env = env)
