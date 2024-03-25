import os

import numpy as np
import datetime
import time

import gymnasium as gym

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

from env import DracoEnv


if __name__ == "__main__":
    env = DracoEnv(render=False)

    
    #TODO: periodically save
    ## train model
    model = PPO("MlpPolicy", env, verbose=1, n_steps = 256, batch_size=64, tensorboard_log="/home/carlos/Desktop/Austin/SeungHyeonProject/PyPnc_pytorch/ppo_rl_log/") #policy_kwargs=dict(net_arch=[64,64, dict(vf=[], pi=[])]), 
    startTime = time.time()
    iters = 0
    TIMESTEPS = 768
    while(True):
        iters += 1
        model.learn(total_timesteps=TIMESTEPS, tb_log_name="second", progress_bar=True, reset_num_timesteps=False)
        endTime = time.time()
        print("Model train time: "+str(datetime.timedelta(seconds=endTime-startTime)))
        ## save the model
        save_path = '{}/{}'.format("rl_model/PPO", f"/{TIMESTEPS*iters}")
        model.save(save_path)