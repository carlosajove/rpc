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
#import tracemalloc
import argparse

new_model = False

if __name__ == "__main__":
    #tracemalloc.start()
    if not new_model:
        parser = argparse.ArgumentParser(description='Training script for your RL model')
        parser.add_argument('--timesteps', type=str, help='Number of timesteps for training')  # Default value is set as an example
        args = parser.parse_args()

        # Extract TIMESTEPS value from command-line argument
        bash_timesteps = int(args.timesteps)



    env = DracoEnv(render=False)
    #env = VecNormalize(not_norm_env, norm_reward=False, clip_obs=50)

    n_steps_ = 256 #512
    batch_size_ = 64
    learning_rate_ = 0.0003



    ## train model
    if new_model:
        tensorboard_dir = cwd + "/ppo_rl_log/"
        model = PPO("MlpPolicy", env, verbose=1, n_steps = n_steps_, batch_size=batch_size_, tensorboard_log=tensorboard_dir, learning_rate=learning_rate_, device = "cpu") #policy_kwargs=dict(net_arch=[64,64, dict(vf=[], pi=[])]), 
        startTime = time.time()
        TIMESTEPS = 1*n_steps_
        CURR_TIMESTEP = 0

    else:
        startTime = time.time()
        CURR_TIMESTEP = bash_timesteps
        model_path = f"{model_dir}/full_obs_yaw10/NSTEPS{n_steps_}_LEARNING_RATE{learning_rate_}_TIME{CURR_TIMESTEP}"
        print("model_path", model_path)
        model = PPO.load(model_path, env=env)
        TIMESTEPS = 20*n_steps_


    #snapshot1 = tracemalloc.take_snapshot()

    while(True):
        try:
            model.learn(total_timesteps=TIMESTEPS, progress_bar=True, reset_num_timesteps=False, tb_log_name="erase2")
            endTime = time.time()
            print("Model train time: "+str(datetime.timedelta(seconds=endTime-startTime)))
            ## save the model
            CURR_TIMESTEP += TIMESTEPS
            #save_subdir = f"/fullobs_nosafe_Ly1/NSTEPS{n_steps_}_LEARNING_RATE{learning_rate_}_TIME{CURR_TIMESTEP}"
            save_subdir = f"/full_obs_yaw10/NSTEPS{n_steps_}_LEARNING_RATE{learning_rate_}_TIME{CURR_TIMESTEP}"
            save_path = model_dir + save_subdir
            print(save_path)
            model.save(save_path)
            with open('timesteps.txt', 'w') as f:
                f.write(str(CURR_TIMESTEP))
                f.flush()


            """
            snapshot2 = tracemalloc.take_snapshot()
            compare_stats_100_1 = snapshot2.compare_to(snapshot1, 'lineno')
            print("[ Top 20 differences ]")
            for stat in compare_stats_100_1[:20]:
                print(stat, ' \n')
            """

        except Exception as e:
            print(f"An error occurred during training: {e}")
            endTime = time.time()
            model = PPO.load(save_path, env = env)
