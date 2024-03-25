import gymnasium as gym
import numpy as np
import pybullet as p
import pybullet_utils.bullet_client as bc
import pybullet_data
import torch

from stable_baselines3 import PPO

import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)
import time, math
from collections import OrderedDict
import copy
import signal
import shutil

import cv2

from config.draco3_alip_config import SimConfig
from config.draco3_alip_config import AlipParams

from pnc_pytorch.draco3_pnc.draco3_interface import Draco3Interface
from util import pybullet_util_rl

from simulator.pybullet.rl.env import DracoEnv


if __name__ == "__main__":
    env = DracoEnv(render=True)

    from stable_baselines3.common.env_checker import check_env
    #check_env(env)

    obs, info = env.reset()
    interface = info["interface"]

    model_dir = cwd + "/rl_model"

    model_path = f"{model_dir}/fist_try.zip"
    model = PPO.load(model_path, env=env)


    while True:
        #action = torch.ones(AlipParams.N_BATCH,3)
        action, _ = model.predict(obs)
        print(action)
        obs, reward, done, trunc, info = env.step(action)
        if done: 
            obs,info = env.reset()

