from plot_utils import *
import os

mass = 39.15342
zH = 0.69
lbound_time = 1  #5
ubound_time = 100  #8

trAlip2 = all_trajectories('Alip2Swing_trajectory.txt')
trRobotSwing = readRobotSwTr('robotSwingFootTraj.txt')
#trSw = merge_swing_traj(trSw1, trSw2)

CurrentComstate = readRobotSwTr('RobotCOM.txt')
RobotCommand = readRobotSwTr('RobotCommand.txt')
MpcComState = readRobotSwTr('MpcCOMstate.txt')

COMmpcoor = readRobotSwTr('RobotCOMmpcOri.txt')

mpc_coor_x = COMmpcoor[:, 0]
mpc_coor_y = COMmpcoor[:, 1]
mpc_coor_z = COMmpcoor[:, 2]
mpc_coor_vx = COMmpcoor[:, 3]
mpc_coor_vy = COMmpcoor[:, 4]
mpc_coor_vz = COMmpcoor[:, 5]
mpc_coor_Lst_x = COMmpcoor[:, 6]
mpc_coor_Lst_y = COMmpcoor[:, 7]
mpc_coor_Lst_z = COMmpcoor[:, 8]
mpc_coor_Lc_x = COMmpcoor[:, 9]
mpc_coor_Lc_y = COMmpcoor[:, 10]
mpc_coor_Lc_z = COMmpcoor[:, 11]
mpc_coor_L_x = COMmpcoor[:, 12]
mpc_coor_L_y = COMmpcoor[:, 13]
mpc_coor_L_z = COMmpcoor[:, 14]
alip_time = COMmpcoor[:, 15]
torso_roll = COMmpcoor[:, 16]
torso_pitch = COMmpcoor[:, 17]
torso_yaw = COMmpcoor[:, 18]

timeCOM = alip_time
time = alip_time

swingXCommand = RobotCommand[:, 0]
swingYCommand = RobotCommand[:, 1]
swingZCommand = RobotCommand[:, 2]
swingVxCommand = RobotCommand[:, 3]
swingVyCommand = RobotCommand[:, 4]
swingVzCommand = RobotCommand[:, 5]
swingAxCommand = RobotCommand[:, 6]
swingAyCommand = RobotCommand[:, 7]
swingAzCommand = RobotCommand[:, 8]
swingTimeCommand = RobotCommand[:, 9]
end_foot_commandx = RobotCommand[:, 10]
end_foot_commandy = RobotCommand[:, 11]
end_foot_commandz = RobotCommand[:, 12]
stance_leg_side = RobotCommand[:, 13]
residual_action_x = RobotCommand[:, 14]
residual_action_y = RobotCommand[:, 15]
residual_action_yaw = RobotCommand[:, 16]
mpc_sol_in_world_x = RobotCommand[:, 17]
mpc_sol_in_world_y = RobotCommand[:, 18]
mpc_sol_in_world_z = RobotCommand[:, 19]
full_action_x = RobotCommand[:, 20]
full_action_y = RobotCommand[:, 21]
full_action_yaw = RobotCommand[:, 22]
# action_time = RobotCommand[:, 23]
action_time = time

MpcxCOM = MpcComState[:, 0]
MpcyCOM = MpcComState[:, 1]
MpczCOM = MpcComState[:, 2]
MpcLxCOM = MpcComState[:, 3]
MpcLyCOM = MpcComState[:, 4]
MpcLzCOM = MpcComState[:, 5]
MpctimeCOM = MpcComState[:, 6]
MpcLx_off = MpcComState[:, 7]
MpcLy_des = MpcComState[:, 8]
MpcMu = MpcComState[:, 9]
MpcLegWidth = MpcComState[:, 10]
MpczH = MpcComState[:, 11]
MpcMassCom = MpcComState[:, 12]
MpcTs = MpcComState[:, 13]

# with open('LandTime.txt', 'r') as file:
# landingTime = [float(line.strip()) for line in file]

inter = sliceTime(time, lbound_time, ubound_time)
# landingTimes_in_range = [
# t for t in landingTime if lbound_time <= t <= ubound_time
# ]

light_grey = [0.85, 0.85, 0.85]
facecolors = [
    light_grey, 'grey', 'brown', 'red', 'orange', 'yellow', 'green', 'blue',
    'purple', 'crimson', 'white'
]
colors = [facecolors[0], facecolors[-1]]
