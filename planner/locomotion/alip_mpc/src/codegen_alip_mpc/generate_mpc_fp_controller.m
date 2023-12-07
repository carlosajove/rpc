%% Foot with MPC and Linear Inverted Pendulum

%RIGHT NOW WE CAN'T USE IT SINCE I DON'T HAVE CASADI MATLAB
%HOWEVER RIGHT NOW WE ARE GOING TO USE THE OPT FILES FROM SOURCE CODE
%TWO OPTIONS: INSTALL CASADI MATLAB OR CONVERT THE CODE TO CPP SO WE CAN USE IT. I PERSONALLY PREFER THE SECOND OPTION
clear; clc; close all;
restoredefaultpath;
cur = pwd;

addpath('/home/carlos/Desktop/Austin/SeungHyeonProject/ALIP MPC/cassie_alip_mpc/external_packages/casadi-linux-matlabR2014b-v3.5.5/');


if ~exist('gen/cassie_mpc_fp_solvers','dir')
    mkdir('gen/cassie_mpc_fp_solvers')
end
addpath(genpath([cur '/gen/']));
addpath(genpath([cur '/utils/']));

% Constants
% mpc_info.params = struct(...
%     'm',            31.8852);


%% Symbolic Functions 
import casadi.*

% Slip limit
mu = SX.sym('mu');
kx = SX.sym('kx');
ky = SX.sym('ky');
zH = SX.sym('zH');

xc_slip_limit = (mu - kx)*zH / (1 + kx^2);
yc_slip_limit = (mu - ky)*zH / (1 + ky^2);

f_xc_slip_limit = Function('f_xc_slip_limit',{zH,mu,kx},{xc_slip_limit});
f_yc_slip_limit = Function('f_yc_slip_limit',{zH,mu,ky},{yc_slip_limit});

% store in struct
mpc_info.symbolics = struct(...
    'f_xc_slip_limit', f_xc_slip_limit,...
    'f_yc_slip_limit', f_yc_slip_limit);

%% Optimization Data
              

%% Formulate LIP foot placement Optimization
N_steps_ahead_list = [2, 4, 6, 8];
nx = 4;

Q = eye(nx,nx);
Q(4,4) = 10
disp("Begin Formulation of ALIP-based FP Optimization Problem...");
for i = 1:length(N_steps_ahead_list)
    nx = 4;
    N_steps_ahead = N_steps_ahead_list(i);
    mpc_info.opt = struct(...
        'N_steps_ahead',    N_steps_ahead,...   % 2 steps makes the friction constraint get invalidated
        'N_intervals',      4,...               % Number of output intervals
        'Q',                eye(nx,nx),...      % state penalty matrix
        'qpsolver',         "qrqp");            % qp solver    
    formulate_alip_mpc_fp_opt(mpc_info);
end
disp("Formulated ALIP-based FP Optimization (" + toc + " sec)");

disp("Begin Formulation of draco ALIP-based FP Optimization Problem...");
for i = 1:length(N_steps_ahead_list)
    nx = 4;
    N_steps_ahead = N_steps_ahead_list(i);
    mpc_info.opt = struct(...
        'N_steps_ahead',    N_steps_ahead,...   % 2 steps makes the friction constraint get invalidated
        'N_intervals',      4,...               % Number of output intervals
        'Q',                eye(nx,nx),...      % state penalty matrix
        'qpsolver',         "qrqp");            % qp solver    
    formulate_draco_alip_mpc_fp_opt(mpc_info);
end
disp("Formulated ALIP-based FP Optimization (" + toc + " sec)");
