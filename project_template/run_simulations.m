% Init
clear all
close all
addpath(genpath(cd));
load('system/parameters_scenarios.mat');

%% E.g. execute simulation with LQR
% clear persisten variables of function controller_lqr
clear controller_lqr; 
% execute simulation starting from T0_1 using lqr controller with scenario 1
T0_1 = [-17; 1.25; 6.1];
T0_2 = [-21; 0.15; 1.6];

% LQR Controller Simulation
% [T, p] = simulate_truck(T0_1, @controller_lqr, scen1);
% [T, p] = simulate_truck(T0_2, @controller_lqr, scen1);

% MPC Controller #1 Simulation
% [T, p] = simulate_truck(T0_1, @controller_mpc_1, scen1);
% [T, p] = simulate_truck(T0_2, @controller_mpc_1, scen1);

% MPC Controller #2 Simulation
% [T, p] = simulate_truck(T0_1, @controller_mpc_2, scen1);
% [T, p] = simulate_truck(T0_2, @controller_mpc_2, scen1);

% MPC Controller #3 Simulation
% [T, p] =  simulate_truck(T0_1, @controller_mpc_3, scen1);
[T, p] =  simulate_truck(T0_2, @controller_mpc_3, scen1);
