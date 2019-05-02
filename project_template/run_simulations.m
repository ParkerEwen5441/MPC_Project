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
[T, p] = simulate_truck(T0_1, @controller_lqr, scen1);