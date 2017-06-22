clear;
close all;
clc;

addpath('data');
addpath('path_planner\RRT-connectstart');
addpath('Trajectory\cubicBezier\cubicBezier')

global robot;
global sim;
global base;
global move_base;
global count;
global steps;
global change;
global ph;

ph = 'ds';

change = 1;
count=0;
move_base = 0;
base = 'rleg';


sim = 1; %0 for OFF and 1 for ON
robot = make_model();
if base == 'rleg'
    %Names and Ids for leg base
    r_arm = 16;
    l_arm = 21;
    l_leg = 28;
    from = 1;
    adj = [13,18];
    robot.parts(7).C_Id = [8,22];
    robot.parts(11).C_Id = [12,17];
elseif base == 'hips'
    %Names and Ids for hip base
    r_arm = 10;
    l_arm = 15;
    r_leg = 22;
    l_leg = 29;
    from = 23;
    adj = [7,12];
    robot.parts(1).C_Id = [2,16,23];
    robot.parts(5).C_Id = [6,11];
end

theta = zeros(1,28);
theta(adj(1)) = -5;
theta(adj(2)) = 5;

theta(3) = -10;
theta(2) = 10;

theta(25)= 10;
theta(26)= -10;

Tr_mat = eye(4);
[com,cm,P_final,L_final,P,L]= ForwKin(theta,Tr_mat,zeros(1,28))
ForwKin_Dyn(1);

for i = 1:30
    theta(7) = i;
    [com,cm,P_final,L_final,P,L]= ForwKin(theta,Tr_mat,zeros(1,28));
    ForwKin_Dyn(1);
    ForwDyn
    robot.parts(9).com_g'
    J_tau = robot.parts(8).u*(10^-9)
    s_d
    pause(0.5)
end

for i = 30:-1:1
    theta(7) = i;
    [com,cm,P_final,L_final,P,L]= ForwKin(theta,Tr_mat,zeros(1,28));
    ForwKin_Dyn(1);
    ForwDyn
    robot.parts(9).com_g'
    J_tau = robot.parts(8).u*(10^-9)
end

load('COM_Traj.mat');
