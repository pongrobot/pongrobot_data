% Script to test out the trajectory calculation WITH drag. uses non linear 
% solver fsolve and compares the results to NO drag case
%% Ben Martell June 6 2021
clear all
close all
% Cup Pose
x_c = 5; % [m]
y_c = 5; % [m]
z_c = -1;% [m]

theta = deg2rad(35); % hardware constraint [rad]
yaw = atan2(y_c, x_c); % Angle (yaw) [rad]
T = [cos(yaw),sin(yaw),0;       % Transformation matrix
     -sin(yaw),cos(yaw),0;
     0,0,1];
p_c = T*[x_c;y_c;z_c]; % define new reference frame in line with launcher

dt = .01; % [s]
t = 0:dt:3; % [s]

% Calculate trajectory with no drag
[v0_guess,p_guess]=calcTargetND(p_c,theta,t); 

% iteratively solve the equation to minimize the error from calcTargetSolver
v0 = fsolve(@(v0)calcTargetSolver(v0,p_c,t),v0_guess); % solve non-linear eqn

% Calculate everything based on the velocity found
[error,p]=calcTarget(v0,p_c,t);

% plot it
plotTrajectoryDrag(p_c,yaw,p_guess,p)


function [error] = calcTargetSolver(v0,p_c,t)
    [error,~] = calcTarget(v0,p_c,t);
end
