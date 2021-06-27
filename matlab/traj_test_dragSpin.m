% Trajectory calculation with Drag AND Spin
%% Ben Martell June 6 2021

clear all
close all

% Cup Pose
x_c = 2; % [m]
y_c = 0; % [m]
z_c = -1;% [m]
curve=2; % [deg]

theta = deg2rad(35); % hardware constraint?
yaw = atan2(y_c, x_c)+curve*pi/180; % Angle (yaw)
T = [cos(yaw),sin(yaw),0;       % Transformation matrix
     -sin(yaw),cos(yaw),0;
     0,0,1];
p_c = T*[x_c;y_c;z_c]; % define new reference frame in line with launcher

dt=.01; 
t=0:dt:3; % [s]

[v0_nDnS,p_nDnS]=calcTargetND(p_c,theta,t);

tol=0.001;


[err1,perr]=calcTarget(v0_nDnS,p_c,t); % how bad is our guess.
% iteratively solve the equation to minimize the error from calcTargetSolver
v0_nS = fsolve(@(v0)calcTargetSolver(v0,p_c,t),v0_nDnS); % solve non-linear eqn DRAG
[~,p_nS]=calcTarget(v0_nS,p_c,t); % Calculate everything based on the velocity found

spin=-5; % THis is the guess[rpm]
%err2=calcTargetSpin(v0_nDnS,p_c,spin,t); % before

scale=300; % this weights the two iC's so it can solve faster. iC=[v;spin]
options = optimoptions(@fmincon,'MaxFunctionEvaluations',2000); % gotta run it a bit longer
iC = fsolve(@(iC)calcTargetSolverSpin([iC(1);iC(2)*scale],p_c,t,theta),[v0_nDnS;spin],options); % solve non-linear eqn
v0=iC(1);
spin=iC(2)*scale;
[error,p]=calcTargetSpin(v0,p_c,spin,t,theta); % after


plotTrajectorySpin(p_c,yaw,p_nDnS,p_nS,p,perr)


spinRad=spin*2*pi/60; % [rad/s]
w_guess=[-cos(theta)*spinRad;0;sin(theta)*spinRad];

function [error] = calcTargetSolver(v0,p_c,t)
    [error,~] = calcTarget(v0,p_c,t);
end

function [error] = calcTargetSolverSpin(iC,p_c,t,theta)
    v0=iC(1);
    spin=iC(2);
    [error,~] = calcTargetSpin(v0,p_c,spin,t,theta);
end
