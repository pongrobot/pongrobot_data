% pulling a lot of this from this paper
% 251992166_Dynamic_model_based_ball_trajectory_prediction_for_a_robot_ping
% This function will calculate the error between the cup and where the ball
% goes. The other script traj_test will try to minimize the error using a
% non-linear solver.
% v0 = initial velocity (m/s)
% p_c = cup position in launcher frame (m) [x;y;z]
% t = time points to solve for. (s)
% error = squared error planar distance between ball and cup at z=z_c
%% Ben Martell June 6 2021

function [error,p] = calcTarget(v0,p_c,t)
    
% Ping pong ball stuff
D = 0.04; % [m] diameter
r = D/2; % [m] radius
m = 0.0027; % [kg] mass
g = 9.8015; % [m/s^2]
rho_a = 1.29; % [kg/m^3] density of air
C_d=.5; % Coefficient of Drag, typically .4-.6

k_d=C_d*rho_a*pi*D^2/(8*m); % Constant for drag

% initialize variables
dt=t(2)-t(1);
p=zeros(3,length(t));
v=zeros(3,length(t));
v_mag=zeros(1,length(t));

p(:,1)=[0;0;0];
v(:,1)=[v0*cosd(30);0;v0*sind(30)];

for i=2:length(t)
    p(:,i)=p(:,i-1)+v(:,i-1)*dt;
    
    k=[-k_d*v_mag(i-1) 0 0;
        0 -k_d*v_mag(i-1) 0;
        0 0 -k_d*v_mag(i-1)]; %  Does not account for Magnus (spin)

    v(:,i)=v(:,i-1)+k*v(:,i-1)*dt+[0;0;-g]*dt;
    v_mag(i)=sqrt(sum(v(:,i).^2));
    
end

in=knnsearch(p(3,:)',p_c(3)); %Find the index of the position closest to that 
% which crosses the cup plane, to compare to p_c

error=sum((p(:,in)-p_c).^2);
end
