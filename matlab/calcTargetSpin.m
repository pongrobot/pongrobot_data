% pulling a lot of this from this paper
% 251992166_Dynamic_model_based_ball_trajectory_prediction_for_a_robot_ping
% v0 = initial velocity (m/s)
% p_c = cup position in launcher frame (m) [x;y;z]
% spin = initial spin in rpm
% t = time points to solve for. (s)
% error = squared error planar distance between ball and cup at z=z_c
%% Ben Martell June 21 2021

function [error,p] = calcTargetSpin(v0,p_c,spin,t,theta)

spinRad=spin*2*pi/60; % [rad/s]
w=[-cos(theta)*spinRad;0;sin(theta)*spinRad];    

% Ping pong ball stuff
D = 0.04; % [m] diameter
r = D/2; % [m] radius
m = 0.0027; % [kg] mass
g = 9.8015; % [m/s^2]
rho_a = 1.29; % [kg/m^3] density of air
I = 2*m*r^2/3; % moment of inertia
C_d=.5; % Coefficient of Drag, typically .4-.6
C_m=1; % Coefficient of Magnus, typically 1

k_d=C_d*rho_a*pi*D^2/(8*m); % Constant for drag
k_m=C_m*rho_a*pi*D^3/(8*m); % Constant for Spin 
dt=t(2)-t(1);
% t=0:dt:3;
p=zeros(3,length(t));
v=zeros(3,length(t));
v_mag=zeros(1,length(t));

p(:,1)=[0;0;0];
v(:,1)=[v0*cosd(30);0;v0*sind(30)];

for i=2:length(t)
    p(:,i)=p(:,i-1)+v(:,i-1)*dt;
    
%     k=[-k_d*v_mag(i-1) 0 0;
%         0 -k_d*v_mag(i-1) 0;
%         0 0 -k_d*v_mag(i-1)]; %  Does not account for Magnus
   k=[-k_d*v_mag(i-1) -k_m*w(3) k_m*w(2);
       k_m*w(3) -k_d*v_mag(i-1) -k_m*w(1);
       -k_m*w(2) k_m*w(1) -k_d*v_mag(i-1)]; %  Does account for Magnus
    
    v(:,i)=v(:,i-1)+k*v(:,i-1)*dt+[0;0;-g]*dt;
    v_mag(i)=sqrt(sum(v(:,i).^2));
    
end

in=knnsearch(p(3,:)',p_c(3)); %Find the index of the position closest to that which crosses the cup plane
%p_final=p(:,in); 

error=sum((p(:,in)-p_c).^2);
end
