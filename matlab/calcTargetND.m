% Calculating the target with no drag and no spin, basic kinematics
function [v0,p]=calcTargetND(p_c,theta,t)
g=9.8; % [m/s^2]

t_c = sqrt( ( 2.0 * ( p_c(1,:) * tan(theta) - p_c(3,:)  ) ) / 9.81 ); 
v0=p_c(1,:) / (cos(theta) * t_c); 

% get position as function of time
p=zeros(size(t));
p(1,:) = v0 * cos(theta) * t;
p(3,:) = v0 * sin(theta) * t - 1/2 * g * t.^2;
end
