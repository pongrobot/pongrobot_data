% function to plot trajectory (comparing WITH drag to NO drag)
% p_c is the position of the cup in the reference frame of launcher [r,0,z]
% yaw is the angle in rad of the launcher wrt the robot
% p_guess is position over time with no drags
% p is position over time with drag.
function plotTrajectoryDrag(p_c,yaw,p_guess,p)

%% Plotting


% plot in 3D
ind_guess=find(p_guess(3,:)<p_c(3));
p_guess=p_guess(:,1:ind_guess(1));
p_guessXYZ=[cos(yaw),0,0;sin(yaw),0,0;0,0,1]*p_guess;
ind=find(p(3,:)<p_c(3));
p=p(:,1:ind(1));

T = [cos(yaw),0,0; sin(yaw),0,0; 0,0,1]; % Transformation matrix
pXYZ=T*p;
p_cXYZ=T*p_c; %Transform to XYZ for 3D plot


figure(1)
clf
plot3(0, 0, 0, 'b*','MarkerSize',20)
hold on
grid on
plot3(p_guessXYZ(1,:), p_guessXYZ(2,:), p_guessXYZ(3,:),'r');
plot3(pXYZ(1,:), pXYZ(2,:), pXYZ(3,:));
plot3(p_cXYZ(1), p_cXYZ(2), p_cXYZ(3), 'g*', 'MarkerSize',20)



% plot in 2D
figure(2)
clf
plot(p_guess(1,:), p_guess(3,:),'r');
hold on
grid on
plot(p_c(1), p_c(3), 'g*', 'MarkerSize',20)
plot(p(1,:),p(3,:))

