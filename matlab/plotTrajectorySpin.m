% function to plot trajectory (comparing WITH spin & drag to WITH drag to NO drag)
% p_c is the position of the cup in the reference frame of launcher [r,0,z]
% yaw is the angle in rad of the launcher wrt the robot
% p_nDnS is position over time with no drag and no spin
% p_nS is position over time with drag but No Spin
% p is position over time with drag AND spin
% perr is position over time with if the velocity for p_nDnS was used when
% there is drag (real life) to give an idea of the distance difference
function plotTrajectorySpin(p_c,yaw,p_nDnS,p_nS,p,perr)

%% Plotting


% plot in 3D

%Transform back to XYZ for 3D plot
endz=p_c(3);
ind=find(p_nDnS(3,:)<endz);
p_nDnSXYZ=[cos(yaw),-sin(yaw),0;sin(yaw),cos(yaw),0;0,0,1]*p_nDnS(:,1:ind(1));
ind=find(p_nS(3,:)<endz);
p_nSXYZ=[cos(yaw),-sin(yaw),0;sin(yaw),cos(yaw),0;0,0,1]*p_nS(:,1:ind(1));
ind=find(p(3,:)<endz);
pXYZ=[cos(yaw),-sin(yaw),0;sin(yaw),cos(yaw),0;0,0,1]*p(:,1:ind(1));

p_cXYZ=[cos(yaw),-sin(yaw),0;sin(yaw),cos(yaw),0;0,0,1]*p_c; 


figure(1)
clf
%plot3(0, 0, 0, 'b*','MarkerSize',20)
hold on
grid on
plot3(p_nDnSXYZ(1,:), p_nDnSXYZ(2,:), p_nDnSXYZ(3,:),'r');
plot3(p_nSXYZ(1,:), p_nSXYZ(2,:), p_nSXYZ(3,:),'b');
plot3(pXYZ(1,:), pXYZ(2,:), pXYZ(3,:),'m');
plot3(p_cXYZ(1), p_cXYZ(2), p_cXYZ(3), 'g*', 'MarkerSize',20)

legend('no Drag, no Spin','Drag, no Spin', 'Drag, Spin')

% plot in 2D
ind=find(p_nDnS(3,:)<endz);
p_nDnS=p_nDnS(:,1:ind(1));
ind=find(p_nS(3,:)<endz);
p_nS=p_nS(:,1:ind(1));
ind=find(perr(3,:)<endz);
perr=perr(:,1:ind(1));

figure(2)
clf
hold on
grid on
plot(p_nDnS(1,:), p_nDnS(3,:),'r');
plot(p_nS(1,:),p_nS(3,:),'b')
plot(perr(1,:),perr(3,:),'m')
plot(p_c(1), p_c(3), 'g*', 'MarkerSize',20)
legend('No Drag','Drag','Drag, with No Drag V_0')

