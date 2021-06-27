clear all; close all; clc;

% Target Pose
x = 10;
y = 5;
z = 0;

theta = deg2rad(35);
r = sqrt(x^2 + y^2);
yaw = atan2(y, x);

t_c = sqrt( ( 2.0 * ( r * tan(theta) - z  ) ) / 9.81 ) 
v = r / (cos(theta) * t_c);

% Simulate shot
t = [0:0.01:t_c];
r_t = v * cos(theta) * t;
z_t = v * sin(theta) * t - 1/2 * 9.81 * t.^2;
x_t = r_t * cos(yaw)
y_t = r_t * sin(yaw)

% Caluclate last point on trajectory
r_tc = v * cos(theta) * t_c ;
z_tc = v * sin(theta) * t_c - 1/2 * 9.81 * t_c^2;
x_tc = r_tc * cos(yaw)
y_tc = r_tc * sin(yaw)

% plot in 3D
plot3(0, 0, 0, 'b*','MarkerSize',20)
hold on
grid on
plot3(x_t, y_t, z_t);
plot3(x, y, z, 'g*', 'MarkerSize',20)

% plot in 2D
figure
plot(r_t, z_t);
hold on
grid on
plot(r_tc, z_tc, 'r*', 'MarkerSize',20)
plot(r, z, 'g*', 'MarkerSize',20)