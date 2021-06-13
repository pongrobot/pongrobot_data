% read in data
rpm_data = csvread('data/rpm_data.csv', 1, 0);
rpm_cmd = rpm_data(:,1);
rpm_measured = rpm_data(:,2);

%perform regression
p = polyfit(rpm_cmd, rpm_measured, 1);
rpm = polyval(p, rpm_cmd);

%plot results
hold on
title("RPM Calibration")
xlabel("RPM Command")
ylabel("RPM Measured")
scatter(rpm_cmd, rpm_measured)
plot(rpm_cmd, rpm)

% generate outputs
b=  p(2)/p(1);
m = -1.0/p(1);

fprintf("REGRESSION RESSULTS:\n");
fprintf("RPM_CMD = RPM_TARGET * %f + %f\n", m , b); 
fprintf("Slope (m): %f\n", m);
fprintf("Offset (b): %f\n", b);

