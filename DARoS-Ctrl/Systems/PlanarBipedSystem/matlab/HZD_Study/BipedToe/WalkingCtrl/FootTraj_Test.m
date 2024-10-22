clc
clear all
close all
%%
traj = FootTrajGen;

ini_time = 0;
swing_duration = 0.3;
pos_ini = [-0.05;0];
pos_fin = [0.1; -0.01];

traj.genTrajectory(pos_ini, pos_fin, ini_time, swing_duration);

t = (ini_time:0.01:ini_time+swing_duration);
len = length(t);
pos = zeros(2,len);
vel = zeros(2,len);
acc = zeros(2,len);

for i = 1:len
    [pos(:,i), vel(:,i), acc(:,i)] = traj.getSplinePt(t(i));
end

%%
figure
plot(pos(1,:), pos(2,:))
xlabel('x');  ylabel('y')
axis equal