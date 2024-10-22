restoredefaultpath

clc
clear all
close all

addpath('./../RobotModel')
addpath('./MPC')
%%
Mass = 29; %(kg)
Inertia = 0.7; % (kg*m^2)

z0 = [0, 0.75, -0.5, 0, 0, 0].'; % state: [x, y, theta, dx, dy, dth]

% contact position: [r_heel, r_toe_back, r_toe_tip, ...]
r_foot = [-0.5; 0];
l_foot = [0.5; 0];
cp = [r_foot , l_foot];

dt = 0.01;
tf = 1.10;

dim = length(z0);
dim_q = dim/2;
num_step = floor(tf/dt);
tspan = linspace(0, tf, num_step); 

z_out = zeros(dim,num_step);
z_out(:,1) = z0;

mpc = WalkingManager(0.05, Mass, Inertia);
x_des = z0 * ones(1,mpc.Nstep+1);

%% Simulation 
for i=1:num_step-1
    state = z_out(:,i);
    curr_time = dt * i;
    
    foot_pos = [  0.1, -0.1;
                    0,    0];
    [F_list, OptTrj] = mpc.runWalking(state, x_des, foot_pos, curr_time);

    dz = fn_dynamics_SRB(z_out(:,i), F_list{1}, foot_pos, Mass, Inertia);
    z_out(dim/2+1:dim,i+1) = z_out(dim/2+1:dim,i) + dz*dt;
    z_out(1:dim/2,i+1) = z_out(1:dim/2,i) + z_out(dim/2+1:dim,i+1)*dt;
end

%%
z_out

figure
for i = 1:6
    subplot(3,2,i)
    hold on
    plot(z_out(i,:))
    plot(x_des(i,:))
end
%% Functions 
function dz = fn_dynamics_SRB(z, forces, cp_pos, M, I)
    cp_offset = [-0.04; 0.015; 0.055 ];

    force_sum = zeros(2,1);
    tau_sum = 0;
    for i=1:3
        tau_sum = tau_sum + ...
            fn_cross2(cp_pos(:,1) + [cp_offset(i);0] - z(1:2), forces(:,i));
        tau_sum = tau_sum + ...
            fn_cross2(cp_pos(:,1) + [cp_offset(i);0] - z(1:2), forces(:,i+3));
        
        force_sum = force_sum + forces(:,i);
        force_sum = force_sum + forces(:,i+3);
    end
    dz = zeros(3,1);
    dz(1) = force_sum(1)/M;
    dz(2) = force_sum(2)/M;
    dz(3) = tau_sum/I;
end

function sol = fn_cross2(a, b)
    % a, b in R^2
    sol = a(1) * b(2) - a(2) * b(1);
end

