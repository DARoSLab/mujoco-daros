restoredefaultpath

clc
clear all
close all
%% Path
addpath('./RobotModel')
addpath('./HZD')
addpath('./WBIC')
addpath('./WalkingCtrl')
addpath('./../Utils')


%%
global z0 dt
restitution_coeff = 0.0;
friction_coeff = 0.8;   

choice_ctrl = 1; % 0: jpos still, 1: HZD, 2: WBC, 3: TVR + WBIC, 4: MPC + WBIC 


% [x; y; body_theta; q_hip_r; q_knee_r; q_ankle_r; q_toe_r; q_hip_l; q_knee_l; q_ankle_l; q_toe_l]
% z0 = [0.0; 0.75; body_th; 0.4 - body_th; -0.4 + body_th; -0.0; 0;  ...
%                           0.05 - body_th; -0.4 + body_th; 0.35; 0; 
%       0.0; 0.0; 0.0; 
%       0; 0; 0; 0;
%       0; 0; 0; 0 ];
% right leg: RED, left leg: BLUE

z0 = [ 0, 0.74, -0.05, 0.35, -0.75, 0.3, 0, 0.35, -0.6, 0.3, 0.0, ...
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0].';   
   
if(choice_ctrl == 3) 
% WBC posture
% case 1: 0.7 m/s
z0 = [ 0, 0.71, -0.05, 0.62, -0.8, 0.2, 0, 0.145, -0.75, 0.56, 0.1, ...
       0.77, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0].';

% case2: 0.4 m/s
% z0 = [ 0, 0.71, -0.05, 0.5, -0.8, 0.3, 0, 0.145, -0.75, 0.56, 0.1, ...
%        0.35, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0].';

% case 3: 0.56 m/s
% z0 = [ 0, 0.71, -0.05, 0.55, -0.75, 0.2, 0, 0.145, -0.75, 0.56, 0.1, ...
%        0.55, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0].';

% case 4: 0.778 m/s
% z0 = [ 0, 0.71, -0.05, 0.65, -0.5, -0.15, 0, 0.145, -0.75, 0.56, 0.1, ...
%        1.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0].';

      
elseif(choice_ctrl == 1)
% HZD state
% case 1: 0.7 m/s
% z0 = [ 0,    0.764, -0.02, 0.21+0.02, 0, 0, 0, -0.21+0.02, 0, 0, 0.21 ...
%        0.79, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0].';

% case 2: 
% z0 = [ 0.0000
%     0.7777
%    -0.0192
%    %%
%     0.1708
%     0.0000
%    -0.0007
%    -0.0000
%    %%
%     0.0074
%    -0.1398
%    -0.3185
%     0.4689
%     %%
%     0.765
%     0.0
%     0.0
%    -2.0012
%     1.9758
%    -1.0229
%     0.1055
%     0.3673
%    -2.8941
%     0.3617
%     1.8066];

z0 = [       0
    0.7777
   -0.0192
   %%
    0.1708
    0.0000
   -0.0007
    0.0000
    %%
   -0.0297
   -0.1285
   -0.3932
    0.5635
    %%
    0.63
    0.0
    0.3
   -2.3242
    1.4719
   -0.9260
   -0.2686
   -0.2683
   -2.6486
   -0.6226
    2.7985];
end


% z0(15:end) = zeros(8,1);
%% Perform Dynamic simulation
dt = 0.001;
tf = 1.51;
dim = length(z0);
dim_q = dim/2;
dim_act = dim_q - 3;
num_step = floor(tf/dt);
tspan = linspace(0, tf, num_step); 

z_out = zeros(dim,num_step);
z_out(:,1) = z0;

right_stance = true;
stance_switch_iter = 1;


jpos_des = z0(4:dim_q);
jvel_des = zeros(dim_act,1);

jpos_des_traj = zeros(dim_act,1);
jvel_des_traj = zeros(dim_act,1);
tau_traj = zeros(dim_act, 1);

body_pos_des = zeros(3,1);
body_vel_des = zeros(3,1);

contact_idx = [4, 6, 7, 10, 12, 13]; % 4:right heel, 6: right toe tip, 7: right toe back, 10: left heel, 12: left toe tip, 13: left toe back

HZD_ctrl = controller_HZD();
TVR_WBC_ctrl = walking_TVR_WBC(z0);

for i=1:num_step-1
    state = z_out(:,i);
    curr_time = dt * i;
    
    switch choice_ctrl
        case 0
            tau = controller_biped_toe(state);
        case 1 % HZD
            [tau, jpos_des, jvel_des] = HZD_ctrl.getJointCMD(z_out(:,i), curr_time);
            jpos_des_traj(:,i) = jpos_des;
            jvel_des_traj(:,i) = jvel_des;
            
            body_pos_des(:,i) = z_out(1:3,i);
            body_vel_des(:,i) = z_out(12:14,i);
            
        case 2
%             tau = wbc_jpos_ctrl(state, jpos_des, jvel_des, contact_idx);
            tau = wbc_body_ctrl(state, curr_time);
%             tau = wbc_body_foot_ctrl(state);
            jpos_des_traj(:,i) = jpos_des;
            jvel_des_traj(:,i) = jvel_des;
        case 3
            tau = TVR_WBC_ctrl.getJointCMD(state, curr_time);
            body_pos_des(:,i) = TVR_WBC_ctrl.body_des;
            body_vel_des(:,i) = TVR_WBC_ctrl.body_vel;
            
        case 4
            tau = MPC_WBIC_Walking(state, curr_time);            
    end
    dz = dynamics_biped(z_out(:,i), tau);
    tau_traj(:,i) = tau(4:end);
    z_out(:,i+1) = z_out(:,i) + dz*dt;
    z_out(dim/2+1:dim,i+1) = contact_constraint_biped_toe(z_out(:,i+1), restitution_coeff, friction_coeff);
    z_out(1:dim/2,i+1) = z_out(1:dim/2,i) + z_out(dim/2+1:dim,i+1)*dt;
end

sim_len = size(tau_traj, 2);

%% Plot body
b_body_plot = true;

if(b_body_plot)
    figure
    for i = 1:3
        subplot(3,1,i)
        hold on
        plot(tspan(1:sim_len), body_pos_des(i,:));
        plot(tspan(1:sim_len), z_out(i,1:sim_len));
    end
    xlabel('body pos')
    
    figure
    for i = 1:3
        subplot(3,1,i)
        hold on
        plot(tspan(1:sim_len), body_vel_des(i,:));
        plot(tspan(1:sim_len), z_out(i+11,1:sim_len));
    end
    xlabel('body vel')

end

%% Plot
figure('position', [0,0, 400, 600])
for i=1:4
    subplot(4,1,i)
    hold on 
    plot(tspan(1:sim_len), z_out(i+3,1:sim_len))
    plot(tspan(1:sim_len), jpos_des_traj(i,:))
end
xlabel('right jpos')

figure('position', [400,0, 400, 600])
for i=1:4
    subplot(4,1,i)
    hold on 
    plot(tspan(1:sim_len), z_out(i+3+4,1:sim_len))
    plot(tspan(1:sim_len), jpos_des_traj(i+4,:))
end
xlabel('left jpos')

figure('position', [800,0, 400, 600])
for i=1:4
    subplot(4,1,i)
    hold on 
    plot(tspan(1:sim_len), z_out(i+3 + dim_q,1:sim_len))
    plot(tspan(1:sim_len), jvel_des_traj(i,:))
end
xlabel('right jvel')

figure('position', [1200,0, 400, 600])
for i=1:4
    subplot(4,1,i)
    hold on 
    plot(tspan(1:sim_len), z_out(i+3+4 + dim_q,1:sim_len))
    plot(tspan(1:sim_len), jvel_des_traj(i+4,:))
end
xlabel('left jvel')

%%
% figure('position', [0,0, 400, 600])
% for i=1:4
%     subplot(4,1,i)
%     hold on 
%     plot(tspan(1:sim_len), tau_traj(i,1:sim_len))
% end
% xlabel('right jpos')
% 
% figure('position', [400,0, 400, 600])
% for i=1:4
%     subplot(4,1,i)
%     hold on 
%     plot(tspan(1:sim_len), tau_traj(i+4,1:sim_len))
% end
% xlabel('left jpos')
% 
% tau_sum = 0;
% for i = 1:sim_len-1
%         tau_sum = tau_sum + tau_traj(:,i).'*tau_traj(:,i);
% end


%% Filter torque
b_filtered_torque = true;

if(b_filtered_torque)
    for i = 1:8
    tau_traj_clean(i,:) = lowpass(tau_traj(i,:),0.01);
    % tau_traj_clean(i,:) = tau_traj(i,:);
    end

    tau_sum_filtered = 0;
    for i = 1:sim_len-1
            tau_sum_filtered = tau_sum_filtered + tau_traj_clean(:,i).'*tau_traj_clean(:,i);
    end
    tau_sum_filtered

    figure('position', [0,0, 400, 600])
    for i=1:4
        subplot(4,1,i)
        hold on 
        plot(tspan(1:sim_len), tau_traj_clean(i,1:sim_len), 'linewidth', 4)
        plot(tspan(1:sim_len), tau_traj(i,1:sim_len), 'linewidth', 2)
    end
    xlabel('right torque')

    figure('position', [400,0, 400, 600])
    for i=1:4
        subplot(4,1,i)
        hold on 
        plot(tspan(1:sim_len), tau_traj_clean(i+4,1:sim_len), 'linewidth',4)
        plot(tspan(1:sim_len), tau_traj(i+4,1:sim_len), 'linewidth', 2)
    end
    xlabel('left torque')
end

%% Animate Solution
fig = figure('Position',[900, 800, 700, 300]); clf
animate_biped_toe(tspan, z_out(:,1:sim_len), fig);