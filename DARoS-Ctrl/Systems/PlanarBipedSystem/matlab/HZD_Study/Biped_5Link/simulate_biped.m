restoredefaultpath

clc
clear all
close all
%% Path
addpath('./RobotModel')
addpath('./HZD')
addpath('./WBIC')
addpath('./../Utils')


%%
global z0 dt
body_th = -0.06;
% z0 = [0.0; 0.54; body_th; 0.6 - body_th; -0.5 + body_th; ...
%                           0.2 - body_th; -0.6 + body_th; 
%       0.3; 0.0; 0.0; 0; 0; 0; 0 ];

z0 = [          0
    0.5400
   -0.0600
    0.6600
   -0.5600
    0.2600
   -0.6600
    0.9
    0.1213
   -0.1071
   -0.3315
   -3.8895
   -4.0333
    3.2135 ];
restitution_coeff = 0.0;
friction_coeff = 1.0;   

%% Optimization (HZD) ---- 

% [alpha, phi_ini, phi_fin] = find_zero_dynamics();

alpha = [     0.6600   -1.3415    1.3681   -0.8192    0.8386    0.2600
   -0.5600    0.3480   -1.1013    0.0291   -1.0944   -0.6600
    0.2600   -0.9104    0.9892   -0.1151    0.8939    0.6600
   -0.6600   -0.4900   -0.7290   -0.5921   -0.8474   -0.5600 ];
phi_ini = 1.5308;
phi_fin = 2.0308;

% phi = pi/2 - (z_out(3) + z_out(hip_idx) + z_out(knee_idx));
% s = (phi - phi_ini)/(phi_fin - phi_ini);


%% Perform Dynamic simulation
dt = 0.001;
tf = 2.95;
dim = length(z0);
num_step = floor(tf/dt);
tspan = linspace(0, tf, num_step); 

z_out = zeros(dim,num_step);
z_out(:,1) = z0;

right_stance = true;
stance_switch_iter = 1;


jpos_des = z0(4:7);
jvel_des = zeros(4,1);

jpos_des_traj = zeros(4,1);
jvel_des_traj = zeros(4,1);

choice_ctrl = 1; % 0: jpos still, 1: HZD, 2: WBC
contact_idx = [3, 5]; % 3:right foot, 5: left foot
swing_leg_idx = 5;
stance_leg_idx = 3;
for i=1:num_step-1
    state = z_out(:,i);
    
    switch choice_ctrl
        case 0
            tau = controller_biped(state);
        case 1
            % HZD
            if(right_stance)
                phi(i) = pi/2 - (state(3) + state(4) + state(5)); 
            else
                phi(i) = pi/2 - (state(3) + state(6) + state(7));
            end

            [tau, jpos_des, jvel_des] = controller_HZD(z_out(:,i), alpha, phi_ini, phi_fin, right_stance, stance_switch_iter);
            jpos_des_traj(:,i) = jpos_des;
            jvel_des_traj(:,i) = jvel_des;
            stance_switch_iter = stance_switch_iter + 1;


            key_pts = keypoints_biped(state);
            swing_leg_pos = key_pts(:,swing_leg_idx);
            stance_leg_pos = key_pts(:,stance_leg_idx);
%             if(abs(phi(i) - phi_fin)<0.0005 && stance_switch_iter>20)            
%             if((swing_leg_pos(2)<=stance_leg_pos(2)) && stance_switch_iter>100)
            if((swing_leg_pos(2)<=0) && stance_switch_iter>100)
                % print out 'Switching Stance'
                'Switching Stance'
                stance_switch_iter = 1;
                right_stance = ~right_stance;
                if(swing_leg_idx == 5)  
                    swing_leg_idx = 3;
                    stance_leg_idx = 5;
                else
                    swing_leg_idx = 5;
                    stance_leg_idx = 3;
                end
                % terminate the loop and fill the last state with the current one
%                 z_out(:,i+1:end) = repmat(z_out(:,i),1,num_step-i);
%                 break;

            end
            % End of HZD
            
        case 2
            tau = wbc_jpos_ctrl(state, jpos_des, jvel_des, contact_idx);
            jpos_des_traj(:,i) = jpos_des;
            jvel_des_traj(:,i) = jvel_des;
    end

    dz = dynamics_biped(z_out(:,i), tau);
    z_out(:,i+1) = z_out(:,i) + dz*dt;
    z_out(dim/2+1:dim,i+1) = contact_constraint_biped(z_out(:,i+1), restitution_coeff, friction_coeff);
    z_out(1:dim/2,i+1) = z_out(1:dim/2,i) + z_out(dim/2+1:dim,i+1)*dt;
end
% z_out(:,end)

%% Plot
sim_len = size(jpos_des_traj, 2);

figure
for i=1:4
    subplot(4,1,i)
    hold on 
    plot(tspan(1:sim_len), z_out(i+3,1:sim_len))
    plot(tspan(1:sim_len), jpos_des_traj(i,:))
end


figure
for i=1:4
    subplot(4,1,i)
    hold on 
    plot(tspan(1:sim_len), z_out(i+3 + 7,1:sim_len))
    plot(tspan(1:sim_len), jvel_des_traj(i,:))
end
%% Compute Energy
% E = energy_biped(z_out);
% figure(1); clf
% plot(tspan,E);xlabel('Time (s)'); ylabel('Energy (J)');

%% Compute foot position over time


%% Animate Solution
fig = figure(3); clf
animate_biped_ellipse(tspan, z_out, fig);