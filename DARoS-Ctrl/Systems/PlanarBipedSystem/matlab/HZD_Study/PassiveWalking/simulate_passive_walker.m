restoredefaultpath

clc
clear all
close all
%%
z0 = [     0.1468
    0.2935
   -0.7466
   -0.0319]; 
%    
% z0 = [ 0.1704
%     0.3407
%    -0.8000
%    -0.0109];   
% p = [M; m; l; g];
params = [25; 5; 0.8; 9.81];
    
%% Perform Dynamic simulation
dt = 0.002;
tf = 3.0;
dim = length(z0);
num_step = floor(tf/dt);
tspan = linspace(0, tf, num_step); 

z_out = zeros(dim,num_step);
stance_pos = zeros(1, num_step);
curr_stance_foot = 0;

PushOff =  2;
[z0, PushOff] = passive_walker_opt(z0, -1.2);

z_out(:,1) = z0;
previous_switch = 0;

for i=1:num_step-1
    state = z_out(:,i);
    
    % Dynamics
    A = A_passive(state, params);
    cori = coriolis_passive(state,params);
    grav = grav_passive(state, params);
    
    dz = zeros(dim,1);
    dz(dim/2+1:dim) = -inv(A)*(cori + grav);    
    z_out(:,i+1) = z_out(:,i) + dz*dt;
    z_out(1:dim/2,i+1) = z_out(1:dim/2,i) + z_out(dim/2+1:dim,i+1)*dt;
    stance_pos(i) = curr_stance_foot;
    % Contact
    if (abs(2*z_out(1, i+1) - z_out(2,i+1)) < 1e-2 && z_out(2,i+1)<-0.1)
        keypoints = keypoints_passive(z_out(:,i+1), params);
        curr_stance_foot = keypoints(1,2) + stance_pos(i);        
        
        th = z_out(1,i+1);
        S = zeros(dim);
        S(1,1) = -1;
        S(2,1) = -2;
        S(3,2) = cos(2*th);
        S(4,2) = cos(2*th)*(1-cos(2*th));
%         pre_state = z_out(:,i+1)
        B = zeros(4,1);
        B(3,1) = sin(2*th);
        B(4,1) = (1-cos(2*th))*sin(2*th);
        z_out(:,i+1) = S * z_out(:,i+1) + B*PushOff;        
%         switched_state = z_out(:,i+1)

%         previous_switch = i;
%         [z0, PushOff] = passive_walker_opt(z_out(:,i+1), -0.8);
    end
end
% z_out(:,end)

%% Plot
sim_len = size(z_out, 2);

figure
for i=1:dim/2
    subplot(dim/2,1,i)
    hold on 
    plot(tspan(1:sim_len), z_out(i,1:sim_len))
end


figure
for i=1:dim/2
    subplot(dim/2,1,i)
    hold on 
    plot(tspan(1:sim_len), z_out(i + 2,1:sim_len))
end
%% Compute Energy
% E = energy_biped(z_out);
% figure(1); clf
% plot(tspan,E);xlabel('Time (s)'); ylabel('Energy (J)');

%% Compute foot position over time


%% Animate Solution
fig = figure(3); clf
animate_passive_walker(tspan, z_out, params, stance_pos, fig);