clc
clear all
close all

addpath('model');


%%
global z0 dt
z0 = [0.1; 0.2; 0.3; 12; 0; 0];

restitution_coeff = 0.0;
friction_coeff = 0.5;   

%% Perform Dynamic simulation
dt = 0.005;
tf = 5.204;
dim = 6;
num_step = floor(tf/dt);
tspan = linspace(0, tf, num_step); 

z_out = zeros(dim,num_step);
z_out(:,1) = z0;


for i=1:num_step-1
    dz = dynamics_arm(z_out(:,i));
    z_out(:,i+1) = z_out(:,i) + dz*dt;
%     z_out(dim/2+1:dim,i+1) = contact_constraint_biped(z_out(:,i+1), restitution_coeff, friction_coeff);
    z_out(1:dim/2,i+1) = z_out(1:dim/2,i) + z_out(dim/2+1:dim,i+1)*dt;
end

%% Compute Energy
E = energy_arm(z_out);
figure(1); clf
plot(tspan,E);xlabel('Time (s)'); ylabel('Energy (J)');

%% Animate Solution
fig = figure(3); clf
animate_arm(tspan, z_out, fig);