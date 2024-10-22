clc
clear all
close all

%%
global z0 dt
body_th = 0;
z0 = [0.0; 0.67; body_th; 0.25 - body_th; -0.4 + body_th; 0.15 - body_th; 0.0; ...
                          0.15 - body_th; -0.4 + body_th; 0.25 - body_th; 0; 
      0.0; -0.01; 0.105; 0; 0; 0; 0; 0; 0; 0; 0];

restitution_coeff = 0.0;
friction_coeff = 0.5;   

%% Perform Dynamic simulation
dt = 0.002;
tf = 1.7;
dim = 22;
num_step = floor(tf/dt);
tspan = linspace(0, tf, num_step); 

z_out = zeros(dim,num_step);
z_out(:,1) = z0;

dyn_sim = true; 

if (dyn_sim)
% forward simulation (basic)
for i=1:num_step-1
    dz = dynamics_biped_toe(z_out(:,i));
    z_out(:,i+1) = z_out(:,i) + dz*dt;
    z_out(dim/2+1:dim,i+1) = contact_constraint_biped_toe(z_out(:,i+1), restitution_coeff, friction_coeff);
%     z_out(dim/2+1:dim,i+1) = per_contact_constraint_biped(z_out(:,i+1), restitution_coeff, friction_coeff);
    z_out(1:dim/2,i+1) = z_out(1:dim/2,i) + z_out(dim/2+1:dim,i+1)*dt;
end

else
% Position based Sim
for i=1:num_step-1
    dz = dynamics_biped(z_out(:,i));
%     z_out(:,i+1) = constraint_pos_biped(z_out(:,i), dz, dt, restitution_coeff, friction_coeff);   
    z_out(:,i+1) = constraint_hybrid_biped(z_out(:,i), dz, dt, restitution_coeff, friction_coeff);   
end

end % if(dyn_sim)

%% Compute Energy
E = energy_biped(z_out);
figure(1); clf
plot(tspan,E);xlabel('Time (s)'); ylabel('Energy (J)');

%% Compute foot position over time


%% Animate Solution
fig = figure(3); clf
animate_biped_ellipse_toe(tspan, z_out, fig);