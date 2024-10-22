clear all
close all
clc

%% Path
addpath(genpath('../../casadi')) % make sure you have added your OS-specific casadi folder to MATLAB-Optimization
addpath(genpath('../casadi')) % make sure you have added your OS-specific casadi folder to MATLAB-Optimization
import casadi.*

%% Derive dynamics
[kinematics,dynamics] = derive_system_casadi();

%% Formulate Optimization
% via trapezoidal Collocation

opti = casadi.Opti(); % Optimization problem

N  = 10;   % number of control intervals
dt = 0.05; % dynamics dt
T  = N*dt; % duration of stance phase
mu = 0.5;

dim_state = 14;
dim_input = 4;

% ---- decision variables ---------
X = opti.variable(dim_state,N+1); % [x, y, theta, hip_r, knee_r, hip_l, knee_l, dx, dy, dtheta, v_hip_r, v_knee_r, v_hip_l, v_knee_l]
U = opti.variable(4, N);   % hip torque (could alternatively parameterize U by a spline
F = opti.variable(4, N);   %  reaction force

% ---- objective          ---------
g = -9.81;
terminal_COM          = kinematics.COM(X(:,end)); 
terminal_com_y_height = terminal_COM(2);
terminal_com_y_vel    = terminal_COM(4);


body_th = -0.06;
z0 = [0.0; 0.54; body_th; 0.6 - body_th; -0.5 + body_th; ...
                          0.2 - body_th; -0.6 + body_th; 
      0.6; 0.0; 0.0; 0; 0; 0; 0 ];

x_initial = z0; 
x_final = x_initial;

Q = eye(dim_state);
Qu = eye(dim_input);

Q(1,1) = 10; % x pos
Q(2,2) = 200; % height
Q(3,3) = 20; % body angle
cost = (X(:,end) - x_final).'*Q*(X(:,end) - x_final);

for i = 1:N
    cost = cost + U(:,i).'*Qu*U(:,i);
end

opti.minimize(cost); 

% ---- dynamic constraints --------
for k=1:N % loop over control intervals
    Xk  = X(:,k); 
    Xk1 = X(:,k+1);
    Uk  = U(:,k);
    Q = [0; 0; 0; Uk];
    Fk  = F(:,k);

    Ak1 = dynamics.A(Xk1);
    grav1 = dynamics.grav(Xk1);
    coriolis1 = dynamics.cori(Xk1);
    Jf = kinematics.J_contact(Xk1);
    Qf = Jf(1:2,:).'*Fk(1:2) + Jf(3:4,:).'*Fk(3:4);
    
    opti.subject_to( Xk1(1:dim_state/2) - Xk(1:dim_state/2) == dt*Xk1(dim_state/2+1:dim_state) ) % Euler integration - position
    opti.subject_to( Ak1*(Xk1(dim_state/2+1:dim_state)-Xk(dim_state/2+1:dim_state))  == dt*(Q - grav1 - coriolis1 + Qf) ) % Euler integration - velocity
end

% ---- path constraints -----------
for k=1:N % loop over control intervals
    Xk  = X(:,k); 
    Xk1 = X(:,k+1);
    Uk  = U(:,k);
    tau_max = 200;
    opti.subject_to( -ones(dim_input,1)*tau_max <= Uk <= ones(dim_input,1)*tau_max)   % Control limits

    opti.subject_to( -pi/2 <= Xk1(3) <= pi/2 ) % Body ori
    opti.subject_to( -pi/2 <= Xk1(4) <= pi/2 ) % Hip joint
    opti.subject_to( -pi/2 <= Xk1(5) <= 0 )    % Knee joint
    
    % reaction force
    Fk  = F(:,k);
    opti.subject_to( 0 <= Fk(2) <= 1000 )
    opti.subject_to( 0 <= Fk(4) <= 1000 )         

    opti.subject_to( -mu*Fk(2) <= Fk(1) <= mu*Fk(2) )          % Unilateral force constraint (no pulling the ground)
    opti.subject_to( -mu*Fk(4) <= Fk(3) <= mu*Fk(4) )          % Unilateral force constraint (no pulling the ground)
end



% ---- boundary conditions --------
opti.subject_to( X(:,1) == x_initial );

% ---- initial values for solver ---
opti.set_initial(X, ones(1, N+1).*x_initial );
%opti.set_initial(U, );
%opti.set_initial(F, );

% ---- solve NLP              ------
p_opts = struct('expand',true); % expand to casadi variables to SX (10x speedup)
s_opts = struct('max_iter',1000);
% opti.solver('ipopt',p_opts,s_opts);    % set numerical backend
opti.solver('ipopt',p_opts,s_opts);    % set numerical backend
sol = opti.solve();

%% ---- post-processing        ------
t = 0:dt:(N*dt);
fig = figure; clf
animate_biped_ellipse(t,sol.value(X),fig, true)

%%
figure
state_start = ones(1, N+1).*x_initial;
for i = 1:5
    subplot(5,1,i)
    hold on
    plot(t, state_start(i,:))
    plot(t, sol.value(X(i,:)))
end

figure
for i = 1:dim_input
    subplot(dim_input,1,i)
    plot(t(1:end-1), sol.value(U(i,:)))
    xlabel('Torque')
end

figure
for i = 1:4
    subplot(4,1,i)
    plot(sol.value(F(i,:)))
    xlabel('Reaction force')
end
%%
com = kinematics.COM(sol.value(X));

figure
for i = 1:4
    subplot(4,1,i)
    plot(t, sol.value(com(i,:)))
end