clear all
close all
clc

%%
addpath(genpath('../casadi')) % make sure you have added your OS-specific casadi folder to MATLAB-Optimization
import casadi.*

%%

z0 = [-0.05; 0.67; 0.6; -1.2;  ...
      0.5; 0.0]; %(xdot, ydot)
global l_shank l_thigh

l_thigh = 0.345;
l_shank = 0.355;

params = [l_thigh; l_shank];

%% Test

% N = 10;
% dt = 0.01;
% opt_x = z0 * ones(1, N);
% 
% t = 0:dt:(N*dt);
% fig = figure; clf
% animate_SingleLeg(t,opt_x,fig)



%% Derive dynamics
[kinematics] = derive_system_casadi();

%% Formulate Optimization
% via trapezoidal Collocation

opti = casadi.Opti(); % Optimization problem

N  = 12;   % number of control intervals
% dt = 0.021; % dynamics dt
% T  = N*dt; % duration of stance phase
mu = 0.7;

dim_state = 6;
dim_input = 1;

% ---- decision variables ---------
X = opti.variable(dim_state,N+1); % [x, y, theta, hip_r, knee_r, hip_l, knee_l, dx, dy, dtheta, v_hip_r, v_knee_r, v_hip_l, v_knee_l]
F = opti.variable(dim_input, N);   % hip torque (could alternatively parameterize U by a spline
dt_list = opti.variable(1, N);   %  reaction force (right foot contact)
% ---- objective          ---------
x_initial = z0;

% cost
% cost = -(X(1,1)-X(1,end))*(X(1,1)-X(1,end));
% cost = cost -X(6,1)*X(6,1);
% cost = cost -X(7,1)*X(7,1);

cost = -2*X(5,end)*X(6,end)/9.81;
for i = 1:N
    cost = cost - X(5,i)*dt_list(i);
end

% Qf = eye(dim_input)*0.001;
% for i = 1:N
%     cost = cost + F(:,i).'*Qf*F(:,i);
% end
opti.minimize(cost); 

m_body = 15; %(kg)

opti.subject_to(F(1) < 5)
opti.subject_to(F(N) < 5)

% ---- dynamic constraints --------
for k=1:N % loop over control intervals
    Xk  = X(:,k); 
    Xk1 = X(:,k+1);
    th = atan(Xk(2)/(Xk(1)+0.001));
    opti.subject_to(F(k) > 0);
    if(k < N)
    opti.subject_to(abs(F(k+1) - F(k))<50);
    end
    opt_fvec = [sign(th)*F(k)*cos(th); F(k)*sin(abs(th))];
    dt = dt_list(k);

    Jf = kinematics.J_contact(Xk);
    
    Qf = Jf.'*opt_fvec;
    
%     opti.subject_to(Xk(1) < -0.03)
    opti.subject_to(Xk(2) > 0)
    opti.subject_to(Xk(5) > 0.0)
    opti.subject_to(0.001 < dt < 0.5)
    
    opti.subject_to( Xk1(1:2) - Xk(1:2) == dt*Xk1(5:6) ) % Euler integration - position
    opti.subject_to( [m_body, 0; 0, m_body]*(Xk1(5:6)-Xk(5:6)) ...
        == dt*(Qf(1:2) - [0; 9.81*m_body]) ) % Euler integration - velocity
    
     % reaction force
%     opti.subject_to( 0 <= Fk(2) <= 5000 )               % Unilateral force constraint (no pulling the ground)
%     opti.subject_to( -mu*Fk(2) <= Fk(1) <= mu*Fk(2) )   % Friction cone       
    
%     opti.subject_to( (Fk(2)/Fk(1)) == (Xk(2)/Xk(1)) );

%     opti.subject_to( ((Fk(2)/Fk(1)) - (Xk(2)/Xk(1)))* ...
%                      ((Fk(2)/Fk(1)) - (Xk(2)/Xk(1))) < 10.1);
%     opti.subject_to( ((Fk(4)/Fk(3)) - (Xk(2)/(Xk(1)-toe_pos(1))))* ...
%                      ((Fk(4)/Fk(3)) - (Xk(2)/(Xk(1)-toe_pos(1)))) < 0.1);
%     
    tau_max = [60; 80];
    opti.subject_to( -tau_max <= Qf(3:4) <= tau_max)   % Control limits
    
    
end

% ---- path constraints -----------
for k=1:N+1 % loop over control intervals
    Xk  = X(:,k);     
    
    opti.subject_to( -pi/2 <= Xk(3) <= pi/2 ) % Hip joint
    opti.subject_to( -pi/3 <= Xk(4) <= 0 )    % Knee joint
       
    key_pts = kinematics.keypoints(Xk);

%     opti.subject_to(key_pts(:,4) == heel_pos);
%     opti.subject_to(key_pts(:,5) == toe_pos);    

    opti.subject_to(key_pts(:,3).'*key_pts(:,3) < 0.001);
%         opti.subject_to(key_pts(:,3) == zeros(2,1)); 
end

% ---- perioidic constraints -----------
% opti.subject_to(X(1,1) == -X(1,end))
% opti.subject_to( abs(X(1,end) - X(1,1)) <0.001) %symmetry constraint

% initial height == final height
opti.subject_to(X(2,1) == X(2,end))
% initial x_vel == final x_vel
opti.subject_to(X(5,1) == X(5, end))
opti.subject_to(X(6,1) == -X(6,end))
opti.subject_to(X(6,1) < 0)


% ---- initial values for solver ---
% opti.set_initial(X, initial_guess);
%opti.set_initial(U, );
%opti.set_initial(F, );

% ---- solve NLP              ------
p_opts = struct('expand',true); % expand to casadi variables to SX (10x speedup)
s_opts = struct('max_iter',1.e4, 'tol', 5e-1,'constr_viol_tol', 0.1,'acceptable_constr_viol_tol', 0.02, ...
                                 'acceptable_tol',0.05);
% opti.solver('ipopt',p_opts,s_opts);    % set numerical backend
opti.solver('ipopt',p_opts,s_opts);    % set numerical backend
sol = opti.solve();

%% ---- post-processing        ------
debug = true;

if(debug)
    opt_x = opti.debug.value(X);
    opt_dt_list = opti.debug.value(dt_list);
    opt_f = opti.debug.value(F);    
else
    opt_x = sol.value(X);
    opt_dt_list = sol.value(dt_list);
    opt_f = sol.value(F);    
end

t = zeros(1,N+1);
for i = 1:N
    t(i+1) = t(i) + opt_dt_list(i);
end
fig = figure; clf
animate_SingleLeg(t,opt_x,fig)
opt_x
opt_dt_list
%% Torque

torque = zeros(4,N);
opt_fvec = zeros(2,N);
for i = 1:N   
    J = J_biped(opt_x(:,i), params);
    
    th = atan(opt_x(2,i)/(opt_x(1,i) + 0.001));
    opt_fvec(:,i) = [sign(th)*opt_f(i)*cos(th); opt_f(i)*sin(abs(th))];
%    torque(:,i) = J.'*opt_f(:,i);
   torque(:,i) = J.'*opt_fvec(:,i);
end

figure
for i = 1:4
    subplot(4,1,i)
    plot(torque(i,:))
end
xlabel('torque')

%% State

% State Check
state_check = zeros(6, N+1);
state_check(:,1) = opt_x(:,1);

for i = 1:N
    dt = opt_dt_list(i);
    Jf = J_biped(opt_x(:,i), params);
    Qf = Jf.'*opt_fvec(:,i);
    state_check(5:6,i+1) = state_check(5:6,i) + ...
        inv([m_body, 0; 0, m_body]) * dt*(Qf(1:2) - [0; 9.81*m_body]); % Euler integration - velocity
         
    state_check(1:2,i+1) = state_check(1:2,i) + dt*state_check(5:6,i+1);

end

figure
for i = 1:6
    subplot(6,1,i)
    hold on
    plot(t, opt_x(i,:) )
    plot(t, state_check(i,:) )
    axis tight
end
xlabel('state')

%% Reaction Force
figure


for i = 1:2
    subplot(2,1,i)
    hold on
    plot(opt_fvec(i,:))
    if i == 1
        plot(mu*opt_fvec(2,:));
        plot(-mu*opt_fvec(2,:));
    end

    xlabel('Reaction force')
end

%%
figure
state_start = ones(1, N+1).*x_initial;
for i = 1:7
    subplot(7,1,i)
    hold on
    plot(t, state_start(i,:))
    plot(t, sol.value(X(i,:)))
end
xlabel('q')

figure
for i = 1:7
    subplot(7,1,i)
    hold on
    plot(t, sol.value(X(i+7,:)))
    plot(t(end) + dt, sol.value(post_impact_vel(i)), '-*')
end
xlabel('qdot')

figure
for i = 1:dim_input
    subplot(dim_input,1,i)
    plot(t(1:end-1), sol.value(F(i,:)))
    xlabel('Torque')
end

