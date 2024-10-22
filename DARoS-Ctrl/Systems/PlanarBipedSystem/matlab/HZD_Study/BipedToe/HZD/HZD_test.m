clear all
close all
clc

%%
addpath(genpath('../RobotModel')) % make sure you have added your OS-specific casadi folder to MATLAB-Optimization

addpath(genpath('../../casadi')) % make sure you have added your OS-specific casadi folder to MATLAB-Optimization
addpath(genpath('../casadi')) % make sure you have added your OS-specific casadi folder to MATLAB-Optimization
import casadi.*

%% Derive dynamics
[kinematics,dynamics] = derive_system_casadi();
%% Formulate Optimization
% via trapezoidal Collocation

opti = casadi.Opti(); % Optimization problem

N  = 10;   % number of control intervals
dt = 0.045; % dynamics dt
T  = N*dt; % duration of stance phase
mu = 0.6;

dim_state = 22;
dim_q = dim_state/2;
dim_input = 8;
bezier_order = 8;

% ---- decision variables ---------
X = opti.variable(dim_state,N+1); % State (q, qdot)
U = opti.variable(dim_input, N);  % Joint toruqe
F = opti.variable(6, N);   %  reaction force (right foot contact) [x, y, x, y, x, y]
alpha = opti.variable(dim_input, bezier_order+1); % bezier parameters 
post_impact_vel = opti.variable(dim_state/2);
impulse_force = opti.variable(2);

% ---- objective          ---------
% v = 0.7 m/s
% v_des = 0.8;
% forward_dist = 0.31;
% z0 = [    0; 0.764; -0.02; 0.21+0.02; 0.0; 0.0; 0.0; -0.21+0.02; 0.0; 0.0; 0.21; ...
%        0.73; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];

forward_dist = 0.25;
v_des = 0.4;
z0 = [ 0; 0.764; -0.02; 0.15+0.02; 0; 0; 0; -0.15+0.02; 0.0; 0.0; 0; ...
       0.6; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];


key_pts = keypoints_biped(z0);
ini_heel_height = key_pts(2,4);

x_initial = z0;
x_final = x_initial;
x_final(1) = x_initial(1) + forward_dist;
x_final(4) = x_initial(6);
x_final(5) = x_initial(7);
x_final(6) = x_initial(4);
x_final(7) = x_initial(5);

for i = 1:dim_state
    initial_guess(i,:) = linspace(x_initial(i), x_final(i), N+1);
end

% cost
cost = 0;
for i= 1:N+1
    cost = cost + 2 * (v_des - X(dim_q+1,i))^2;
    cost = cost + 150 * (x_initial(3) - X(3,i))^2;
    cost = cost + 50 * (x_initial(2) - X(2,i))^2;    
    cost = cost + 5 * (X(dim_q+4:dim_state,i).'*X(dim_q+4:dim_state,i));
end

% cost = cost + 10* impulse_force.'*impulse_force;

Qu = eye(dim_input)*1.1;
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

    Ak1 = dynamics.A(Xk);
    grav1 = dynamics.grav(Xk);
    coriolis1 = dynamics.cori(Xk);
    Jf = kinematics.J_contact(Xk);
    
    
    pos_tol = 0.0001;
    if (k*dt < 0.12)
        opti.subject_to( Jf(1:2,:)*Xk1(dim_state/2+1:dim_state) == zeros(2,1) ) % right heel contact
        Qf = Jf(1:2,:).'*Fk(1:2); % right foot only  
        opti.subject_to(Fk(3:6) == zeros(4,1))
        key_pts = kinematics.keypoints(Xk1);
        % heel
%         opti.subject_to(key_pts(2,4) == ini_heel_height);
        opti.subject_to( (key_pts(2,4) - ini_heel_height)^2 < pos_tol);
    elseif(k*dt < 0.3)
        opti.subject_to( Jf(1:6,:)*Xk1(dim_state/2+1:dim_state) == zeros(6,1) ) % right foot contact
        Qf = Jf(1:6,:).'*Fk(1:6); % right foot only
        
        key_pts = kinematics.keypoints(Xk1);
%         opti.subject_to(key_pts(2,4) == ini_heel_height);
%         opti.subject_to(key_pts(2,5) == ini_heel_height);
%         opti.subject_to(key_pts(2,6) == ini_heel_height);
        opti.subject_to( (key_pts(2,4) - ini_heel_height)^2 < pos_tol);
        opti.subject_to( (key_pts(2,6) - ini_heel_height)^2 < pos_tol);
        opti.subject_to( (key_pts(2,7) - ini_heel_height)^2 < pos_tol);

    else
        opti.subject_to( Jf(3:6,:)*Xk1(dim_state/2+1:dim_state) == zeros(4,1) ) % right toe contact
        Qf = Jf(3:6,:).'*Fk(3:6); % right foot only        
        opti.subject_to(Fk(1:2) == zeros(2,1))
        key_pts = kinematics.keypoints(Xk1);
        opti.subject_to( key_pts(2,4) >= ini_heel_height);
        opti.subject_to( (key_pts(2,6) - ini_heel_height)^2 < pos_tol);
        opti.subject_to( (key_pts(2,7) - ini_heel_height)^2 < pos_tol);
    end
    
    opti.subject_to( Xk1(1:dim_state/2) - Xk(1:dim_state/2) == dt*Xk1(dim_state/2+1:dim_state) ) % Euler integration - position
    opti.subject_to( Ak1*(Xk1(dim_state/2+1:dim_state)-Xk(dim_state/2+1:dim_state))  == dt*(Q - grav1 - coriolis1 + Qf) ) % Euler integration - velocity
end

% ---- path constraints -----------
for k=1:N % loop over control intervals
    Xk  = X(:,k); 
    Xk1 = X(:,k+1);
    Uk  = U(:,k);
    tau_max = 200;
    vel_max = 25;
    opti.subject_to( -ones(dim_input,1)*tau_max <= Uk <= ones(dim_input,1)*tau_max)   % Control limits
    opti.subject_to( -pi/4 <= Xk(4) <= pi/4 ) % Hip joint
    opti.subject_to( -pi/4 <= Xk(4+dim_input/2) <= pi/4 ) % Hip joint (left, swing)

    opti.subject_to( -0.01 <= Xk(5) <= 0 )    % Knee joint  (right, stance)
    opti.subject_to( -pi/3 <= Xk(5+dim_input/2) <= 0 )    % Knee joint

    opti.subject_to( -pi/3 <= Xk(6) <= pi/3 )    % Ankle joint
    opti.subject_to( -pi/3 <= Xk(6+dim_input/2) <= pi/3 )    % Ankle joint

    opti.subject_to( -pi/6 <= Xk(7) <= pi/3 )    % Toe joint
    opti.subject_to( -pi/6 <= Xk(7+dim_input/2) <= pi/3 )    % Toe joint

    
    % velocity constraints
    opti.subject_to( -vel_max*ones(dim_input,1) <= Xk(dim_q+4:dim_state) <= vel_max*ones(dim_input,1) )
    
    % swing foot height
    if (2 < k < N-2)
        key_pts = kinematics.keypoints(Xk1);
%         l_heel = key_pts(:,10);
        opti.subject_to(key_pts(2, 10) >= 0.035);
        opti.subject_to(key_pts(2, 12) >= 0.035);
%         opti.subject_to(key_pts(2, 13) >= -0.001);
    end
    
    % reaction force
    Fk  = F(:,k);
    for i = 1:3
        opti.subject_to( 0 <= Fk(2*i) <= 1000 )               % Unilateral force constraint (no pulling the ground)
        opti.subject_to( -mu*Fk(2*i) <= Fk(2*i-1) <= mu*Fk(2*i) )   % Friction cone       
    end
end

% ---- zero dynamics constraints -----------
% phi_ini = pi/2 - (x_initial(3) + x_initial(4) + x_initial(5));
% phi_fin = pi/2 - (x_initial(3) + x_initial(6) + x_initial(7));

% hip to stance foot
% phi_ini = pi/2 - (x_initial(3) + x_initial(4) + x_initial(5));
% phi_fin = pi/2 - (x_final(3) + x_final(4) + x_final(5));

for k=1:N % loop over control intervals
    Xk  = X(:,k); 

%     phi_k = pi/2 - (Xk(3) + Xk(4) + Xk(5));
%     phi_k1 = pi/2 - (Xk1(3) + Xk1(4) + Xk1(5));
%     
%     phi_k_dot = (Xk(10) + Xk(11) + Xk(12));
    
%     opti.subject_to( phi_k < phi_k1 ) % monotonic increase
%     opti.subject_to( phi_ini <= phi_k <= phi_fin ) % monotonic increase

    s = (k-1)/N;
    
    % bezier trajectory matching
    M = bezier_order;
    for jidx = 1:dim_input
        b = 0;
        bdot = 0;
        for ii = 0:M
            b = b + alpha(jidx,ii+1)* factorial(M)/(factorial(ii) * factorial(M-ii) )* s^ii *  (1-s)^(M-ii);
        end 
        opti.subject_to( b == Xk(3+jidx) )
    end
end

% bezier trajectory matching
M = bezier_order;
s = 1;
for jidx = 1:dim_input
    b = 0;
    for ii = 0:M
        b = b + alpha(jidx,ii+1)* factorial(M)/(factorial(ii) * factorial(M-ii) )* s^ii *  (1-s)^(M-ii);
    end 
    opti.subject_to( b == X(3+jidx, N+1) )
end

% ---- Impact invariant -----------
X_end = X(:,N+1);
X_ini = X(:,1);
A_end = dynamics.A(X_end);
J_c = kinematics.J_contact(X_end);
J_heel_l = J_c(7:8,:);

opti.subject_to( A_end * (post_impact_vel - X_end(dim_state/2+1:dim_state)) == J_heel_l.' * impulse_force)
opti.subject_to(J_heel_l * post_impact_vel == zeros(2,1))
opti.subject_to(impulse_force(2) >= 0)

% -------- posture  mirror
opti.subject_to(X_end(2) == X_ini(2))
opti.subject_to(X_end(3) == X_ini(3))
% Joint (right)
opti.subject_to(X_end(4) == X_ini(8))
opti.subject_to(X_end(5) == X_ini(9))
opti.subject_to(X_end(6) == X_ini(10))
opti.subject_to(X_end(7) == X_ini(11))
% Joint (left)
opti.subject_to(X_end(8) == X_ini(4))
opti.subject_to(X_end(9) == X_ini(5))
opti.subject_to(X_end(10) == X_ini(6))
opti.subject_to(X_end(11) == X_ini(7))

% -------- velocity mirror
opti.subject_to(post_impact_vel(1) == X_ini(12))
opti.subject_to(post_impact_vel(2) == X_ini(13))
opti.subject_to(post_impact_vel(3) == X_ini(14))
% Joint (right)
opti.subject_to(post_impact_vel(4) == X_ini(19))
opti.subject_to(post_impact_vel(5) == X_ini(20))
opti.subject_to(post_impact_vel(6) == X_ini(21))
opti.subject_to(post_impact_vel(7) == X_ini(22))
% Joint (left)
opti.subject_to(post_impact_vel(8) == X_ini(15))
opti.subject_to(post_impact_vel(9) == X_ini(16))
opti.subject_to(post_impact_vel(10) == X_ini(17))
opti.subject_to(post_impact_vel(11) == X_ini(18))


% ---- boundary conditions --------
% opti.subject_to( X(1:dim_state/2,1) == x_initial(1:dim_state/2) );
opti.subject_to( X(1:3,1) == x_initial(1:3) );
opti.subject_to( X(4:7,1) == x_initial(4:7) );


% ---- initial values for solver ---
opti.set_initial(X, initial_guess);
%opti.set_initial(U, );
%opti.set_initial(F, );

% ---- solve NLP              ------
p_opts = struct('expand',true); % expand to casadi variables to SX (10x speedup)
s_opts = struct('max_iter',1.e4, 'tol', 5e-1,'constr_viol_tol', 0.1,'acceptable_constr_viol_tol', 0.02, ...
                                 'acceptable_tol',0.05);

% s_opts = {"max_cpu_time": 0.1, 
% 				  "print_level": 0, 
% 				  "tol": 5e-1, 
% 				  "dual_inf_tol": 5.0, 
% 				  "constr_viol_tol": 1e-1,
% 				  "compl_inf_tol": 1e-1, 
% 				  "acceptable_tol": 1e-2, 
% 				  "acceptable_constr_viol_tol": 0.01, 
% 				  "acceptable_dual_inf_tol": 1e10,
% 				  "acceptable_compl_inf_tol": 0.01,
% 				  "acceptable_obj_change_tol": 1e20,
% 				  "diverging_iterates_tol": 1e20}
opti.solver('ipopt',p_opts,s_opts);    % set numerical backend
sol = opti.solve();


%% ---- post-processing        ------
t = 0:dt:(N*dt);
% x_opt = opti.debug.value(X);
opt_x = sol.value(X);
opt_torque = sol.value(U);

fig = figure; clf
animate_biped_toe(t,opt_x,fig, true)

%%
sol.value(impulse_force)
% opti.debug.value(impulse_force)
figure
state_start = ones(1, N+1).*x_initial;
for i = 1:11
    subplot(11,1,i)
    hold on
    plot(t, state_start(i,:))
    plot(t, initial_guess(i,:))
    plot(t,opt_x(i,:))
end
xlabel('q')

figure
for i = 1:11
    subplot(11,1,i)
    hold on
    plot(t, opt_x(i+7,:))
    plot(t(end) + dt, sol.value(post_impact_vel(i)), '-*')
end
xlabel('qdot')

figure
torque_sum = 0;
for i = 1:dim_input
    subplot(dim_input,1,i)
    plot(t(1:end-1), opt_torque(i,:))
    torque_sum = torque_sum + opt_torque(i,:)* opt_torque(i,:).';
end
xlabel('Torque')
torque_sum

figure
for i = 1:6
    subplot(6,1,i)
    plot(sol.value(F(i,:)))
end
xlabel('Reaction force')
%% Bezier curve check
alpha = sol.value(alpha);
q = sol.value(X(1:dim_state/2,:));
dq = sol.value(X(dim_state/2 +1:dim_state,:));
len = size(q,2);
phi_ini = pi/2 - (q(3,1) + q(4,1) + q(5,1));
phi_fin = pi/2 - (q(3,end) + q(4,end) + q(5,end));

idx = 1;
for i = 1:len
%     phi(i) = pi/2 - (q(3,i) + q(4,i) + q(5,i));
%     phi(i+1) = pi/2 - (q(3,i+1) + q(4,i+1) + q(5,i+1));

%     phi_dot(i) = -dq(3,i) - dq(4,i) - dq(5,i);
%     phi_dot(i+1) = -dq(3,i+1) - dq(4,i+1) - dq(5,i+1);

    s = (i-1)/N;
    sdot = 1/N;
    for j = 1:dim_input
        [joint_trj(j,i), jvel_trj(j,i)] = fn_bezier_pt(alpha(j,:), s, sdot);
    end
end
% jvel_trj
% r_knee = fn_bezier_curve(alpha(1,:), 200);

% figure
% plot(phi)

figure('position', [0,0, 400, 800])
for j = 1:dim_input
    subplot(dim_input, 1, j)
hold on
plot(joint_trj(j,:))
plot(q(3+j,:),'*')
end
xlabel('jpos(bezier)')

figure('position', [400,0, 400, 800])
for j = 1:dim_input
    subplot(dim_input, 1, j)
hold on
plot(jvel_trj(j,:))
plot(dq(3+j,:),'*')
end

%%

alpha

[q(:,1); dq(:,1)]