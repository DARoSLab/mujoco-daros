clear all
close all
clc

%%
addpath(genpath('../../casadi')) % make sure you have added your OS-specific casadi folder to MATLAB-Optimization
addpath(genpath('../casadi')) % make sure you have added your OS-specific casadi folder to MATLAB-Optimization
import casadi.*

%% Derive dynamics
[kinematics,dynamics] = derive_system_casadi();

%% Formulate Optimization
% via trapezoidal Collocation

opti = casadi.Opti(); % Optimization problem

N  = 10;   % number of control intervals
dt = 0.021; % dynamics dt
T  = N*dt; % duration of stance phase
mu = 0.5;

dim_state = 14;
dim_input = 4;
bezier_order = 5;

% ---- decision variables ---------
X = opti.variable(dim_state,N+1); % [x, y, theta, hip_r, knee_r, hip_l, knee_l, dx, dy, dtheta, v_hip_r, v_knee_r, v_hip_l, v_knee_l]
U = opti.variable(dim_input, N);   % hip torque (could alternatively parameterize U by a spline
F = opti.variable(2, N);   %  reaction force (right foot contact)
alpha = opti.variable(dim_input, bezier_order+1); % bezier parameters 
post_impact_vel = opti.variable(dim_state/2);
impulse_force = opti.variable(2);

% ---- objective          ---------
body_th = -0.06;
z0 = [0.0; 0.54; body_th; 0.6 - body_th; -0.5 + body_th; ...
                          0.2 - body_th; -0.6 + body_th; 
      0.5; 0.0; 0.0; 0; 0; 0; 0 ];

x_initial = z0;
x_final = x_initial;
x_final(1) = x_initial(1) + 0.25;
x_final(4) = x_initial(6);
x_final(5) = x_initial(7);
x_final(6) = x_initial(4);
x_final(7) = x_initial(5);

for i = 1:dim_state
    initial_guess(i,:) = linspace(x_initial(i), x_final(i), N+1);
end

% cost
cost = 0;

v_des = 0.6;
for i= 1:N+1
    cost = cost + 2 * (v_des - X(8,i))^2;
    cost = cost + 100 * (x_initial(3) - X(3,i))^2;
    cost = cost + 50 * (x_initial(2) - X(2,i))^2;
    
    cost = cost + 5 * (X(8:14,i).'*X(8:14,i));
end

cost = cost + 10* impulse_force.'*impulse_force;

Qu = eye(dim_input)*0.01;
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
    Qf = Jf(1:2,:).'*Fk(1:2); % right foot only

    opti.subject_to( Jf(1:2,:)*Xk1(dim_state/2+1:dim_state) == zeros(2,1) ) % right foot contact

    opti.subject_to( Xk1(1:dim_state/2) - Xk(1:dim_state/2) == dt*Xk1(dim_state/2+1:dim_state) ) % Euler integration - position
    opti.subject_to( Ak1*(Xk1(dim_state/2+1:dim_state)-Xk(dim_state/2+1:dim_state))  == dt*(Q - grav1 - coriolis1 + Qf) ) % Euler integration - velocity
end

% ---- path constraints -----------
for k=1:N % loop over control intervals
    Xk  = X(:,k); 
    Xk1 = X(:,k+1);
    Uk  = U(:,k);
    tau_max = 200;
    vel_max = 15;
    opti.subject_to( -ones(dim_input,1)*tau_max <= Uk <= ones(dim_input,1)*tau_max)   % Control limits

    opti.subject_to( -pi/2 <= Xk1(3) <= pi/2 ) % Body ori
    opti.subject_to( -pi/2 <= Xk1(4) <= pi/2 ) % Hip joint
    opti.subject_to( -pi/2 <= Xk1(6) <= pi/2 ) % Hip joint

    opti.subject_to( -pi/2 <= Xk1(5) <= 0 )    % Knee joint
    opti.subject_to( -pi/2 <= Xk1(7) <= 0 )    % Knee joint

    % velocity constraints
    opti.subject_to( -vel_max <= Xk(11) <= vel_max ) % Hip joint
    opti.subject_to( -vel_max <= Xk(12) <= vel_max ) % Hip joint

    opti.subject_to( -vel_max <= Xk(13) <= vel_max )    % Knee joint
    opti.subject_to( -vel_max <= Xk(14) <= vel_max )    % Knee joint
    
    % swing foot height
    if (1 < k < N-1)
        key_pts = kinematics.keypoints(Xk1);
        l_ankle = key_pts(:,5);
        opti.subject_to(l_ankle(2) >= 0);
    end
    
    % reaction force
    Fk  = F(:,k);
    opti.subject_to( 0 <= Fk(2) <= 1000 )               % Unilateral force constraint (no pulling the ground)
    opti.subject_to( -mu*Fk(2) <= Fk(1) <= mu*Fk(2) )   % Friction cone       
end

% ---- zero dynamics constraints -----------
% phi_ini = pi/2 - (x_initial(3) + x_initial(4) + x_initial(5));
% phi_fin = pi/2 - (x_initial(3) + x_initial(6) + x_initial(7));

% hip to stance foot
phi_ini = pi/2 - (x_initial(3) + x_initial(4) + x_initial(5));
phi_fin = pi/2 - (x_final(3) + x_final(4) + x_final(5));

for k=1:N % loop over control intervals
    Xk  = X(:,k); 
    Xk1 = X(:,k+1);

    phi_k = pi/2 - (Xk(3) + Xk(4) + Xk(5));
    phi_k1 = pi/2 - (Xk1(3) + Xk1(4) + Xk1(5));
    
    phi_k_dot = (Xk(10) + Xk(11) + Xk(12));
    
    opti.subject_to( phi_k < phi_k1 ) % monotonic increase
    opti.subject_to( phi_ini <= phi_k <= phi_fin ) % monotonic increase

    s = (phi_k - phi_ini)/(phi_fin - phi_ini) ;
    sdot = (phi_k_dot )/(phi_fin - phi_ini);
    % bezier trajectory matching
    M = bezier_order;
    for jidx = 1:dim_input
        b = 0;
        bdot = 0;
        for ii = 0:M
            b = b + alpha(jidx,ii+1)* factorial(M)/(factorial(ii) * factorial(M-ii) )* s^ii *  (1-s)^(M-ii);
            
            if (ii < M)
                bdot = bdot + (alpha(jidx,ii+2) - alpha(jidx,ii+1))* factorial(M)/(factorial(ii) * factorial(M-ii -1) )... 
                      * s^ii *  (1-s)^(M-ii-1);
            end
        end 
        opti.subject_to( b == Xk(3+jidx) )
%         opti.subject_to( bdot == Xk(3+jidx + 7) )
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
J_left = J_c(3:4,:);

opti.subject_to( A_end * (post_impact_vel - X_end(dim_state/2+1:dim_state)) == J_left.' * impulse_force)
opti.subject_to(J_left * post_impact_vel == zeros(2,1))
opti.subject_to(impulse_force(2) >= 0)

% opti.subject_to(post_impact_vel == X_end(dim_state/2+1:dim_state))

% posture  mirror
opti.subject_to(X_end(2) == X_ini(2))
opti.subject_to(X_end(3) == X_ini(3))
opti.subject_to(X_end(4) == X_ini(6))
opti.subject_to(X_end(5) == X_ini(7))
opti.subject_to(X_end(6) == X_ini(4))
opti.subject_to(X_end(7) == X_ini(5))
% velocity mirror
opti.subject_to(post_impact_vel(1) == X_ini(8))
opti.subject_to(post_impact_vel(2) == X_ini(9))
opti.subject_to(post_impact_vel(3) == X_ini(10))
opti.subject_to(post_impact_vel(4) == X_ini(13))
opti.subject_to(post_impact_vel(5) == X_ini(14))
opti.subject_to(post_impact_vel(6) == X_ini(11))
opti.subject_to(post_impact_vel(7) == X_ini(12))


% ---- boundary conditions --------
opti.subject_to( X(1:dim_state/2,1) == x_initial(1:dim_state/2) );
% opti.subject_to( X(:,1) == x_initial );

% ---- initial values for solver ---
opti.set_initial(X, initial_guess);
%opti.set_initial(U, );
%opti.set_initial(F, );

% ---- solve NLP              ------
p_opts = struct('expand',true); % expand to casadi variables to SX (10x speedup)
s_opts = struct('max_iter',1.e6);
% opti.solver('ipopt',p_opts,s_opts);    % set numerical backend
opti.solver('ipopt',p_opts,s_opts);    % set numerical backend
sol = opti.solve();

%% ---- post-processing        ------
t = 0:dt:(N*dt);
fig = figure; clf
animate_biped_ellipse(t,sol.value(X),fig, true)

%%
sol.value(impulse_force)

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
    plot(t(1:end-1), sol.value(U(i,:)))
    xlabel('Torque')
end

figure
for i = 1:2
    subplot(2,1,i)
    plot(sol.value(F(i,:)))
    xlabel('Reaction force')
end
%% Bezier curve check
alpha = sol.value(alpha);
q = sol.value(X(1:dim_state/2,:));
dq = sol.value(X(dim_state/2 +1:dim_state,:));
len = size(q,2);
phi_ini = pi/2 - (q(3,1) + q(4,1) + q(5,1));
phi_fin = pi/2 - (q(3,end) + q(4,end) + q(5,end));

idx = 1;
for i = 1:len
    phi(i) = pi/2 - (q(3,i) + q(4,i) + q(5,i));
%     phi(i+1) = pi/2 - (q(3,i+1) + q(4,i+1) + q(5,i+1));

    phi_dot(i) = -dq(3,i) - dq(4,i) - dq(5,i);
%     phi_dot(i+1) = -dq(3,i+1) - dq(4,i+1) - dq(5,i+1);

    s = (phi(i) - phi_ini)/(phi_fin - phi_ini);
    sdot = (phi_dot(i))/(phi_fin - phi_ini);
    for j = 1:dim_input
        [joint_trj(j,i), jvel_trj(j,i)] = fn_bezier_pt(alpha(j,:), s, sdot);
    end
end
% jvel_trj
% r_knee = fn_bezier_curve(alpha(1,:), 200);

% figure
% plot(phi)

figure
for j = 1:dim_input
    subplot(dim_input, 1, j)
hold on
plot(joint_trj(j,:))
plot(q(3+j,:),'*')
end

figure
for j = 1:dim_input
    subplot(dim_input, 1, j)
hold on
plot(jvel_trj(j,:))
plot(dq(3+j,:),'*')
end