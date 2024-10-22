function [alpha, phi_in, phi_out] = find_zero_dynamics()    
    addpath(genpath('../../casadi')) % make sure you have added your OS-specific casadi folder to MATLAB-Optimization
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
    bezier_order = 5;

    % ---- decision variables ---------
    X = opti.variable(dim_state,N+1); % [x, y, theta, hip_r, knee_r, hip_l, knee_l, dx, dy, dtheta, v_hip_r, v_knee_r, v_hip_l, v_knee_l]
    U = opti.variable(dim_input, N);   % hip torque (could alternatively parameterize U by a spline
    F = opti.variable(2, N);   %  reaction force (right foot contact)
    alpha = opti.variable(dim_input, bezier_order+1); % bezier parameters 
    post_impact_vel = opti.variable(dim_state/2);
    impulse_force = opti.variable(2);

    % ---- objective          ---------
%     g = -9.81;
%     terminal_COM          = kinematics.COM(X(:,end)); 
%     terminal_com_y_height = terminal_COM(2);
%     terminal_com_y_vel    = terminal_COM(4);

    global z0
    
    x_initial = z0; 
    cost = 0;
    
    % state cost
%     Q = eye(dim_state);
%     Q(1,1) = 10; % x pos
%     Q(2,2) = 200; % height
%     Q(3,3) = 20; % body angle
%     cost = (X(:,end) - x_final).'*Q*(X(:,end) - x_final);

    Qu = eye(dim_input);
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
        opti.subject_to( -ones(dim_input,1)*tau_max <= Uk <= ones(dim_input,1)*tau_max)   % Control limits

        opti.subject_to( -pi/2 <= Xk1(3) <= pi/2 ) % Body ori
        opti.subject_to( -pi/2 <= Xk1(4) <= pi/2 ) % Hip joint
        opti.subject_to( -pi/2 <= Xk1(6) <= pi/2 ) % Hip joint

        opti.subject_to( -pi/2 <= Xk1(5) <= 0 )    % Knee joint
        opti.subject_to( -pi/2 <= Xk1(7) <= 0 )    % Knee joint

        % reaction force
        Fk  = F(:,k);
        opti.subject_to( 0 <= Fk(2) <= 1000 )               % Unilateral force constraint (no pulling the ground)
        opti.subject_to( -mu*Fk(2) <= Fk(1) <= mu*Fk(2) )   % Friction cone       
    end
    
    % ---- zero dynamics constraints -----------
    phi_ini = pi/2 - (X(3,1) + X(4,1) + X(5,1));
    phi_fin = pi/2 - (X(3,N+1) + X(4,N+1) + X(5,N+1));

    for k=1:N % loop over control intervals
        Xk  = X(:,k); 
        Xk1 = X(:,k+1);
        
        phi_k = pi/2 - (Xk(3) + Xk(4) + Xk(5));
        phi_k1 = pi/2 - (Xk1(3) + Xk1(4) + Xk1(5));
        
        opti.subject_to( phi_k < phi_k1 ) % monotonic increase
        
        s = (phi_k - phi_ini)/(phi_fin - phi_ini);
        
        % bezier trajectory matching
%         b = zeros(dim_input,1);
        M = bezier_order;
        for jidx = 1:dim_input
            b = 0;
            for ii = 0:M
                b = b + alpha(jidx,ii+1)* factorial(M)/(factorial(ii) * factorial(M-ii) )* s^ii *  (1-s)^(M-ii);
            end            
%             opti.subject_to(b == Xk(3+jidx))
        end
    end
    % ---- Impact invariant -----------
    X_end = X(:,N+1);
    X_ini = X(:,1);
    A_end = dynamics.A(X_end);
    J_c = kinematics.J_contact(X_end);
    J_left = J_c(3:4,:);
    
    opti.subject_to( A_end * (post_impact_vel - X_end(dim_state/2+1:dim_state)) == J_left.' * impulse_force)
    opti.subject_to(J_left * post_impact_vel == zeros(2,1))
    
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

    % ---- initial values for solver ---
    opti.set_initial(X, ones(1, N+1).*x_initial );
    %opti.set_initial(U, );
    %opti.set_initial(F, );

    % ---- solve NLP              ------
    p_opts = struct('expand',true); % expand to casadi variables to SX (10x speedup)
    s_opts = struct('max_iter',1.e6);
    % opti.solver('ipopt',p_opts,s_opts);    % set numerical backend
    opti.solver('ipopt',p_opts,s_opts);    % set numerical backend
    sol = opti.solve();

    %% ---- post-processing        ------
    t = 0:dt:((N+1)*dt);
    fig = figure; clf
    animate_biped_ellipse(t,sol.value(X),fig, true)
    
end