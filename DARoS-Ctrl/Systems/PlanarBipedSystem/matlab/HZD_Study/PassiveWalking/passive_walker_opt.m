function [z0, PushOff] = passive_walker_opt(state, dth_cmd)

    % ---------- Opti solver
    addpath(genpath('../../casadi')) 
    addpath(genpath('../casadi'))
    import casadi.*
    
    [kinematics,dynamics] = derive_passive_system_casadi();


    % -----------------------
    opti = casadi.Opti(); % Optimization problem                
    dim_state = length(state);
%     dth_cmd = state(3);
    N = 15;
    % ---- decision variables ---------
    X = opti.variable(dim_state, N+1); % [x, y, theta, dx, dy, dtheta]
%     tau = opti.variable(2, N-1);
    P_push = opti.variable(1);   % push off
    %     dt = 0.025; %opti.variable(1);
    dt = opti.variable(N-1);
    % ---- objective          ---------
    cost = P_push*P_push*0.1;
    cost = cost + (X(3,1) - dth_cmd)* (X(3,1) - dth_cmd)*0.1;

%     for k =1:N-1
%         cost = cost + tau(:,k).'*tau(:,k)*1.01;
%     end

    opti.minimize(cost); 

    for k=1:N-1 % loop over control intervals
        Xk  = X(:,k); 
        Xk1 = X(:,k+1);

    %     Ak1 = dynamics.A(Xk1);
    %     grav1 = dynamics.grav(Xk1);
    %     coriolis1 = dynamics.cori(Xk1);

        Ak1 = dynamics.A(Xk);
        grav1 = dynamics.grav(Xk);
        coriolis1 = dynamics.cori(Xk);

        opti.subject_to(0.01 < dt(k))
        opti.subject_to(dt(k) <0.2)
        opti.subject_to(Xk1(1:dim_state/2) - Xk(1:dim_state/2) == dt(k)*Xk1(dim_state/2+1:dim_state) ) % Euler integration - position
        opti.subject_to(Ak1*(Xk1(dim_state/2+1:dim_state)-Xk(dim_state/2+1:dim_state))  == ... 
                                 dt(k)*( - grav1 - coriolis1) ) % Euler integration - velocity
    end

    th0 = X(1,1);    
    phi0 = X(2,1);

    opti.subject_to(2*th0 == phi0)
    opti.subject_to(th0 > 0.1)

    % final state
    th_f = X(1,N);
    S = [-1 0 0 0;
         -2 0 0 0;
         0 cos(2*th_f) 0 0;
         0 cos(2*th_f)*(1-cos(2*th_f)) 0 0];

    B = [0; 0; sin(2*th_f); (1-cos(2*th_f))*sin(2*th_f)];
    opti.subject_to(X(:,N+1) == S*X(:,N) + B * P_push);
    opti.subject_to(P_push>0)

    % opti.subject_to(X_final == X(:,1));
    % opti.subject_to( (X_final - X(:,1)).'*(X_final - X(:,1)) < 0.0001);
    opti.subject_to( (X(:,N+1) - X(:,1)).'*(X(:,N+1) - X(:,1)) < 0.0000000001);
    % opti.subject_to( X(:,N+1) == X(:,1));

    phi_f = X(2,N);
    %     opti.subject_to(th_f < -0.02)
    opti.subject_to(2*th_f == phi_f)

    % ---- solve NLP              ------
    p_opts = struct('expand',true); % expand to casadi variables to SX (10x speedup)
    %     s_opts = struct('max_iter',1.e2, 'print_level', 1);
    s_opts = struct('max_iter',1.e3);
    opti.solver('ipopt',p_opts,s_opts);    % set numerical backend
    sol = opti.solve();

    x_opt = sol.value(X);
    PushOff = sol.value(P_push);
    dt_opt = sol.value(dt);
%     tau_opt = sol.value(tau);
    z0 = x_opt(:,1);   
           
    PushOff
    dt_opt    
    x_opt
%     tau_opt
end