function tau = wbic(state, Jt_list, JtDot_list, task_acc_list, Jc, JcDot, Fr_des)
    dim = length(state)/2;
    
    qdot = state(dim+1:2*dim);
    
    A = A_biped(state);
    Ainv = inv(A);
    cori = coriolis_biped(state);
    grav = grav_biped(state);
    
    if (size(Jc) > 1)
        JcBar = fn_DynInv(Jc, Ainv, 1.e-5);
        qddot = JcBar*(-JcDot*qdot);
        Npre = fn_NullProj(Jc, Ainv, 1.e-5);
    else
        Npre = eye(dim);
        qddot = zeros(dim,1);
    end
    
    % Task
    num_task = length(Jt_list);
    for i = 1:num_task
        Jt = Jt_list{i};
        JtDot = JtDot_list{i};
        acc = task_acc_list{i};
        JtN = Jt*Npre;
        JtNBar = fn_DynInv(JtN,Ainv, 1.e-5);        
        qddot = qddot + JtNBar*(acc - Jt * qddot - JtDot*qdot);
        
        Npre = Npre*fn_NullProj(JtN, Ainv, 1.e-5);
    end

%     [sol_del_qdd, sol_del_rf] = fn_casadi_opt(Jc, Fr_des, A, cori, grav, qddot);
    if(size(Jc)>1)
        [sol_del_qdd, sol_del_rf] = fn_matlab_QP(Jc, Fr_des, A, cori, grav, qddot);
        tau = A*(qddot + [sol_del_qdd; zeros(8,1)]) + cori + grav - Jc.' * (Fr_des + sol_del_rf);
    else
        tau = A*qddot + cori + grav; % + [sol_del_qdd; zeros(4,1)];
    end
end

function JBar = fn_DynInv(J, Ainv, tol)
    JBar = Ainv * J.' * pinv(J*Ainv*J.', tol);
end

function N = fn_NullProj(J, Ainv, tol)
    dim = size(J,2);
    JBar = fn_DynInv(J, Ainv, tol);
    N = eye(dim) - JBar * J;
end

function [sol_del_qdd, sol_del_rf] = fn_matlab_QP(Jc, Fr_des, A, cori, grav, qddot)
    % sol = [ del_qdd; del_rf];
    dim_rforce = size(Jc,1);
    dim = length(qddot);

    H = eye(3 + dim_rforce); H(1:3, 1:3) = eye(3)*1.e6;
    f = zeros(3+dim_rforce,1);
    
    % Equality
    S_float = zeros(3, dim);
    S_float(1:3, 1:3) = eye(3);
    
    Aeq = [S_float*A*S_float.', -S_float*Jc.'];
    beq = S_float*(Jc.'*Fr_des - (A*qddot + cori + grav));
    
    % Inequality (reaction forces)
    mu = 0.4;
    Uf = [0, -1;
         -1, -mu;
          1, -mu];
    num_contact = dim_rforce/2;
    A_ieq = zeros(3*num_contact, 3 + 2*num_contact);
    b_ieq = zeros(3*num_contact,1); 
    for i = 1:num_contact
        A_ieq(3*i-2:3*i, 3 + 2*i-1: 3 + 2*i) = Uf;
        b_ieq(3*i-2:3*i) = -Uf*Fr_des(2*i-1:2*i) + [-2;0;0]; %(Fz > 10)
    end
    
    sol = quadprog(H,f,A_ieq,b_ieq,Aeq,beq);
    sol_del_qdd = sol(1:3);
    sol_del_rf = sol(4:end);
end

function [sol_del_qdd, sol_del_rf] = fn_matlab_QP_structure(Jc, Fr_des, A, cori, grav, qddot)
    % sol = [ del_qdd; del_rf];
    dim_rforce = size(Jc,1);
    dim = length(qddot);
    
    x = optimvar('x',3 + dim_rforce);
    H = eye(3 + dim_rforce); H(1:3, 1:3) = eye(3)*1.e3;
        
    objec = x.'*H*x;
    prob = optimproblem('Objective',objec);
    
    S_float = zeros(3, dim);
    S_float(1:3, 1:3) = eye(3);
    prob.Constraints.cons1 = S_float * (A * qddot + cori + grav - Jc.'*(Fr_des + x(4:end))) == x(1:3);
    
    % Inequality (reaction forces)
    mu = 0.6;
    Uf = [0, -1;
         -1, -mu;
          1, -mu];
    num_contact = dim_rforce/2;
    A_ieq = zeros(3*num_contact, 2*num_contact);
    b_ieq = zeros(3*num_contact,1); 
    for i = 1:num_contact
        A_ieq(3*i-2:3*i, 2*i-1: 2*i) = Uf;
        b_ieq(3*i-2:3*i) = -Uf*Fr_des(2*i-1:2*i);
    end
    prob.Constraints.cons2 = A_ieq*x(4:end) <= b_ieq;

    problem = prob2struct(prob);
    sol = quadprog(problem);
    sol_del_qdd = sol(1:3);
    sol_del_rf = sol(4:end);
end


function [sol_del_qdd, sol_del_rf] = fn_casadi_opt(Jc, Fr_des, A, cori, grav, qddot)
% ---------- QP solver
    addpath(genpath('../../casadi')) % make sure you have added your OS-specific casadi folder to MATLAB-Optimization
    addpath(genpath('../casadi')) % make sure you have added your OS-specific casadi folder to MATLAB-Optimization
    import casadi.*
% -----------------------
    opti = casadi.Opti(); % Optimization problem
    
    dim = length(qddot);
    dim_rforce = size(Jc,1);
    mu = 0.6;

    % ---- decision variables ---------
    del_qdd = opti.variable(3); 
    del_rf = opti.variable(dim_rforce); 
    % ---- cost ---------
    Q_qdd = eye(3)*10;
    Q_rf = eye(dim_rforce);
    cost = del_qdd.'*Q_qdd*del_qdd + del_rf.'*Q_rf*del_rf;
    opti.minimize(cost); 
    
    % ---- dynamic constraints --------
    S_float = zeros(3, dim);
    S_float(1:3, 1:3) = eye(3);
    opti.subject_to( S_float * (A * qddot + cori + grav - Jc.'*(Fr_des + del_rf)) == del_qdd ) % right foot contact
    
    Fr = Fr_des + del_rf;
    opti.subject_to(Fr(2) >=0)
    opti.subject_to(-mu*Fr(2) <= Fr(1) <= mu*Fr(2))
    
    % ---- initial values for solver ---
%     opti.set_initial(X, ones(1, N+1).*x_initial );

    % ---- solve NLP              ------
    p_opts = struct('expand',true); % expand to casadi variables to SX (10x speedup)
    s_opts = struct('max_iter',1.e2, 'print_level', 0);
    opti.solver('ipopt',p_opts,s_opts);    % set numerical backend
    sol = opti.solve();
    
    sol_del_qdd = sol.value(del_qdd);
    sol_del_rf = sol.value(del_rf);
% -----------------------
end