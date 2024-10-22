function tau = MPC_WBIC_Walking(state, curr_time)
    global z0    
    dim_q = length(state)/2;
    
    Mass = 29; %(kg)
    Inertia = 0.7; % (kg*m^2)
    mpc = WalkingManager(0.05, Mass, Inertia);
    % state: [x, y, theta, dx, dy, dth]
    x_des = [0, 0.74, -0.5, 0.5, 0, 0].' * ones(1,mpc.Nstep+1);

    
    foot_pos = [  0.1, -0.1;
                    0,    0];
    body_state = [state(1:3); state(dim_q+1:dim_q+3)];
    [F_list, OptTrj, fpos_cmd, fvel_cmd, facc_cmd, stance_leg] = mpc.runWalking(body_state, x_des, foot_pos, curr_time);

    
    %% Task Set 
    % (body pos & ori)
    JtBody = zeros(3, dim_q); JtBody(1:3, 1:3) = eye(3);
    JtDotBody = zeros(3, dim_q);
    
    body_des = [0; 0.7; -0.1] + [0; 0.02*sin(2*pi*1*curr_time); 0.1*sin(2*pi*1*curr_time)];
    task_acc_body = 150.* (body_des - state(1:3)) + 10.5*(zeros(3,1) - state(dim_q+1:dim_q+3));    
    
    % foot
    
    
    % full joint
    dim_act = dim_q - 3;
    JtJoint = zeros(dim_act, dim_q); JtJoint(1:dim_act, 4:dim_q) = eye(dim_act);
    JtDotJoint = zeros(dim_act, dim_q);
    task_acc_joint = 350.* (z0(4:dim_q) - state(4:dim_q)) + 10.5*(zeros(dim_q-3,1) - state(dim_q+4:end));   
    
    %% Contact Set
    contact_idx_list = [4, 6, 7, 10, 12, 13];
    contact_len = length(contact_idx_list);

    Jc = [];
    JcDot = [];
    if(contact_len > 0)
        key_pts_J = J_biped(state);
        key_pts_JDot = Jdot_biped(state);
        contact_dim = 0;
        
        for i = 1:contact_len        
            contact_idx = contact_idx_list(i);
            % accumlate Jc, assume that each contact has (fr_x, fr_y) 
            Jc(i*2-1:i*2, :) = key_pts_J(contact_idx*2-1:contact_idx*2,:);
            JcDot(i*2-1:i*2, :) = key_pts_JDot(contact_idx*2-1:contact_idx*2,:);
            contact_dim = contact_dim + 2;
        end
        Fr_des = zeros(contact_dim,1);
    end
    
    Jt_list = {JtBody, JtFoot, JtJoint};
    JtDot_list = {JtDotBody, JtDotFoot, JtDotJoint};
    task_acc_list = {task_acc_body, task_acc_foot, task_acc_joint};
    
    tau = wbic(state, Jt_list, JtDot_list, task_acc_list, Jc, JcDot, F_list{1});
    tau(1:3) = zeros(3,1);
end