function tau = wbc_walking_ctrl(state, body_des, body_vel, f_pos,f_vel,f_acc, jpos_des, swing_foot_idx, contact_idx_list)
    dim = length(state)/2;
    dim_act = dim - 3;
    key_pts = keypoints_biped(state);
    key_pts_vel = keypoints_vel_biped(state);
    key_pts_J = J_biped(state);
    key_pts_JDot = Jdot_biped(state);
    
    % Task Set (body pos & ori)
    Jt_body = zeros(3, dim); Jt_body(1:3, 1:3) = eye(3);
    JtDot_body = zeros(3, dim);    
    task_acc_body = 150.* (body_des - state(1:3)) + 35.5*(body_vel - state(dim+1:dim+3));    
    
    % Foot
    foot_pos = key_pts(:,swing_foot_idx);
    foot_vel = key_pts_vel(:,swing_foot_idx);
    Jt_foot = key_pts_J(swing_foot_idx*2-1:swing_foot_idx*2,:);
    JtDot_foot = key_pts_JDot(swing_foot_idx*2-1:swing_foot_idx*2,:);
    Jt_foot(:,1:3) = zeros(2,3);
    JtDot_foot = zeros(2,dim);
    task_acc_foot = 650.*(f_pos - foot_pos) + 25.5*(f_vel - foot_vel) + f_acc;
    % TEST
%     task_acc_foot = [0; 1];
    
    % Joint Position
    Jt_joint = zeros(dim_act, dim); Jt_joint(1:dim_act, 4:dim) = eye(dim_act);
    JtDot_joint = zeros(dim_act, dim);
    task_acc_joint = 150.* (jpos_des - state(4:dim)) + 25.5*(zeros(dim_act,1) - state(dim+4:end));    

    
    % Contact Set
    contact_len = length(contact_idx_list);
    Fr_des = 0;
    Jc = [];
    JcDot = [];
    if(contact_len > 0)
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
     
    Jt_list = {Jt_body, Jt_foot, Jt_joint};
    JtDot_list = {JtDot_body, JtDot_foot, JtDot_joint};
    task_acc_list = {task_acc_body, task_acc_foot, task_acc_joint};        

%     Jt_list = {Jt_body, Jt_joint, Jt_foot};
%     JtDot_list = {JtDot_body, JtDot_joint, JtDot_foot};
%     task_acc_list = {task_acc_body, task_acc_joint, task_acc_foot};
     
%     Jt_list = {Jt_body, Jt_joint};
%     JtDot_list = {JtDot_body, JtDot_joint};
%     task_acc_list = {task_acc_body, task_acc_joint};
    
    tau = wbic(state, Jt_list, JtDot_list, task_acc_list, Jc, JcDot, Fr_des);
    tau(1:3) = zeros(3,1);
end