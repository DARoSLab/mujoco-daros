function tau = wbc_body_ctrl(state, curr_time)
    dim = length(state)/2;
    dim_act = dim - 3;
    
    % Task Set (body pos & ori)
    Jt = zeros(3, dim); Jt(1:3, 1:3) = eye(3);
    JtDot = zeros(3, dim);
    
    body_des = [0; 0.7; -0.1] + [0; 0.02*sin(2*pi*1*curr_time); 0.1*sin(2*pi*1*curr_time)];
    task_acc = 50.* (body_des - state(1:3)) + 5.5*(zeros(3,1) - state(dim+1:dim+3));    
    
    % Contact Set
    contact_idx_list = [4, 6, 7, 10, 12, 13];
    contact_len = length(contact_idx_list);
    Fr_des = 0;
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
    
    Jt_list = {Jt};
    JtDot_list = {JtDot};
    task_acc_list = {task_acc};
    
    tau = wbic(state, Jt_list, JtDot_list, task_acc_list, Jc, JcDot, Fr_des);
    tau(1:3) = zeros(3,1);
end