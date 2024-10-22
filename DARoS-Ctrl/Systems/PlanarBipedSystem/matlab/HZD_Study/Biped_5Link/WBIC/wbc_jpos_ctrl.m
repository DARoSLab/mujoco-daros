function tau = wbc_jpos_ctrl(state, jpos_des, jvel_des, contact_idx_list)
    dim = length(state)/2;

    % Task Set
    Jt = zeros(4, dim); Jt(1:4, 4:7) = eye(4);
    JtDot = zeros(4, dim);
    task_acc = 450.* (jpos_des - state(4:7)) + 30.5*(jvel_des - state(11:14));    
    
    % Contact Set
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