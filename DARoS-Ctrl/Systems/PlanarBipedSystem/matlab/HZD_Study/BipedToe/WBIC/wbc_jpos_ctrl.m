function tau_des = wbc_jpos_ctrl(state, jpos_des, jvel_des, contact_idx_list)
    dim = length(state)/2;
    dim_act = dim - 3;
    % Task Set
    Jt = zeros(dim_act, dim); Jt(1:dim_act, 4:dim) = eye(dim_act);
    JtDot = zeros(dim_act, dim);
    
%     task_acc = 5050.* (jpos_des - state(4:dim)) + 100.5*(jvel_des - state(dim+4:end));    
    task_acc = 300.* (jpos_des - state(4:dim)) + 8.5*(jvel_des - state(dim+4:end));    
%     task_acc = 750.* (jpos_des - state(4:dim)) + 20.5*(zeros(8,1) - state(dim+4:end));    
    
    % Toe
%     jidx = 4;
%     task_acc(jidx) = 100.* (jpos_des(jidx) - state(jidx + 3)) + 5.5*(jvel_des(jidx) - state(dim+3 + jidx));   
%     jidx = 3;
%     task_acc(jidx) = 100.* (jpos_des(jidx) - state(jidx + 3)) + 5.5*(jvel_des(jidx) - state(dim+3 + jidx));   

%     task_acc
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
%             JcDot(i*2-1:i*2, :) = zeros(2,dim);
            contact_dim = contact_dim + 2;
        end
        Fr_des = zeros(contact_dim,1);
    end
    Jt_list = {Jt};
    JtDot_list = {JtDot};
    task_acc_list = {task_acc};
    
%     Jc = [];
    tau_wbic = wbic(state, Jt_list, JtDot_list, task_acc_list, Jc, JcDot, Fr_des);
    tau_wbic(1:3) = zeros(3,1);
    
%     tau_max = [0;0;0;120;180;70;50;120;180;70;5];
%     tau_des = max(-tau_max, min(tau_wbic,tau_max));
    tau_des = tau_wbic;
end