function qdot = contact_constraint_biped_toe(z, rest_coeff, fric_coeff)
    dim = length(z);
    qdot = z(dim/2+1:dim);
    
    key_pts = keypoints_biped_toe(z);
    key_pts_vel = keypoints_vel_biped_toe(z);
    key_pts_J = J_biped_toe(z);
    A = A_biped_toe(z);
    Ainv = inv(A);
    
    check_pt_idx = [4, 5, 6, 9, 10, 11];
    % select contact points that violate a constraint
    num_cp = 0;
    cp_idx = [];
    for i = 1:6
        idx = check_pt_idx(i);
        if(key_pts(2,idx) < 0)
            J_cp(2*num_cp+1:2*num_cp+2,:) = key_pts_J(idx*2-1:idx*2,:);
            vel_des(num_cp+1) = key_pts_vel(2,idx);
            if(vel_des(num_cp+1) < 0)
                vel_des(num_cp+1) = -vel_des(num_cp+1)*rest_coeff;
            end
            num_cp = num_cp + 1;
            cp_idx(num_cp) = idx;
        end
    end
    

    
    if(num_cp > 0)
    
    Fz_sum = zeros(num_cp,1);
    Fx_sum = zeros(num_cp,1);
    
    for iter = 1:20
        Fz_inc_sum = 0;
        for j = 1:num_cp
          J  = J_cp(2*j-1:2*j,:);
          
          % vertical
          J_z = J(2,:);
          lambda_z = 1/(J_z * Ainv * J_z.');
          Fz_inc = lambda_z*(vel_des(j) - J_z*qdot);
          next_Fz_sum = max(0, Fz_sum(j)+Fz_inc);
          Fz_inc = next_Fz_sum - Fz_sum(j);
          qdot = qdot + Ainv*J_z.'*Fz_inc;
          Fz_sum(j) = Fz_sum(j) + Fz_inc;

          % tagential
          J_x = J(1,:);
          lambda_x = 1/(J_x * Ainv * J_x.');
          Fx_inc = lambda_x * (0 - J_x * qdot);
          next_Fx_sum = max(-fric_coeff*Fz_sum(j), min(Fx_sum(j) + Fx_inc, fric_coeff*Fz_sum(j)));
          Fx_inc = next_Fx_sum - Fx_sum(j);
          qdot = qdot + Ainv*J_x.'*Fx_inc; 
          Fx_sum(j) = Fx_sum(j) + Fx_inc;
          
          Fz_inc_sum = Fz_inc_sum + Fz_inc;
        end
        if(Fz_inc_sum < 1.e-3)
            break;
        end
    end
%      iter
    end % if(num_cp > 0)
end