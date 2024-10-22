function qdot = per_contact_constraint_biped(z, rest_coeff, fric_coeff)
    dim = length(z);
    qdot = z(dim/2+1:dim);
    
    key_pts = keypoints_biped(z);
    key_pts_vel = keypoints_vel_biped(z);
    key_pts_J = J_biped(z);

    num_pts = size(key_pts,2);
    
    check_pt_idx = [4, 5, 8, 9];
    % select contact points that violate a constraint
    num_cp = 0;
    cp_idx = [];
    for i = 1:4
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
    
    A = A_biped(z);
    Ainv = inv(A);
    
    if(num_cp > 0)
    
    Fz_sum = zeros(num_cp,1);
    Fx_sum = zeros(num_cp,1);
    
    for iter = 1:20
        F_inc_sum = zeros(2,1);
        for j = 1:num_cp
          J  = J_cp(2*j-1:2*j,:);
          
          lambda = inv(J * Ainv * J.');
          F_inc = lambda*([0; vel_des(j)] - J*qdot);
          % vertical
          next_Fz_sum = max(0, Fz_sum(j)+F_inc(2));
          F_inc(2) = next_Fz_sum - Fz_sum(j);
          Fz_sum(j) = Fz_sum(j) + F_inc(2);

          % tagential
          next_Fx_sum = max(-fric_coeff*Fz_sum(j), min(Fx_sum(j) + F_inc(1), fric_coeff*Fz_sum(j)));
          F_inc(1) = next_Fx_sum - Fx_sum(j);
          Fx_sum(j) = Fx_sum(j) + F_inc(1);

          qdot = qdot + Ainv*J.'*F_inc;
          
          F_inc_sum = F_inc_sum + F_inc;
        end
        if(norm(F_inc_sum) < 1.e-3)
            break;
        end
    end
%     iter
    end % if(num_cp > 0)
end