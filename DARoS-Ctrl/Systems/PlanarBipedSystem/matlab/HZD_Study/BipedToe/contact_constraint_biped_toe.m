function qdot = contact_constraint_biped_toe(z, rest_coeff, fric_coeff)
    dim = length(z);
    qdot = z(dim/2+1:dim);
    
    key_pts = keypoints_biped(z);
    key_pts_vel = keypoints_vel_biped(z);
    key_pts_J = J_biped(z);
    
%     keypoints: [pelvis 
%         knee_r ankle_r(3) heel_r(4) toe_joint_r(5) toe_tip_r toe_back_r(7)
%         knee_l ankle_l(9) heel_l(10) toe_joint_l(11) toe_tip_l(12) toe_back_l (13)
%         shoulder (14)]
    
    check_pt_idx = [4, 6, 7, 10, 12, 13];
    % select contact points that violate a constraint
    num_cp = 0;
    cp_idx = [];
    for i = 1:length(check_pt_idx)
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
    
    b_rknee_lim = false;
    b_lknee_lim = false;
    Frknee_sum = 0;
    Flknee_sum = 0;
    if (z(5) > 0)
        b_rknee_lim = true;
    end
    if (z(9) >0)
        b_lknee_lim = true;
    end
    
    if(num_cp > 0)
    
        Fz_sum = zeros(num_cp,1);
        Fx_sum = zeros(num_cp,1);

        for iter = 1:30
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

              % joint
              if(b_rknee_lim)
                  J_rknee = zeros(1, 11);
                  J_rknee(5) = 1;
                  lambda_rknee = 1/(J_rknee * Ainv * J_rknee.');
                  Frknee_inc = lambda_rknee * (0 - J_rknee * qdot);
                  nx_Frknee_sum = min(0, Frknee_sum + Frknee_inc);
                  Frknee_inc = nx_Frknee_sum - Frknee_sum;
                  qdot = qdot + Ainv*J_rknee.'*Frknee_inc;
                  Frknee_sum = Frknee_sum + Frknee_inc;
              end
              
              if(b_lknee_lim)
                  J_lknee = zeros(1, 11);
                  J_lknee(9) = 1;
                  lambda_lknee = 1/(J_lknee * Ainv * J_lknee.');
                  Flknee_inc = lambda_lknee * (0 - J_lknee * qdot);
                  nx_Flknee_sum = min(0, Flknee_sum + Flknee_inc);
                  Flknee_inc = nx_Flknee_sum - Flknee_sum;
                  qdot = qdot + Ainv*J_lknee.'*Flknee_inc;
                  Flknee_sum = Flknee_sum + Flknee_inc;
              end
            end
            if(Fz_inc_sum < 1.e-3)
                break;
            end
        end
%     iter
    end % if(num_cp > 0)
end