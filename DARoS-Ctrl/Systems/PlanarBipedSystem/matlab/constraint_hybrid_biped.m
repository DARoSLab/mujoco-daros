function z_next = constraint_hybrid_biped(z, dz, dt, rest_coeff, fric_coeff)
    dim = length(z);
    q = z(1:dim/2);
    qdot = z(dim/2+1:end);
    check_pt_idx = [4, 5, 8, 9];
    
    key_pts_ini = keypoints_biped([q; qdot]);
    key_pts = key_pts_ini;
    
    A = A_biped([q; qdot]);
    Ainv = inv(A);  

    %% Simulation
    num_iter = 10;
    h = dt/num_iter;
    
    alpha = 1/1000;
    alpha_tilde = alpha/h^2;
    
    % select contact points that violate a constraint
    num_check_pts = 4;
    check_idx = [4, 5, 8, 9];

    
    q_pre = q;
    
    for main_iter = 1:num_iter
        qdot = qdot + dz(dim/2+1:end)*h;
        
        key_pts_vel = keypoints_vel_biped([q;qdot]);
        key_pts_J = J_biped([q; qdot]);

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
        
        if(num_cp > 0)    
            Fz_sum = zeros(num_cp,1);
            Fx_sum = zeros(num_cp,1);

            for iter = 1:5
                Fz_inc_sum = 0;
                for j = 1:num_cp
                  J  = J_cp(2*j-1:2*j,:);

                  % vertical
                  J_z = J(2,:);
                  lambda_z = 1/(J_z * Ainv * J_z.');
                  Fz_inc = lambda_z*(vel_des(j) - J_z*qdot);
                  next_Fz_sum = min(max(0, Fz_sum(j)+Fz_inc), 100);
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
    %     iter
        end % if(num_cp > 0)
        
        q = q + qdot*h;

        
        % Pos  Update Iteration
        lambda_sum = zeros(2, num_check_pts);
        del_lambda = zeros(2, num_check_pts);
        C = zeros(2,num_check_pts);
        delta_q = zeros(dim/2,1);
        key_pts_ini = keypoints_biped([q; qdot]);

        for pos_iter = 1:5
            delta_q_sum = zeros(dim/2,1);

            for j = 1:num_check_pts
                key_pts_J = J_biped([q; qdot]);
                key_pts = keypoints_biped([q; qdot]);
                
                Jc  = key_pts_J(check_idx(j)*2-1:check_idx(j)*2,:);
                C(:,j) = (key_pts(:,check_idx(j)) - [key_pts_ini(1, check_idx(j)); 0]);
                ContactMassInv = Jc*Ainv*Jc.';
                
                del_lambda(:,j) = inv(ContactMassInv + alpha_tilde*eye(2))*(-C(:,j) - alpha_tilde*lambda_sum(:,j));
                lambda_sum_next = lambda_sum(:,j) + del_lambda(:,j);
                % normal force > 0
                lambda_sum_next(2) = max(lambda_sum_next(2), 0);
                % |tangential force| < mu * normal force
                lambda_sum_next(1) = max(-fric_coeff * lambda_sum_next(2), min(lambda_sum_next(1), fric_coeff * lambda_sum_next(2)));
                
                del_lambda(:,j) = lambda_sum_next - lambda_sum(:,j) ;

                delta_q = Ainv*Jc.'*del_lambda(:,j);
                
                lambda_sum(:,j) = lambda_sum(:,j) + del_lambda(:,j);
                q = q + delta_q;    
                delta_q_sum = delta_q_sum + delta_q;
            end
          
            if(norm(delta_q_sum) < 1.e-5)
                break;
            end
        end
%         pos_iter
        
        qdot = (q - q_pre)/h;
        q_pre = q;
    
    end
    
    z_next = [q; qdot];
end