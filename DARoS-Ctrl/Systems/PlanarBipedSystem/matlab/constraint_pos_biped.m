function z_next = constraint_pos_biped(z, dz, dt, rest_coeff, fric_coeff)
    dim = length(z);
    q = z(1:dim/2);
    qdot = z(dim/2+1:end);

    key_pts_ini = keypoints_biped([q; qdot]);
    key_pts = key_pts_ini;

    num_iter = 1;
    h = dt/num_iter;
    
    alpha = 1/100000;
    alpha_tilde = alpha/h^2;
    
    
    check_pt_idx = [4, 5, 8, 9];
    % select contact points that violate a constraint
    num_cp = 0;
    cp_idx = [];
    for i = 1:4
        idx = check_pt_idx(i);
%         if(key_pts(2,idx) < 0)
        if(true)
            num_cp = num_cp + 1;
            cp_idx(num_cp) = idx;
        end
    end
        
    A = A_biped([q; qdot]);
    Ainv = inv(A);
    
    
    q_pre = q;
    
    for main_iter = 1:num_iter
        q = q + qdot*h  + dz(dim/2+1:end)*h*h;

        
        % Pos  Update Iteration
        lambda_sum = zeros(2, num_cp);
        del_lambda = zeros(2, num_cp);
        C = zeros(2,num_cp);
        delta_q = zeros(dim/2,1);

        for pos_iter = 1:20
            delta_q_sum = zeros(dim/2,1);

            for j = 1:num_cp
                key_pts_J = J_biped([q; qdot]);
                key_pts = keypoints_biped([q; qdot]);
                
                Jc  = key_pts_J(cp_idx(j)*2-1:cp_idx(j)*2,:);
                C(:,j) = (key_pts(:,cp_idx(j)) - [key_pts_ini(1, cp_idx(j)); 0]);
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
        pos_iter
        
        qdot = (q - q_pre)/h;
        q_pre = q;
    
    end
    
    z_next = [q; qdot];
end