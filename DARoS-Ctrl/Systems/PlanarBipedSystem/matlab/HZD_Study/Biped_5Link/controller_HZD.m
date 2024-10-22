function [tau, jpos_des, jvel_des] = controller_HZD(state, alpha, phi_ini, phi_fin, right_stance, iter)
    % tau = controller_HZD(z, alpha, phi_ini, phi_fin)
    % This function computes the control input for the biped robot using the
    % Hybrid Zero Dynamics (HZD) controller.
    % 
    % Inputs:
    % z: 1x14 vector representing the state of the robot
    % alpha: 4x6 matrix representing the coefficients of the HZD controller
    % phi_ini: scalar representing the initial value of the phase variable
    % phi_fin: scalar representing the final value of the phase variable
    % 
    % Outputs:
    % tau: 6x1 vector representing the control input for the robot
    
    % Extract the state variables
    q = state(1:7);
    dq = state(8:14);
    contact_idx = 3; % 3: right, 5: left
    % Compute the phase variable
    if(right_stance)
        phi = pi/2 - (q(3) + q(4) + q(5)); 
        phi_dot = -dq(3) - dq(4) - dq(5);
        
        % Test
        phi = iter * 0.001/0.23 * (phi_fin - phi_ini) + phi_ini;
        phi_dot = 0;
    else
        phi = pi/2 - (q(3) + q(6) + q(7));
        phi_dot = -dq(3) - dq(6) - dq(7);
        contact_idx = 5;
        
        % Test
        phi = iter * 0.001/0.23 * (phi_fin - phi_ini) + phi_ini;
        phi_dot = 0;
    end

    s = (phi - phi_ini)/(phi_fin - phi_ini);
    sdot = (phi_dot)/(phi_fin - phi_ini);
    s = min(max(0, s),1);
    jpos_des = zeros(4,1);
    jvel_des = zeros(4,1);
    % Compute the control input
    for i=1:2
        if(right_stance)
            [jpos_des(i), jvel_des(i)] = fn_bezier_pt(alpha(i,:), s, sdot);
            [jpos_des(i+2), jvel_des(i+2)] = fn_bezier_pt(alpha(i+2,:), s, sdot);
        else
            [jpos_des(i), jvel_des(i)] = fn_bezier_pt(alpha(i+2,:), s, sdot);
            [jpos_des(i+2), jvel_des(i+2)] = fn_bezier_pt(alpha(i,:), s, sdot);
        end
    end
    
    tau = wbc_jpos_ctrl(state, jpos_des, jvel_des, contact_idx);
%     tau = zeros(7,1);
%     for i = 1:4 
%         tau(3+i) = 150*(jpos_des(i) - q(3+i)) + 0.5*(0-dq(3+i));
%     end
end