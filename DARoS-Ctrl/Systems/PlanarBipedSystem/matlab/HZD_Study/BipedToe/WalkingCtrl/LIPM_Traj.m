classdef LIPM_Traj < handle
properties
    omega = 0;
    state_ini = [-0.1; 0.4];
    stance_loc = 0.01;
end

methods
    function obj = LIPM_Traj(height)
        obj.omega = sqrt(9.81/height);
    end
    
    function ret = updateState(obj, state_ini, stance_loc)
        obj.state_ini = state_ini;
        obj.stance_loc = stance_loc;
        ret = true;
    end
    
    function loc = computeLandingLoc(obj, vel_cmd, swing_time)
        omg_t = obj.omega * swing_time;
        omg = obj.omega;
        A = [cosh(omg_t), 1/omg * sinh(omg_t);
             omg * sinh(omg_t), cosh(omg_t)];
        B = [1-cosh(omg_t);
             -omg * sinh(omg_t)];
         
        nx_state = A*obj.state_ini + B*obj.stance_loc;
        nx_state(2) = nx_state(2)*0.85;
        % next step
        omg_t_half = obj.omega * swing_time/2;

        a11 = cosh(omg_t_half);
        a12 = 1/omg * sinh(omg_t_half);
        a21 = omg * sinh(omg_t_half);
        a22 = cosh(omg_t_half);
        
        b1 = 1-cosh(omg_t_half);
        b2 = -omg * sinh(omg_t_half);
        
        loc = (a11*nx_state(1) + a12*nx_state(2))/(1-b1);
        
        vel_min = a21*nx_state(1) + a22*nx_state(2) + b2 * loc;
        
        loc = loc + (vel_min - vel_cmd)*0.1;
    end
    
    function pend_state = getPt(obj, curr_time)
        omg_t = obj.omega * curr_time;
        omg = obj.omega;
        A = [cosh(omg_t), 1/omg * sinh(omg_t);
             omg * sinh(omg_t), cosh(omg_t)];
        B = [1-cosh(omg_t);
             -omg * sinh(omg_t)];
         
        pend_state = A*obj.state_ini + B*obj.stance_loc;
    end
end
end