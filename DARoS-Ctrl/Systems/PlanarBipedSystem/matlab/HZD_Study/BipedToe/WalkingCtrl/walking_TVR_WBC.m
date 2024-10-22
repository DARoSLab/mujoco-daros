classdef walking_TVR_WBC < handle
properties
    swing_time = 0.5;
    step_timeoffset = -10;
    vel_cmd = 0.5;
    height = 0.0;
    jpos_des;
    
    foot_trj = FootTrajGen;
    lipm_trj;
    stance_leg = 0; % 1: right, 0:left
    swing_foot_idx = 9; % 3: r_ankle, 9: l_ankle
    
    body_des;
    body_vel;    
end

methods
    function obj = walking_TVR_WBC(ini_state)
        obj.height = ini_state(2);
        obj.lipm_trj = LIPM_Traj(obj.height);
        len = length(ini_state)/2 - 3;
%         obj.jpos_des = zeros(len,1);
        obj.jpos_des = ini_state(4:3+len);
    end
    
    function tau = getJointCMD(obj, state, curr_time)
        seq_time = (curr_time - obj.step_timeoffset);
        
        % End of swing
        if(seq_time >= obj.swing_time)
            obj.step_timeoffset = curr_time;
            seq_time = (curr_time - obj.step_timeoffset);
            obj.stance_leg = ~obj.stance_leg;
            
            key_pts = keypoints_biped(state);
            
            if(obj.stance_leg) % right
                obj.swing_foot_idx = 9;
                swing_foot_pos = key_pts(:,obj.swing_foot_idx);
                stance_loc = key_pts(:, 3);  
                
            else
                obj.swing_foot_idx = 3;
                swing_foot_pos = key_pts(:,obj.swing_foot_idx);
                stance_loc = key_pts(:, 9);
            end
            obj.lipm_trj.updateState([state(1); state(12)], stance_loc(1));
            
            landing_loc = obj.lipm_trj.computeLandingLoc(obj.vel_cmd, obj.swing_time);
            landing_loc_vec = [landing_loc; stance_loc(2)];
            obj.foot_trj.genTrajectory(swing_foot_pos, landing_loc_vec, 0, obj.swing_time);
        end
        
        if(obj.stance_leg) % right
            contact_idx = [4, 6, 7];
        else % left
            contact_idx = [10, 12, 13];
        end

        pend_state = obj.lipm_trj.getPt(seq_time);
        obj.body_des = [pend_state(1); obj.height; -0.05]; % [x, y, theta_body]
        obj.body_vel = [pend_state(2); 0; 0];       
        
        [f_pos, f_vel, f_acc] = obj.foot_trj.getSplinePt(seq_time);
        
        % TEST
%         body_des = [0; obj.height + 0.02*sin(4*pi*curr_time); -0.05]; % [x, y, theta_body]
%         body_vel = [0; 0; 0];


        tau = wbc_walking_ctrl(state, obj.body_des, obj.body_vel, f_pos,f_vel,f_acc, ...
                               obj.jpos_des, obj.swing_foot_idx, contact_idx);
        
        

    end
end
end