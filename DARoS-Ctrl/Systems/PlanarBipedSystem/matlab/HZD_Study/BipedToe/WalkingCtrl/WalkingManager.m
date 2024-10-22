classdef WalkingManager < handle
    properties
        dtMPC = 0.05
        previous_time
        F_list
        OptTrj
        Nstep = 14;
        Mass
        Inertia
        ContactTB
        contact_table_idx
        heel_offset = -0.04;
        toe_back_offset = 0.015;
        toe_tip_offset = 0.055;
        swing_time_offset = 0.0;
        swing_duration
        stance_leg = 1; % 1: Right, 2:Left
        foot_ini
        first_visit = true;
        foot_traj = FootTrajGen;
    end   
    
    methods
        % Constructor
        function obj = WalkingManager(dtMPC, Mass, Inertia)
            obj.dtMPC = dtMPC;
            obj.previous_time = -dtMPC;
            obj.Mass = Mass;
            obj.Inertia = Inertia;
            
            % Contact table
            N = obj.Nstep;
            obj.contact_table_idx = N;
            obj.ContactTB = zeros(2, N);            
            obj.ContactTB(1,1:N/2) = ones(1, N/2);
            obj.ContactTB(2, N/2+1:N) = ones(1,N/2);
%             obj.ContactTB = ones(2, N);  % full contact standing (test)       

            obj.swing_duration = dtMPC * N/2;
        end
        
        function step_loc = landing_loc(obj, state, x_des)
            step_loc = [state(1) + state(4)*obj.swing_duration/2 - 0.5 * (x_des(4,1) - state(4));
                        -0.005];
        end
        
        % run MPC
        function [F_list, OptTrj, fpos_cmd, fvel_cmd, facc_cmd, stance_leg] = runWalking(obj, state, x_des, foot_pos, curr_time)
            run_opti = false;
            if(obj.first_visit)
                obj.foot_ini = foot_pos(:,obj.stance_leg);
                foot_fin = obj.landing_loc(state, x_des);
                obj.swing_time_offset = curr_time;
                obj.foot_traj.genTrajectory(obj.foot_ini, foot_fin, 0, obj.swing_duration);
                run_opti = true;
                
                obj.first_visit = false;
            end
            
            % Run Optimization if needed
            if (curr_time >= obj.previous_time + obj.dtMPC)                  
                run_opti = true;
                obj.previous_time = curr_time;
                
                % contact set
                obj.contact_table_idx = obj.contact_table_idx + 1;
                if(obj.contact_table_idx > obj.Nstep)
                    obj.contact_table_idx = 1;
                end
                TableNow = [obj.ContactTB(:,obj.contact_table_idx:end), obj.ContactTB(:,1:obj.contact_table_idx-1)];
                ct_offset = [obj.heel_offset, obj.toe_back_offset, obj.toe_tip_offset];                 
                
                
                % stance leg switch -----------------------
                if (obj.contact_table_idx == 1) % right stance, left swing start
                    obj.stance_leg = 1;
                    obj.swing_time_offset = curr_time;
                    obj.foot_ini = foot_pos(:,obj.stance_leg);
                end                
                if (obj.contact_table_idx == obj.Nstep/2+1) % left stance, right swing start
                    obj.stance_leg = 2;
                    obj.swing_time_offset = curr_time;
                    obj.foot_ini = foot_pos(:,obj.stance_leg);
                end
                % new swing trajectory generation
                foot_fin = obj.landing_loc(state, x_des);
                obj.foot_traj.genTrajectory(obj.foot_ini, foot_fin, 0, obj.swing_duration);                
            end
                      
            % Find Optimal
            if(run_opti)
                % ---------- Opti solver
                addpath(genpath('../../casadi')) 
                addpath(genpath('../casadi'))
                import casadi.*
                % -----------------------
                opti = casadi.Opti(); % Optimization problem                
                dim = length(state);
                dim_f = 12;
                N = obj.Nstep;
                % ---- decision variables ---------
                X = opti.variable(dim, N+1); % [x, y, theta, dx, dy, dtheta]
                F = opti.variable(dim_f, N);   % reaction force 
                                            % right: heel, toe_back, toe_tip vertical
                                            % left: heel, toe_back, toe_tip
                                            % right: horizontal
                                            % left: horizontal                                            
                % ---- objective          ---------
                Q = eye(dim);
                Q(2,2) = 10;
                Q_final = eye(dim)*10;
                Qu = eye(dim_f)*0.0001;
                mu = 0.6;
                
                cost = (X(:,end) - x_des(:,end)).'*Q_final*(X(:,end) - x_des(:,end));

                for i = 1:N
                    cost = cost + (x_des(:,i) - X(:,i)).'*Q*(x_des(:,i) - X(:,i));
                    cost = cost + F(:,i).'*Qu*F(:,i);
                end
                opti.minimize(cost); 

                for k=1:N % loop over control intervals
                    Xk  = X(:,k); 
                    Xk1 = X(:,k+1);
                    Fk  = F(:,k);
                    
                    F_lin = [ones(1,6)*Fk(7:12); ones(1,6)*Fk(1:6)];
                    F_tau = 0;
                    for i = 1:3 
                        % Right
                        F_tau = F_tau + (foot_pos(1,1) + ct_offset(i) - Xk(1))*Fk(i) - (foot_pos(2,1) - Xk(2))*Fk(i + 6);
                        % Left
                        F_tau = F_tau + (foot_pos(1,2) + ct_offset(i) - Xk(1))*Fk(i+3) - (foot_pos(2,2) - Xk(2))*Fk(i + 6 + 3);
                    end                   

                    opti.subject_to(Xk1(1:3) - Xk(1:3) == obj.dtMPC * Xk1(4:6) ) % Euler integration - position
                    opti.subject_to(diag([obj.Mass; obj.Mass])*(Xk1(4:5)-Xk(4:5)) == obj.dtMPC * (F_lin + [0; -9.81*obj.Mass]) ); 
                    opti.subject_to(obj.Inertia*(Xk1(6)-Xk(6)) == obj.dtMPC * F_tau ); 
                end

                % ---- path constraints -----------
                for k=1:N % loop over control intervals
                    Xk  = X(:,k); 
                    Fk  = F(:,k);

                    % Body ori
                    opti.subject_to( -pi/2 <= Xk(3) <= pi/2 ) 
                    
                    max_steplength = 0.5;
                    max_force = 900;
                    % step length
                    opti.subject_to(-max_steplength < foot_pos(1,1) - Xk(1) < max_steplength);
                    opti.subject_to(-max_steplength < foot_pos(1,2) - Xk(1) < max_steplength);

                    % reaction force
                    for leg = 1:2 % right:1, left:2
                        if(TableNow(leg, k) == 0)
                            opti.subject_to( Fk((leg-1)*3 + 1: (leg-1)*3 + 3) == zeros(3,1) )          % No reaction force
                        else
                            opti.subject_to( zeros(3,1) <= Fk((leg-1)*3 + 1: (leg-1)*3 + 3) <= ones(3,1)*max_force )
                        end
                    end

                    opti.subject_to( -mu*Fk(1:6) <= Fk(7:12) <= mu*Fk(1:6) ) 
                end

                % ---- boundary conditions --------
                opti.subject_to( X(:,1) == state );

                % ---- initial values for solver ---
                opti.set_initial(X, x_des);
                % opti.set_initial(X, ones(1, N+1).*x_initial );
                %opti.set_initial(U, );
                %opti.set_initial(F, );

                % ---- solve NLP              ------
                p_opts = struct('expand',true); % expand to casadi variables to SX (10x speedup)
                s_opts = struct('max_iter',1.e2, 'print_level', 0);
                opti.solver('ipopt',p_opts,s_opts);    % set numerical backend
                sol = opti.solve();
                
                obj.OptTrj = sol.value(X);
                F_list = sol.value(F);
%                 obj.OptTrj
%                 F_list
                for i = 1:N
                    obj.F_list{i} = [F_list(7:12,i).';
                                     F_list(1:6,i).'];
                end
            end
                        
            % Swing foot command update
            swing_time = curr_time - obj.swing_time_offset;
            [fpos_cmd, fvel_cmd, facc_cmd] = obj.foot_traj.getSplinePt(swing_time);
            stance_leg = obj.stance_leg;
            
            % Reaction force & State Trajectory
            F_list = obj.F_list;
            OptTrj = obj.OptTrj;
        end
    end
end

