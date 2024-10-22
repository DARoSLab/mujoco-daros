clear; clc;
timers.SymbolicPRMPC_timer = tic;

% Choose which of the sections to calculate
options.RUN_ALL = false;
options.PARAMS = false|options.RUN_ALL;
options.INITIALIZATION = false|options.RUN_ALL;
options.HEURISTICS = true|options.RUN_ALL;
options.COST = false|options.RUN_ALL;
options.CONSTRAINTS = false|options.RUN_ALL;
options.LAGRANGIAN_HESSIAN = false|options.RUN_ALL;

% Choose which of the sections to load
options.LOAD_ALL = true;
options.LOAD_INITIALIZATION = trues|options.LOAD_ALL;
options.LOAD_HEURISTICS = true|options.LOAD_ALL;
options.LOAD_COST = true|options.LAGRANGIAN_HESSIAN|options.LOAD_ALL;
options.LOAD_CONSTRAINTS = true|options.LAGRANGIAN_HESSIAN|options.LOAD_ALL;
options.LOAD_LAGRANGIAN_HESSIAN = true|options.LOAD_ALL;

% Choose which of the sections to generate
options.GEN_FUNCTIONS = false;
options.GEN_ALL = true;
options.GEN_INITIALIZATION = true|options.GEN_ALL;
options.GEN_HEURISTICS = false|options.GEN_ALL;
options.GEN_COST = false|options.GEN_ALL;
options.GEN_CONSTRAINTS = false|options.GEN_ALL;
options.GEN_LAGRANGIAN_HESSIAN = false|options.GEN_ALL;

%% General RPC Parameters
if options.PARAMS
    timers.params_timer = tic;
    fprintf('General symbolic variable setup...\n')
    
    % Number of robot legs
    NUM_FEET = 4;
    
    % Problem Parameters
    NUM_STATES = 12;
    NUM_INPUTS = 6*NUM_FEET;
    NUM_DECISION_VARS = NUM_STATES + NUM_INPUTS;
    % {Dynamics} + {Foot on Ground} + {No Slip} + {Friction Pyramid} %+ {Kinematic Limits}
    NUM_CONSTRAINTS = NUM_STATES + NUM_FEET + 2*NUM_FEET + 4*NUM_FEET; %+ NUM_FEET
    NUM_HEURISTICS_MAX = 10;
    
    % Number of timesteps
    K = 1;
    
    % Set the infinity value
    INF = 2e19;
    
    % Projection matrix to the ground
    P = [1,0,0;0,1,0];
    
    % State vectors
    x = sym('x_%d', [NUM_STATES, K], 'real');   % states at k
    x0 = sym('x0_%d', [NUM_STATES, K], 'real');  % states at k-1
    x1 = sym('x1_%d', [NUM_STATES, K], 'real');  % states at k+1
    xt = sym('xt_%d', [NUM_STATES, K], 'real');  % states at t
    
    % Input vectors
    r = sym('r_%d_%d', [NUM_FEET, 3], 'real');  % footsteps at k
    f = sym('f_%d_%d', [NUM_FEET, 3], 'real');  % forces at k
    u = [r, f]'; u = repmat(u(:),1,K);          % inputs at k
    
    r0 = sym('f0_%d_%d', [NUM_FEET, 3], 'real');  % footsteps at k-1
    f0 = sym('r0_%d_%d', [NUM_FEET, 3], 'real');  % forces at k-1
    u0 = [r0, f0]'; u0 = repmat(u0(:),1,K);       % inputs at k-1
    
    r1 = sym('r1_%d_%d', [NUM_FEET, 3], 'real');  % footsteps at k+1
    f1 = sym('f1_%d_%d', [NUM_FEET, 3], 'real');  % forces at k+1
    u1 = [r1, f1]'; u1 = repmat(u1(:),K);         % inputs at k+1
    
    rt = sym('rt_%d_%d', [NUM_FEET, 3], 'real');  % footsteps at t
    ft = sym('ft_%d_%d', [NUM_FEET, 3], 'real');  % forces at t
    ut = [rt, ft]'; ut = repmat(ut(:),1,K);          % inputs at t
    
    % Decision Variables
    X = [x; u];     % decision variables at k
    X0 = [x0; u0];  % decision variables at k-1
    X1 = [x1; u1];  % decision variables at k+1
    
    % Contacts state for the feet
    c_states = sym('c_%d', [NUM_FEET, K], 'real');  % contact states at k
    c_states0 = sym('c0_%d', [NUM_FEET, K], 'real');  % contact states at k-1
    c_states1 = sym('c1_%d', [NUM_FEET, K], 'real');  % contact states at k+1
    c_statest = sym('ct_%d', [NUM_FEET, K], 'real');  % contact states at t
    
    % Gait phases for the feet
    s_Phi = sym('s_Phi_%d', [NUM_FEET, K], 'real');  % contact states at k
    
    % Maximum and minimum states
    x_max = sym('x_max_%d', [NUM_STATES, K], 'real');
    x_min = sym('x_min_%d', [NUM_STATES, K], 'real');
    
    % Maximum and minimum inputs
    u_max = sym('u_max_%d', [NUM_INPUTS, K], 'real');
    u_min = sym('u_min_%d', [NUM_INPUTS, K], 'real');
    
    % Reference trajectories
    x_input = sym('x_input_%d', [NUM_STATES, K], 'real');  % states reference
    x_ref = sym('x_ref_%d', [NUM_STATES, K], 'real');  % states reference
    u_ref = sym('u_ref_%d', [NUM_INPUTS, K], 'real');  % inputs reference
    X_ref = [x_ref; u_ref];                            % decision variables reference
    
    % Heuristic Regularization
    Hx = sym('Hx_%d', [NUM_STATES, K], 'real');         % states heuristics
    Hr = sym('Hr_%d_%d', [NUM_FEET, 3], 'real');         % inputs reference
    Hf = sym('Hf_%d_%d', [NUM_FEET, 3], 'real');         % inputs reference
    Hu = [Hr, Hf]'; Hu = repmat(Hu(:),1,K);          % inputs at t
    HX = [Hx; Hu];
    
    % Heuristic Regularization gains
    K_Hx = sym('K_Hx_%d', [NUM_HEURISTICS_MAX, K], 'real');  % states heuristics
    K_Hr = sym('K_Hr_%d', [NUM_HEURISTICS_MAX, K], 'real');  % inputs heuristics
    K_Hf = sym('K_Hf_%d', [NUM_HEURISTICS_MAX, K], 'real');  % inputs heuristics
    K_Hu = [K_Hr; K_Hf];                                     % inputs heuristics
    K_HX = [K_Hx; K_Hu];                                     % decision variables heuristics
    
    % Cost Weights
    Q = sym('q%d', [NUM_STATES, K], 'real');  % state weights
    R = sym('r%d', [NUM_INPUTS, K], 'real');  % input weights
    W = [diag(Q), zeros(NUM_STATES,NUM_INPUTS);...         % decision variable weights
        zeros(NUM_INPUTS,NUM_STATES), diag(R)];
    
    % Environment
    g = sym('g_%d', [3, 1], 'real');               % Gravity vector
    mu_g = sym('mu_g_%d', [NUM_FEET, K], 'real');  % friction at each foot
    z_g = sym('z_g_%d', [NUM_FEET, K], 'real');    % Ground height at each foot
    
    % General cyclic gait parameters
    syms dt stance_fraction flight_phase real
    
    % Initial foot location
    p_foot_touchdown = sym('p_foot_td_%d_%d', [NUM_FEET,3], 'real')';
    
    % Stance time at current prediction
    T_s = sym('T_s_%d', [NUM_FEET,1], 'real');
    
    % Boolean for each leg flagging touchdown event steps
    touchdown = sym('td_%d', [NUM_FEET,1], 'real')';
    
    % CoM state at which a touchdown occurs for each leg
    x_touchdown = sym('x_td_%d_%d', [NUM_FEET,3], 'real')';
    
    % Robot physical parameters
    m = sym('m', 'real');
    I = sym('I_%d', [3, 1], 'real');
    I_tensor = [m*eye(3), zeros(3); zeros(3), diag(I)];
    I_tensor_inv = I_tensor^-1;
    
    % Hip locations relative to CoM in body frame
    r_hip = sym('r_hip_%d_%d', [NUM_FEET,3], 'real')';
    
    % Robot linearized system
    System.A = [eye(6), dt*eye(6); zeros(6), eye(6)];
    System.B = [(dt^2)/2*I_tensor_inv; dt*I_tensor_inv];
    System.C = eye(12);
    System.D = zeros(12,6);
    System.Dw = [((dt)^2)/2*eye(6); eye(6)*dt];
    System.w = [g;zeros(3,1)];
    
    % Sparsity pattern parameters
    syms iter NUM_X NUM_C real
    
    % Objective factor for cost Hessian
    syms obj_factor real
    
    % Lagrange multipliers on the constraint hessian
    lambda = sym('lambda%d', [NUM_CONSTRAINTS,K], 'real');
    
    % Save the symbolic variables from the workspace
    save('Data/RPC_params.mat',  '-regexp', '^(?!(options|timers)$).');
    
    % Notify end of setup
    fprintf('DONE general symbolic variable setup: %f s\n\n',toc(timers.params_timer));
elseif options.LOAD_INITIALIZATION
    timers.params_timer = tic;
    fprintf('Loading general symbolic variable setup...\n')
    load('Data/RPC_params.mat')
    fprintf('DONE loading general symbolic variable setup: %f s\n\n',toc(timers.params_timer));
else
end


%% Initialization
if options.INITIALIZATION
    % Start the timer for calculating initialization
    timers.initialization_timer = tic;
    fprintf('Calculating Initialization...\n')
    
    % Initialize reference foot locations
    p_foot_ref = sym(zeros(3*NUM_FEET,K));
    x_input_mod = sym(zeros(NUM_STATES,K));
    x_ref_initial = sym(zeros(NUM_STATES,K));
    u_ref_initial = sym(zeros(3*NUM_FEET,K));
    constraints_ub = sym(zeros(NUM_CONSTRAINTS,K));
    constraints_lb = sym(zeros(NUM_CONSTRAINTS,K));
    
    x0 = xt;
    
    % Timestep k loop
    for k = 1:K
        
        %%% STATE BOUNDS %%%
        state_ub = [x_max(1:2) + x_input(1:2); x_max(3:12)];  % states upper bound
        state_lb = [x_min(1:2) + x_input(1:2); x_min(3:12)];  % states lower bound
        
        % Rotation matrix at k
        rpy = [1;1;1].*x(4:6,k);
        RotM = simplify(RPYToR(rpy));
        
        % Yaw Rotation matrix at k
        yaw = [0;0;1].*x(4:6,k);
        RotYaw = simplify(RPYToR(yaw));
        
        % Sum of the number of feet in stance at current phase
        num_stance_feet = sum(c_states);
        
        % Normalized gravity
        norm_g = simplify(norm(g));
        
        % Normalized velocity
        norm_vel_d = simplify(norm(x_input(7:8,k)));
        
        % Initial reference states
        %         x_ref_initial = x_ref + Hx;
        x_input_mod = [x_input(1:6,k);RotYaw*x_input(7:9,k);x_input(10:NUM_STATES,k)];
        %x_input_mod(1:2,k) = x_input_mod(1:2,k) + x_input_mod(7:8,k)*dt;
        %x_input_mod(6,k) = x_input_mod(6,k) + x_input_mod(12,k)*dt;
        x_ref_initial = x_input_mod + Hx;
        
        % Loop through the feet
        for foot = 1:NUM_FEET
            % First foot input index
            i_u = NUM_INPUTS/NUM_FEET*(foot - 1);
            i_f = 3*(foot - 1);
            
            % Reference footstep location in world orientation frame offset to robot CoM
            % p_touchdown_ref = P'*P*(Hr(foot,:)');
            p_touchdown_ref = Hr(foot,:)';
            
            % Determine if foot location needs to be updated
            p_foot_ref(1+i_f:3+i_f,:) = touchdown(1,foot)*p_touchdown_ref...  % touchdown phase, use heuristic reference
                + (1 - touchdown(1,foot))*p_foot_touchdown(1:3,foot);  % not a touchdown phase, use prev touchdown position
            
            % Find distance CoM has travelled since last foot touchdown
            travel_distance = (x(1:3,k) + x(7:9,k)*dt/2) - x_touchdown(:,foot);
            
            % Footsetp vector, r_foot,
            u_ref_initial(1+i_u:3+i_u,:) = c_states(foot,k)*(p_foot_ref(1+i_f:3+i_f,:) - ...
                [travel_distance(1:2);x(3)]);% add in some weighted x0 + xd blending?
            
            % Forces in World frame
            f_foot_ref = Hf(foot,:)';
            
            % Forces vector, f_foot
            u_ref_initial(4+i_u:6+i_u,:) = c_states(foot,k)*f_foot_ref;
            
            %%% INPUT BOUNDS %%%
            % Foot vector input upper bound
            input_ub(1+i_u:3+i_u,:) = c_states(foot,k)*(RotYaw*r_hip(:,foot) + u_max(1+i_u:3+i_u,:));
            
            % Foot vector input lower bound
            input_lb(1+i_u:3+i_u,:) = c_states(foot,k)*(RotYaw*r_hip(:,foot) + u_min(1+i_u:3+i_u,:));
            
            % Ground reaction force input upper bound
            input_ub(4+i_u:6+i_u,:) = c_states(foot,k)*u_max(4+i_u:6+i_u,:);
            
            % Ground reaction force input lower bound
            input_lb(4+i_u:6+i_u,:) = c_states(foot,k)*u_min(4+i_u:6+i_u,:);
            
        end
        %%% INITIAL STATES %%%
        % Convert GRF and footstep locations to momentum rate of change
        h_dot = NonlinearInput(x(:,k), u_ref_initial(:,k), c_states(:,k), NUM_FEET);
        
        % Compute simplified discrete dynamics
        y = DiscreteDynamics(x(:,k), h_dot, System);
        
        %%% CONSTRAINT BOUNDS %%%
        % Constraint index
        i_c = NUM_CONSTRAINTS*(k - 1);
        
        % Dynamics, equality constraint {NUM_STATES}
        constraints_ub(i_c+1:i_c+NUM_STATES,k) = zeros(NUM_STATES,1);
        constraints_lb(i_c+1:i_c+NUM_STATES,k) = zeros(NUM_STATES,1);
        i_c = i_c + NUM_STATES;
        
        % Foot on ground, equality constraint {NUM_FEET}
        constraints_ub(i_c+1:i_c+NUM_FEET,k) = zeros(NUM_FEET,1);
        constraints_lb(i_c+1:i_c+NUM_FEET,k) = zeros(NUM_FEET,1);
        i_c = i_c + NUM_FEET;
        
        % No slip, equality constraint {2*NUM_FEET}
        constraints_ub(i_c+1:i_c+2*NUM_FEET,k) = zeros(2*NUM_FEET,1);
        constraints_lb(i_c+1:i_c+2*NUM_FEET,k) = zeros(2*NUM_FEET,1);
        i_c = i_c + 2*NUM_FEET;
        
        % Friction pyramids, inequality constraint {4*NUM_FEET}
        constraints_ub(i_c+1:i_c+4*NUM_FEET,k) = zeros(4*NUM_FEET,1);
        constraints_lb(i_c+1:i_c+4*NUM_FEET,k) = -INF*ones(4*NUM_FEET,1);
        i_c = i_c + 4*NUM_FEET;
        
    end
    
    % Initial reference inputs
    u_ref_initial = simplify(u_ref_initial);
    
    % Initial decision variables
    X_initial = [simplify(y); u_ref_initial]; %simplify([y; u_ref_initial]);
    
    % Reference footstep locations relative to CoM
    p_foot_ref = simplify(p_foot_ref);
    
    % Decision variable bounds
    X_lb = simplify([state_lb;input_lb]);
    X_ub = simplify([state_ub;input_ub]);
    
    % Save the symbolic variables from the workspace
    save('Data/RPC_initialization.mat','X_initial','x_ref_initial','u_ref_initial','p_foot_ref');
    save('Data/RPC_bounds.mat','X_ub','X_lb','constraints_ub','constraints_lb');
    
    % Print timing statistics
    fprintf('DONE calculating initialization: %f s\n\n',toc(timers.initialization_timer));
elseif options.LOAD_INITIALIZATION
    timers.initialization_timer = tic;
    fprintf('Loading Initialization...\n')
    load('Data/RPC_initialization.mat');
    load('Data/RPC_bounds.mat');
    fprintf('DONE loading initialization: %f s\n\n',toc(timers.initialization_timer));
else
    % Do not load or calculate initialization
end


%%
if options.HEURISTICS
    timers.heuristics_timer = tic;
    fprintf('Calculating Heuristics...\n')
    x_input_rot = sym(zeros(NUM_STATES,K));
    H_X = sym(zeros(NUM_DECISION_VARS,K));
    H_x = sym(zeros(NUM_STATES,K));
    
    for k = 1:K
        % Rotation matrix at k
        rpy = [1;1;1].*x(4:6,1);
        RotM = simplify(RPYToR(rpy));
        
        % Yaw Rotation matrix at k
        yaw = [0;0;1].*x(4:6,1);
        RotYaw = simplify(RPYToR(yaw));
        
        % Yaw des Rotation matrix at k
        yaw_input = [0;0;1].*x_input(4:6,k);
        RotYaw_input = simplify(RPYToR(yaw_input));
        
        % Rotate the velocities		
	    x_input_rot(7:9,k) = RotYaw*x_input(7:9,k);
        
        % Sum of the number of feet in stance at current phase
        num_stance_feet = sum(c_states);
        
        % Normalized gravity
        norm_g = simplify(norm(g));
        
        % Normalized velocity
        norm_vel_d = simplify(norm(x_input(7:8,k)));
        
        %%% HEURISTIC REFERENCE FOR STATES %%%
        % Linear lateral velocity induced roll compensation
        H_theta_1 = K_Hx(1)*x_input(8,k) + K_Hx(2);
        H_x(4,k) = H_x(4,k) + H_theta_1;
        
        % Linear forward velocity induced pitch compensation
        H_phi_1 = K_Hx(3)*x_input(7,k) + K_Hx(4);
        H_x(5,k) = H_x(5,k) + H_phi_1;
        
        % Pitch periodic gait limit cycle
        H_phi_2 = K_Hx(5)*sin(2*pi*s_Phi(1) + pi);
        H_x(5,k) = H_x(5,k) + H_phi_2;
        
        % Linear forward velocity induced height compensation
        H_z_1 = K_Hx(6)*x_input(7,k) + K_Hx(7);
        H_x(3,k) = H_x(3,k) + H_z_1;
        
        % Position based on dynamics
        H_xy_1 = 0*x(1:2,k);
        H_x(1:2,k) = H_x(1:2,k) + H_xy_1;
        
        % Yaw based on dynamics
        H_psi_1 = 0*x(6,k);
        H_x(6,k) = H_x(6,k) + H_psi_1;
        
        % Set the state heuristics
        H_X(1:NUM_STATES,1) = H_x;
        
        for foot = 1:NUM_FEET
            i_r = NUM_STATES + 6*(foot - 1);
            i_f = i_r + 3;
            
            % Footsetp heuristics in world frame
            H_r_foot = [RotYaw*r_hip(1:3,foot),...                                            % hip location from COM in World Frame
                T_s(foot,k)/2*RotYaw_input*[x_input(7:8,k);0],...                                           % forward stepping heuristic
                T_s(foot,k)/2*CrossProd([0;0;x_input(12,k)])*(RotYaw*r_hip(1:3,foot)),...  % turning in place heuristic
                sqrt(x(3)/norm_g)*(x(7:9,k) - RotYaw_input*x_input(7:9,k)),...                          % capture point heuristic
                x(3,k)/norm_g*CrossProd(RotYaw_input*[x_input(7:8,k);0])*[0;0;x_input(12,k)]];            % High Speed Turning
            H_X(i_r+1:i_r+3) = P'*P*(H_r_foot*K_Hr(1:size(H_r_foot,2),k));
            
            % Ground reaction force heuristics
            H_f_foot = [-stance_fraction*m*g/(flight_phase + (1-flight_phase)*num_stance_feet),...  % vertical impulse scaling
                -m*x_input(12,k)*norm_vel_d*[sin(x_input(6,k));-cos(x_input(6,k));0]];               % centripetal forces
            H_X(i_f+1:i_f+3) = H_f_foot*K_Hf(1:size(H_f_foot,2),k);
        end
    end
    H_X = simplify(H_X);
    
    % Save the symbolic variables from the workspace
    save('Data/RPC_heuristics.mat','H_X');
    
    % Print timing statistics
    fprintf('DONE calculating heuristics: %f s\n\n',toc(timers.heuristics_timer));
    
elseif options.LOAD_HEURISTICS
    timers.heuristics_timer = tic;
    fprintf('Loading Heuristics...\n')
    load('Data/RPC_heuristics.mat');
    fprintf('DONE loading heuristics: %f s\n\n',toc(timers.heuristics_timer));
else
end


%% Cost, Gradient, and Cost Hessian
if options.COST
    % Start the timer for calculating cost
    timers.cost_timer = tic;
    fprintf('Calculating Cost...\n')
    
    % Initialize cost
    cost = 0;
    
    for k = 1:K
        % Convert GRF to momentum rate of change
        h_dot = NonlinearInput(x(:,k), u(:,k), c_states(:,k), NUM_FEET);
        
        % Compute simplified discrete dynamics
        y = DiscreteDynamics(x(:,k), h_dot, System);
        
        % State trajectory tracking error
        % x_error = (x_d - y);       % dynamics in the cost
        x_error = (x_ref - x(:,k));  % no dynamics in cost
        
        % Policy reference regularization
        u_error = (u_ref - u(:,k));
        
        % Objective cost function
        cost = cost + x_error'*diag(Q)*x_error + u_error'*diag(R)*u_error;
    end
    
    % Cost
    J = simplify(cost);
    
    % Gradient
    fprintf('Calculating Cost Gradient...\n')
    cost_gradient = simplify(jacobian(J, [x;u])');
    
    % Cost Hessian
    fprintf('Calculating Cost Hessian...\n')
    cost_hessian = simplify(obj_factor*jacobian(cost_gradient, [x;u]));
    
    % Save the cost results
    save('Data/RPC_cost.mat','J','cost_gradient','cost_hessian');
    
    % Print timing statistics
    fprintf('DONE calculating cost function: %f s\n\n',toc(timers.cost_timer));
    
elseif options.LOAD_COST
    % Start the timer for loading cost
    timers.cost_timer = tic;
    fprintf('Loading Cost...\n')
    load('Data/RPC_cost.mat');
    fprintf('DONE loading cost function: %f s\n\n',toc(timers.cost_timer));
else
end


%% Constraints
if options.CONSTRAINTS
    % Start the timer for calculating constraints
    timers.constraint_timer = tic;
    fprintf('Calculating Constraints...\n')
    
    % Initialize iteration constraint vector
    constraints = sym(zeros(NUM_CONSTRAINTS, K));
    
    for k = 1:K
        
        % Constraint index
        i_c = NUM_CONSTRAINTS*(k - 1);
        
        % Convert GRF to momentum rate of change
        h_dot = NonlinearInput(x(:,k), u(:,k), c_states(:,k), NUM_FEET);
        
        % Compute simplified discrete dynamics
        y = DiscreteDynamics(x(:,k), h_dot, System);
        
        % Dynamics, x_{k+1} - f(x_k, u_k) = 0 {NUM_STATES}
        constraints(i_c+1:i_c+NUM_STATES,k) = x1(:,k) - y;
        i_c = i_c + NUM_STATES;
        
        % CoM postion in future step
        pos_com1 = x1(1:3,k);
        
        % Add the constraints for each foot
        for foot = 1:NUM_FEET
            % Reset constraint index
            i_c = NUM_CONSTRAINTS*(k - 1) + NUM_STATES;
            
            % Leg state number
            i_f = NUM_INPUTS/NUM_FEET*(foot - 1);
            
            % Foot position vector from CoM (CURRENTLY ONLY FOR FLAT GROUND)
            f_foot = u(4+i_f:6+i_f,k);
            
            % Foot position in the world: p_foot_{k} = p_com_{k} + r_foot_{k}
            p_foot = (x(1:3,k) + u(1+i_f:3+i_f,k));
            
            % Foot position in the world future: p_foot_{k+1} = p_com_{k+!} + r_foot_{k+1}
            p_foot1 = (x1(1:3,k) + u1(1+i_f:3+i_f,k));
            
            %% Input constraints
            % Foot on ground {NUM_FEET}
            i_ground = i_c + (foot - 1);
            constraints(i_ground+1,k) = ...
                c_states(foot)*(z_g(foot,k) - p_foot(3,k));
            i_c = i_c + NUM_FEET;
            
            % No slip {2*NUM_FEET}
            i_slip = i_c + 2*(foot - 1);
            constraints(i_slip+1:i_slip+2,k) = ...
                c_states1(foot,k)*c_states(foot,k)*(p_foot1(1:2,1) - p_foot(1:2,1));
            i_c = i_c + 2*NUM_FEET;
            
            % Friction pyramids {4*NUM_FEET}
            i_friction = i_c + 4*(foot - 1);
            constraints(i_friction+1:i_friction+4,k) = ...
                c_states(foot,k)*[f_foot(1,k) - mu_g(foot,k)*f_foot(3,k);...
                -f_foot(1,k) - mu_g(foot,k)*f_foot(3,k);...
                f_foot(2,k) - mu_g(foot,k)*f_foot(3,k);...
                -f_foot(2,k) - mu_g(foot,k)*f_foot(3,k)];
        end
    end
    
    % Simplify the symbolic constraints
    constraints = simplify(constraints);
    
    % Calculate the constraint jacobian
    %    rows: constraints
    %    columns: decision variables
    fprintf('Calculating Constraint Jacobian...\n')
    constraint_jacobian = simplify(jacobian(constraints,[x;u;x1;u1]));
    
    % Non zero entries in the constraint jacobian
    constraint_jacobian_nz = constraint_jacobian(constraint_jacobian~=0);
    
    % Calculate the constraint jacobian sparsity pattern
    fprintf('Calculating Constraint Jacobian Sparsity Pattern...\n')
    [row_index_CJ, col_index_CJ] = find(constraint_jacobian);
    
    % Shift indices for C++
    row_index_CJ = row_index_CJ - 1;
    col_index_CJ = col_index_CJ - 1;
    
    % Add the iteration index modifier
    row_index_CJ = row_index_CJ + ones(size(row_index_CJ,1),1)*iter*NUM_C;
    col_index_CJ = col_index_CJ + ones(size(col_index_CJ,1),1)*iter*NUM_X;
    
    % Calculate the constraint hessian
    fprintf('Calculating Constraint Hessian...\n')
    constraint_hessian = zeros(2*NUM_DECISION_VARS:2*NUM_DECISION_VARS);
    
    % Sum of the hessian of each constraint with Lagrange multiplier
    for c = 1:NUM_CONSTRAINTS
        constraint_hessian = constraint_hessian + ...
            lambda(c)*jacobian(constraint_jacobian(c,:),[x;u;x1;u1]);
    end
    
    % Non zeros only in current timestep entries
    constraint_hessian = simplify(constraint_hessian(1:NUM_DECISION_VARS,1:NUM_DECISION_VARS));
    
    %% Final constraints
    % Since the initial timestep does not include future dynamics or foot
    % locations, it needs to be treated as a special case where only the
    % instantaneous forces and foot vectors are constrained.
    % Needed if constraints are x_{k} -> x_{k+1}
    fprintf('Calculating Final Constraints...\n')
    
    % Final constraints do not include next iteration decision variables
    constraints_final = constraints;
    
    % Final dynamics are not constrained
    constraints_final(1:NUM_STATES,k) = zeros(NUM_STATES,1);
    
    % Final no slip condition is not constrained
    i_slip = NUM_STATES + NUM_FEET;
    constraints_final(i_slip + 1:i_slip + 2*NUM_FEET,k) = ...
        zeros(2*NUM_FEET,1);
    
    % Final iteration constraint jacobian (no future decision variables)
    fprintf('Calculating Final Constraint Jacobian...\n')
    constraint_jacobian_final = simplify(jacobian(constraints_final,[x;u]));
    
    % Non zero entries in the initial constraint jacobian
    constraint_jacobian_final_nz = ...
        constraint_jacobian_final(constraint_jacobian_final~=0);
    
    % Calculate the initial constraint jacobian sparsity pattern
    fprintf('Calculating Final Constraint Jacobian Sparsity Pattern...\n')
    [row_index_final_CJ, col_index_final_CJ] = ...
        find(constraint_jacobian_final);
    
    % Shift indices for C++
    row_index_final_CJ = row_index_final_CJ - 1;
    col_index_final_CJ = col_index_final_CJ - 1;
    
    % Add the iteration index modifier
    row_index_final_CJ = row_index_final_CJ + ...
        ones(size(row_index_final_CJ,1),1)*iter*NUM_C;
    col_index_final_CJ = col_index_final_CJ + ...
        ones(size(col_index_final_CJ,1),1)*iter*NUM_X;
    
    % Calculate the initial constraint hessian
    fprintf('Calculating Final Constraint Hessian...\n')
    constraint_hessian_final = zeros(NUM_DECISION_VARS:NUM_DECISION_VARS);
    
    % Sum of the hessian of each constraint with Lagrange multiplier
    for c = 1:NUM_CONSTRAINTS
        constraint_hessian_final = constraint_hessian_final + ...
            lambda(c)*jacobian(constraint_jacobian_final(c,:),[x;u]);
    end
    
    % Non zeros only in current timestep entries
    constraint_hessian_final = ...
        simplify(constraint_hessian_final(1:NUM_DECISION_VARS,1:NUM_DECISION_VARS));
    
    %% Initial constraints  (DOES NOT WORK YET)
    % Since the final timestep does not include future dynamics or foot
    % locations, it needs to be treated as a special case where only the
    % instantaneous forces and foot vectors are constrained.
    % Needed if constraints are x_{k-1} -> x_{k}
    fprintf('Calculating Initial Constraints...\n')
    
    % Initial constraints do not include next iteration decision variables
    constraints_initial = constraints;
    
    % Initial no slip condition is not constrained
    i_slip = NUM_STATES + NUM_FEET;
    constraints_initial(i_slip + 1:i_slip + 2*NUM_FEET,k) = ...
        zeros(2*NUM_FEET,1);   % p_foot in from current - p_foot_k
    
    % Initial iteration constraint jacobian (no future decision variables)
    fprintf('Calculating Initial Constraint Jacobian...\n')
    constraint_jacobian_initial = simplify(jacobian(constraints_initial,[x1;u]));
    
    % Non zero entries in the initial constraint jacobian
    constraint_jacobian_initial_nz = ...
        constraint_jacobian_initial(constraint_jacobian_initial~=0);
    
    % Calculate the initial constraint jacobian sparsity pattern
    fprintf('Calculating Initial Constraint Jacobian Sparsity Pattern...\n')
    [row_index_initial_CJ, col_index_initial_CJ] = ...
        find(constraint_jacobian_initial);
    
    % Shift indices for C++
    row_index_initial_CJ = row_index_initial_CJ - 1;
    col_index_initial_CJ = col_index_initial_CJ - 1;
    
    % Add the iteration index modifier
    row_index_initial_CJ = row_index_initial_CJ + ...
        ones(size(row_index_initial_CJ,1),1)*iter*NUM_C;
    col_index_initial_CJ = col_index_initial_CJ + ...
        ones(size(col_index_initial_CJ,1),1)*iter*NUM_X;
    
    % Calculate the initial constraint hessian
    fprintf('Calculating Initial Constraint Hessian...\n')
    constraint_hessian_initial = zeros(NUM_DECISION_VARS:NUM_DECISION_VARS);
    
    % Sum of the hessian of each constraint with Lagrange multiplier
    for c = 1:NUM_CONSTRAINTS
        constraint_hessian_initial = constraint_hessian_initial + ...
            lambda(c)*jacobian(constraint_jacobian_initial(c,:),[x1;u]);
    end
    
    % Non zeros only in current timestep entries
    constraint_hessian_initial = ...
        simplify(constraint_hessian_initial(1:NUM_DECISION_VARS,1:NUM_DECISION_VARS));
    
    % Print out the non zero entry results
    fprintf('   Nonzeros in Constraint Jacobian: %i\n',nnz(constraint_jacobian));
    fprintf('   Nonzeros in final Constraint Jacobian: %i\n',nnz(constraint_jacobian_final));
    fprintf('   Nonzeros in initial Constraint Jacobian: %i\n',nnz(constraint_jacobian_initial));
    
    save('Data/RPC_constraints.mat','constraints','constraint_jacobian','constraint_jacobian_nz','constraint_hessian','row_index_CJ','col_index_CJ',...
        'constraints_final','constraint_jacobian_final','constraint_jacobian_final_nz','constraint_hessian_final','row_index_final_CJ','col_index_final_CJ',...
        'constraints_initial','constraint_jacobian_initial','constraint_jacobian_initial_nz','constraint_hessian_initial','row_index_initial_CJ','col_index_initial_CJ');
    
    % Print timing statistics
    fprintf('DONE calculating constraints: %f s\n\n',toc(timers.constraint_timer));
elseif options.LOAD_CONSTRAINTS
    timers.constraint_timer = tic;
    fprintf('Loading Constraints...\n')
    load('Data/RPC_constraints.mat');
    fprintf('   Nonzeros in Constraint Jacobian: %i\n',nnz(constraint_jacobian));
    fprintf('   Nonzeros in final Constraint Jacobian: %i\n',nnz(constraint_jacobian_final));
    fprintf('   Nonzeros in initial Constraint Jacobian: %i\n',nnz(constraint_jacobian_initial));
    fprintf('DONE loading constraints: %f s\n\n',toc(timers.constraint_timer));
    
else
end



%% Lagrangian Hessian
if options.LAGRANGIAN_HESSIAN
    timers.hessian_timer = tic;
    fprintf('Calculating Lagrangian Hessian...\n');
    
    % Combine the cost and constraint hessians
    lagrangian_hessian = cost_hessian + constraint_hessian;
    
    % Only use the lower triangular part since it is symmetric
    lagrangian_hessian_full = lagrangian_hessian;
    lagrangian_hessian = simplify(tril(lagrangian_hessian));
    
    % Non zero entries in the lagrangian hessian
    lagrangian_hessian_nz = lagrangian_hessian(lagrangian_hessian~=0);
    
    fprintf('Calculating Lagrangian Hessian Sparsity Pattern...\n')
    
    % Find the non-zero row and column indices
    [row_index_H, col_index_H] = find(lagrangian_hessian);
    
    % Shift indices for C++
    row_index_H = row_index_H - 1;
    col_index_H = col_index_H - 1;
    
    % Add the iteration index modifier
    row_index_H = row_index_H + ones(size(row_index_H,1),1)*iter*NUM_X;
    col_index_H = col_index_H + ones(size(col_index_H,1),1)*iter*NUM_X;
    
    %% Final Hessian
    fprintf('Calculating Final Lagrangian Hessian...\n');
    
    % Combine the cost and constraint hessians
    lagrangian_hessian_final = cost_hessian + constraint_hessian_final;
    
    % Only use the lower triangular part since it is symmetric
    lagrangian_hessian_full_final = lagrangian_hessian_final;
    lagrangian_hessian_final = simplify(tril(lagrangian_hessian_final));
    
    % Non zero entries in the lagrangian hessian
    lagrangian_hessian_final_nz = lagrangian_hessian_final(lagrangian_hessian_final~=0);
    
    fprintf('Calculating Final Lagrangian Hessian Sparsity Pattern...\n')
    
    % Find the non-zero row and column indices
    [row_index_final_H, col_index_final_H] = find(lagrangian_hessian_final);
    
    % Shift indices for C++
    row_index_final_H = row_index_final_H - 1;
    col_index_final_H = col_index_final_H - 1;
    
    % Add the iteration index modifier
    row_index_final_H = row_index_final_H +...
        ones(size(row_index_final_H,1),1)*iter*NUM_X;
    col_index_final_H = col_index_final_H +...
        ones(size(col_index_final_H,1),1)*iter*NUM_X;
    
    %% Initial Hessian
    fprintf('Calculating Initial Lagrangian Hessian...\n');
    
    % Combine the cost and constraint hessians
    lagrangian_hessian_initial = cost_hessian + constraint_hessian_initial;
    
    % Only use the lower triangular part since it is symmetric
    lagrangian_hessian_full_initial = lagrangian_hessian_initial;
    lagrangian_hessian_initial = simplify(tril(lagrangian_hessian_initial));
    
    % Non zero entries in the lagrangian hessian
    lagrangian_hessian_initial_nz = lagrangian_hessian_initial(lagrangian_hessian_initial~=0);
    
    fprintf('Calculating Initial Lagrangian Hessian Sparsity Pattern...\n')
    
    % Find the non-zero row and column indices
    [row_index_initial_H, col_index_initial_H] = find(lagrangian_hessian_initial);
    
    % Shift indices for C++
    row_index_initial_H = row_index_initial_H - 1;
    col_index_initial_H = col_index_initial_H - 1;
    
    % Add the iteration index modifier
    row_index_initial_H = row_index_initial_H +...
        ones(size(row_index_initial_H,1),1)*iter*NUM_X;
    col_index_initial_H = col_index_initial_H +...
        ones(size(col_index_initial_H,1),1)*iter*NUM_X;
    
    % Print out the non zero entry results
    fprintf('   Nonzeros in Lagrangian Hessian: %i\n',nnz(lagrangian_hessian));
    fprintf('   Nonzeros in final Lagrangian Hessian: %i\n',nnz(lagrangian_hessian_final));
    fprintf('   Nonzeros in initial Lagrangian Hessian: %i\n',nnz(lagrangian_hessian_initial));
    
    save('Data/RPC_hessian.mat','lagrangian_hessian','lagrangian_hessian_nz','row_index_H','col_index_H',...
        'lagrangian_hessian_final','lagrangian_hessian_final_nz','row_index_final_H','col_index_final_H',...
        'lagrangian_hessian_initial','lagrangian_hessian_initial_nz','row_index_initial_H','col_index_initial_H');
    
    
    fprintf('DONE calculating Lagrangian Hessian: %f s\n\n',toc(timers.hessian_timer));
elseif options.LOAD_LAGRANGIAN_HESSIAN
    timers.hessian_timer = tic;
    fprintf('Calculating Lagrangian Hessian...\n');
    load('Data/RPC_hessian');
    fprintf('   Nonzeros in Lagrangian Hessian: %i\n',nnz(lagrangian_hessian));
    fprintf('   Nonzeros in final Lagrangian Hessian: %i\n',nnz(lagrangian_hessian_final));
    fprintf('   Nonzeros in initial Lagrangian Hessian: %i\n',nnz(lagrangian_hessian_initial));
    fprintf('DONE loading Lagrangian Hessian: %f s\n\n',toc(timers.hessian_timer));
else
    
end


%% Generate Functions
if options.GEN_FUNCTIONS
    timers.functions_timer = tic;
    fprintf('Generating MATLAB functions...\n')
    if options.GEN_INITIALIZATION
        % Generate decision variable and constraint bounds
        fprintf('Bounds function Gen...\n')
        matlabFunction(X_lb, X_ub, constraints_ub, constraints_lb,...
            'file','RPCGen/RPCBounds',...
            'vars',{x, x_input, c_states, r_hip, x_max, x_min, u_max, u_min});
        
        % Generate initial guess, reference policy, and footstep references
        fprintf('Initial Conditions function Gen...\n')
        matlabFunction(X_initial, x_ref_initial, u_ref_initial, p_foot_ref,...
            'file','RPCGen/RPCInitialize',...
            'vars',{x, x_input, c_states, HX, p_foot_touchdown, touchdown,...
            x_touchdown, dt, m, I, g});
    end
    
    if options.GEN_HEURISTICS
        % Generate Heuristics (change function name for other heuristics)
        fprintf('Heuristic function Gen...\n')
        matlabFunction(H_X,...
            'file','RPCGen/RPCHeuristics',...
            'vars',{x, x_input, c_states, s_Phi, r_hip,...
            stance_fraction, flight_phase, T_s, K_HX, m, g});
    end
    
    if options.GEN_COST
        % Generate cost function
        fprintf('Cost function Gen...\n');
        matlabFunction(J,...
            'file','RPCGen/RPCCost',...
            'vars',{x, u, x_ref, u_ref, Q, R});
        
        % Generate cost gradient function
        fprintf('Cost Gradient function Gen...\n');
        matlabFunction(cost_gradient,...
            'file','RPCGen/RPCCostGradient',...
            'vars',{x, u, x_ref, u_ref, Q, R});
        
    end
    
    if options.GEN_CONSTRAINTS
        % Generate constraints function
        fprintf('Constraints function Gen...\n');
        matlabFunction(constraints,...
            'file','RPCGen/RPCConstraints',...
            'vars',{x, u, x1, u1, c_states, c_states1, dt,...
            m, I, g, z_g, mu_g});
        
        % Generate constraint jacobian function
        fprintf('Constraint Jacobian function Gen...\n');
        matlabFunction(constraint_jacobian_nz,...
            'file','RPCGen/RPCConstraintJacobian',...
            'vars',{x, u, c_states, c_states1, dt, m, I, mu_g});
        
        % Generate constraint jacobian sparsity function
        fprintf('Constraint Jacobian Sparsity function Gen...\n')
        matlabFunction(row_index_CJ, col_index_CJ,...
            'file','RPCGen/RPCConstraintJacobianSP',...
            'vars', {iter, NUM_X, NUM_C});
        
        % Generate final constraints function
        fprintf('Final Constraints function Gen...\n');
        matlabFunction(constraints_final,...
            'file','RPCGen/RPCConstraintsFinal',...
            'vars',{x, u, c_states, z_g, mu_g});
        
        % Generate final constraint jacobian function
        fprintf('Final Constraint Jacobian function Gen...\n');
        matlabFunction(constraint_jacobian_final_nz,...
            'file','RPCGen/RPCConstraintJacobianFinal',...
            'vars',{c_states, mu_g});
        
        % Generate final constraint jacobian sparsity function
        fprintf('Final Constraint Jacobian Sparsity function Gen...\n')
        matlabFunction(row_index_final_CJ, col_index_final_CJ,...
            'file','RPCGen/RPCConstraintJacobianFinalSP',...
            'vars', {iter, NUM_X, NUM_C});
        
        % % Generate initial constraints function
        % fprintf('Initial Constraints function Gen...\n');
        % matlabFunction(constraints_initial,...
        %     'file','RPCGen/RPCConstraintsInitial',...
        %     'vars',{x; u; x1; c_states; dt;...
        %     m; I; g; z_g; mu_g});
        %
        % % Generate initial constraint jacobian function
        % fprintf('Initial Constraint Jacobian function Gen...\n');
        % matlabFunction(constraint_jacobian_initial_nz,...
        %     'file','RPCGen/RPCConstraintJacobianInitial',...
        %     'vars',{x; u; c_states; dt; m; I; mu_g});
        %
        % % Generate initial constraint jacobian sparsity function
        % fprintf('Initial Constraint Jacobian Sparsity function Gen...\n')
        % matlabFunction(row_index_CJ, col_index_CJ,...
        %     'file','RPCGen/RPCConstraintJacobianInitialSP',...
        %     'vars', {iter, NUM_X, NUM_C});
        
    end
    
    if options.GEN_LAGRANGIAN_HESSIAN
        % Generate lagrangian hessian function
        fprintf('Lagrangian Hessian function Gen...\n')
        matlabFunction(lagrangian_hessian_nz,...
            'file','RPCGen/RPCLagrangianHessian',...
            'vars',{x, u, c_states, Q, R, dt,...
            obj_factor, lambda, I});
        
        % Generate hessian sparsity function
        fprintf('Lagrangian Hessian Sparsity function Gen...\n')
        matlabFunction(row_index_H, col_index_H,...
            'file','RPCGen/RPCLagrangianHessianSP',...
            'vars', {iter, NUM_X});
        
        % Generate final lagrangian hessian function
        fprintf('Final Lagrangian Hessian function Gen...\n')
        matlabFunction(lagrangian_hessian_final_nz,...
            'file','RPCGen/RPCLagrangianHessianFinal',...
            'vars',{Q, R, obj_factor});
        
        % Generate final hessian sparsity function
        fprintf('Final Lagrangian Hessian Sparsity function Gen...\n')
        matlabFunction(row_index_final_H, col_index_final_H,...
            'file','RPCGen/RPCLagrangianHessianFinalSP',...
            'vars', {iter, NUM_X});
    end
    
    fprintf('DONE generating MATLAB functions: %f s\n\n',toc(timers.functions_timer));
else
    fprintf('No MATLAB functions were generated.\n\n');
end


%%
% Finalize generation statistics
fprintf(['DONE generating symbolic functions!\n',...
    '   Total time taken: %f s\n\n'],toc(timers.SymbolicPRMPC_timer))