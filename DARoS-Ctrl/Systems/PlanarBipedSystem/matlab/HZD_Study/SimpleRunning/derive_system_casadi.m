function [kinematics] = derive_system_casadi() 
% Biped (need to draw a diagram of the leg to reference)

% Returns 2 structures
% @kinematics: contains casadi functions for kinematics
% @dynamics: contains casadi functions for dynamics

% Define variables for generalized coordinates + derivatives and controls
x    = casadi.SX.sym('x',1,1);    % horizontal position
y    = casadi.SX.sym('y',1,1);    % vertical position
q_hip   = casadi.SX.sym('q_hip',1,1);
q_knee   = casadi.SX.sym('q_knee',1,1);

dx   = casadi.SX.sym('dx',1,1);   % horizontal velocity
dy   = casadi.SX.sym('dy',1,1);   % vertical velocity
dq_hip   = casadi.SX.sym('dq_hip',1,1);
dq_knee   = casadi.SX.sym('dq_knee',1,1);


ddx  = casadi.SX.sym('ddx',1,1);  % horizontal acceleration
ddy  = casadi.SX.sym('ddy',1,1);  % vertical acceleration
ddq_hip   = casadi.SX.sym('ddq_hip',1,1);
ddq_knee   = casadi.SX.sym('ddq_knee',1,1);


% Parameters %%%%%%%%%%%%%%% Warning: Need to synchronize with simulation
% l_thigh = casadi.SX.sym('l_thigh', 1,1);
% l_shank = casadi.SX.sym('l_shank',1,1);

l_thigh = 0.345;
l_shank = 0.355;


m_body = 15;

g = 9.81;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Group terms for later use
q   = [x; y; q_hip; q_knee];      % generalized coordinates
dq   = [dx; dy; dq_hip; dq_knee];
ddq   = [ddx; ddy; ddq_hip; ddq_knee];                 

% Define vectors to key points.
pelvis   = [x; y];         % floating base
knee = pelvis + [l_thigh*sin(q_hip); -l_thigh*cos(q_hip)];
ankle = knee + [l_shank*sin(q_hip + q_knee); -l_shank*cos(q_hip + q_knee)];


keypoints = [pelvis knee ankle]; 

% Compute foot jacobian
J_ankle = jacobian(ankle,q);

%%% Write functions to evaluate dynamics, etc...
z = [q;dq(1:2)];
p = [l_thigh; l_shank];
kinematics.keypoints = casadi.Function('keypoints',{z},{keypoints});
kinematics.J_contact = casadi.Function('J_contact',{z},{J_ankle});


