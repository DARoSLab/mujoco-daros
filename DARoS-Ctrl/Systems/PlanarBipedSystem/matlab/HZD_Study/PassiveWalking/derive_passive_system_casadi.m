function [kinematics,dynamics] = derive_passive_system_casadi() 
% Biped (need to draw a diagram of the leg to reference)

% Returns 2 structures
% @kinematics: contains casadi functions for kinematics
% @dynamics: contains casadi functions for dynamics

% Define variables for generalized coordinates + derivatives and controls
th   = casadi.SX.sym('th',1,1);   % stance angle
phi    = casadi.SX.sym('phi',1,1);    % swing leg angle

dth  = casadi.SX.sym('dth',1,1);  % shin angular velocity
dphi   = casadi.SX.sym('dphi',1,1);   % vertical velocity

ddth = casadi.SX.sym('ddth',1,1); % body angular acceleration
ddphi  = casadi.SX.sym('ddy',1,1);  % vertical acceleration

% Parameters %%%%%%%%%%%%%%% Warning: Need to synchronize with simulation
% model
params = [25; 5; 0.8; 9.81];
m_body = params(1);
m_foot = params(2);
length = params(3);

g = params(4);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Group terms for later use
q   = [th; phi];      % generalized coordinates
dq   = [dth; dphi];
ddq   = [ddth; ddphi];                

%%% Calculate important vectors and their time derivatives.

% A handy anonymous function for taking first and second time derivatives
% of vectors using the chain rule.  See Lecture 6 for more information. 
ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; 

% Define vectors to key points.
cm_body   = [-length*sin(th); length*cos(th)];         % floating base
cm_foot = cm_body + [-length*sin(phi-th); -length*cos(phi-th)];

keypoints = [cm_body cm_foot]; 
               
% Take time derivatives of vectors as required for kinetic energy terms.
dcm_body = ddt(cm_body);
dcm_foot = ddt(cm_foot);

%%% Calculate Kinetic Energy, Potential Energy, and Generalized Forces
T_body  = simplify(0.5*m_body*dot(dcm_body, dcm_body));
T_foot  = simplify(0.5*m_foot*dot(dcm_foot, dcm_foot));

jhat = [0; 1];
V_body = m_body*g*dot(cm_body, jhat);
V_foot = m_foot*g*dot(cm_foot, jhat);

% Sum kinetic energy  and potential energy terms
T = T_body + T_foot;
V = V_body + V_foot;

%% All the work is done!  Just turn the crank...
%%% Derive Energy Function and Equations of Motion
E = T+V;                                         % total system energy
L = T-V;                                         % the Lagrangian
eom = ddt(jacobian(L,dq)') - jacobian(L,q)';  % form the dynamics equations

% Rearrange Equations of Motion
A = simplify(jacobian(eom,ddq));

% Equations of motion are
% eom = A *ddq + (coriolis term) + (gravitational term) = 0
Grav_Joint_Sp = simplify(jacobian(V, q)');
Cori_Joint_Sp = simplify( eom - Grav_Joint_Sp - A*ddq);

%%% Rearrange Equations of Motion. 
A = jacobian(eom,ddq);
b = A*ddq - eom;

%%% Write functions to evaluate dynamics, etc...
z = [q;dq];
dynamics.energy      = casadi.Function('energy',{z},{E});
dynamics.A           = casadi.Function('A',{z},{A});
dynamics.grav        = casadi.Function('grav',{z},{Grav_Joint_Sp});
dynamics.cori        = casadi.Function('cori',{z},{Cori_Joint_Sp});
kinematics.keypoints = casadi.Function('keypoints',{z},{keypoints});

