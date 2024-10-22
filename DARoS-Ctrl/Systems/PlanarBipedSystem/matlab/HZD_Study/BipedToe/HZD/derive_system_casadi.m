function [kinematics,dynamics] = derive_system_casadi() 
% Biped (need to draw a diagram of the leg to reference)

% Returns 2 structures
% @kinematics: contains casadi functions for kinematics
% @dynamics: contains casadi functions for dynamics

% Define variables for generalized coordinates + derivatives and controls
x    = casadi.SX.sym('x',1,1);    % horizontal position
y    = casadi.SX.sym('y',1,1);    % vertical position
th   = casadi.SX.sym('th',1,1);   % body angle
q_hip_r   = casadi.SX.sym('q_hip_r',1,1);
q_knee_r   = casadi.SX.sym('q_knee_r',1,1);
q_ankle_r   = casadi.SX.sym('q_ankle_r',1,1);
q_toe_r   = casadi.SX.sym('q_toe_r',1,1);
q_hip_l   = casadi.SX.sym('q_hip_l',1,1);
q_knee_l   = casadi.SX.sym('q_knee_l',1,1);
q_ankle_l   = casadi.SX.sym('q_ankle_l',1,1);
q_toe_l = casadi.SX.sym('q_toe_l',1,1);

dx   = casadi.SX.sym('dx',1,1);   % horizontal velocity
dy   = casadi.SX.sym('dy',1,1);   % vertical velocity
dth  = casadi.SX.sym('dth',1,1);  % shin angular velocity
dq_hip_r   = casadi.SX.sym('dq_hip_r',1,1);
dq_knee_r   = casadi.SX.sym('dq_knee_r',1,1);
dq_ankle_r = casadi.SX.sym('dq_ankle_r',1,1);
dq_toe_r = casadi.SX.sym('dq_toe_r',1,1);
dq_hip_l   = casadi.SX.sym('dq_hip_l',1,1);
dq_knee_l   = casadi.SX.sym('dq_knee_l',1,1);
dq_ankle_l = casadi.SX.sym('dq_ankle_l',1,1);
dq_toe_l = casadi.SX.sym('dq_toe_l',1,1);

ddx  = casadi.SX.sym('ddx',1,1);  % horizontal acceleration
ddy  = casadi.SX.sym('ddy',1,1);  % vertical acceleration
ddth = casadi.SX.sym('ddth',1,1); % body angular acceleration
ddq_hip_r   = casadi.SX.sym('ddq_hip_r',1,1);
ddq_knee_r   = casadi.SX.sym('ddq_knee_r',1,1);
ddq_ankle_r = casadi.SX.sym('ddq_ankle_r',1,1);
ddq_toe_r = casadi.SX.sym('ddq_toe_r',1,1);
ddq_hip_l   = casadi.SX.sym('ddq_hip_l',1,1);
ddq_knee_l   = casadi.SX.sym('ddq_knee_l',1,1);
ddq_ankle_l = casadi.SX.sym('ddq_ankle_l',1,1);
ddq_toe_l = casadi.SX.sym('ddq_toe_l',1,1);


% Parameters %%%%%%%%%%%%%%% Warning: Need to synchronize with simulation
c_body = 0.2;
c_thigh = 0.15;
c_shank = 0.12;
c_foot = 0.02;
c_toe = 0.01;

l_thigh = 0.38;
l_shank = 0.34;
l_heel_x = -0.06;

l_heel_y = -0.05;
l_toe_jpos_y = -0.03;
l_toe_tip_y = -0.02;

l_toe_jpos_x = 0.048;
l_toe_tip_x = 0.05;
l_toe_back_x = -0.025;

m_body = 10;
m_thigh = 3.0;
m_shank = 1.0;
m_foot = 0.7;
m_toe = 0.1;

I_body = 0.35;  % body inertia
I_thigh = 0.04;
I_shank = 0.01;
I_foot = 0.002;
I_toe = 0.001;
I_rotor = 0.00001; % rotor inertia
N_ratio = 9; % gear ratio

p_shoulder = 0.55;
g = 9.81;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Group terms for later use
q   = [x; y; th; q_hip_r; q_knee_r; q_ankle_r; q_toe_r; q_hip_l; q_knee_l; q_ankle_l; q_toe_l];      % generalized coordinates

dq   = [dx; dy; dth; dq_hip_r; dq_knee_r; dq_ankle_r; dq_toe_r; ...
                     dq_hip_l; dq_knee_l; dq_ankle_l; dq_toe_l];
                 
ddq   = [ddx; ddy; ddth; ddq_hip_r; ddq_knee_r; ddq_ankle_r; ddq_toe_r; ...
                         ddq_hip_l; ddq_knee_l; ddq_ankle_l; ddq_toe_l];

%%% Calculate important vectors and their time derivatives.

% A handy anonymous function for taking first and second time derivatives
% of vectors using the chain rule.  See Lecture 6 for more information. 
ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; 

% Define vectors to key points.
pelvis   = [x; y];         % floating base
knee_r = pelvis + [l_thigh*sin(th + q_hip_r); -l_thigh*cos(th + q_hip_r)];
ankle_r = knee_r + [l_shank*sin(th + q_hip_r + q_knee_r); -l_shank*cos(th + q_hip_r + q_knee_r)];

theta_ankle_r = th + q_hip_r + q_knee_r + q_ankle_r;
heel_r = ankle_r + [cos(theta_ankle_r), -sin(theta_ankle_r); 
                    sin(theta_ankle_r), cos(theta_ankle_r)]*[l_heel_x; l_heel_y];
toe_joint_r = ankle_r + [cos(theta_ankle_r), -sin(theta_ankle_r); 
                    sin(theta_ankle_r), cos(theta_ankle_r)]*[l_toe_jpos_x; l_toe_jpos_y];

theta_toe_r = th + q_hip_r + q_knee_r + q_ankle_r + q_toe_r;
toe_tip_r = toe_joint_r + [cos(theta_toe_r), -sin(theta_toe_r); 
                    sin(theta_toe_r), cos(theta_toe_r)]*[l_toe_tip_x; l_toe_tip_y];
toe_back_r = toe_joint_r + [cos(theta_toe_r), -sin(theta_toe_r);
                    sin(theta_toe_r), cos(theta_toe_r)]*[l_toe_back_x; l_toe_tip_y];  


knee_l = pelvis + [l_thigh*sin(th + q_hip_l); -l_thigh*cos(th + q_hip_l)];
ankle_l = knee_l + [l_shank*sin(th + q_hip_l + q_knee_l); -l_shank*cos(th + q_hip_l + q_knee_l)];

theta_ankle_l = th + q_hip_l + q_knee_l + q_ankle_l;
heel_l = ankle_l + [cos(theta_ankle_l), -sin(theta_ankle_l); 
                    sin(theta_ankle_l), cos(theta_ankle_l)]*[l_heel_x; l_heel_y];
toe_joint_l = ankle_l + [cos(theta_ankle_l), -sin(theta_ankle_l); 
                    sin(theta_ankle_l), cos(theta_ankle_l)]*[l_toe_jpos_x; l_toe_jpos_y];

theta_toe_l = th + q_hip_l + q_knee_l + q_ankle_l + q_toe_l;
toe_tip_l = toe_joint_l + [cos(theta_toe_l), -sin(theta_toe_l); 
                    sin(theta_toe_l), cos(theta_toe_l)]*[l_toe_tip_x; l_toe_tip_y];
toe_back_l = toe_joint_l + [cos(theta_toe_l), -sin(theta_toe_l);
                    sin(theta_toe_l), cos(theta_toe_l)]*[l_toe_back_x; l_toe_tip_y];  


shoulder = pelvis + [ -p_shoulder*sin(th); p_shoulder*cos(th)];

keypoints = [pelvis knee_r ankle_r heel_r toe_joint_r toe_tip_r toe_back_r ...
                    knee_l ankle_l heel_l toe_joint_l toe_tip_l toe_back_l ...
                    shoulder]; 

% Define vector for CoM
cm_body = pelvis + [-c_body*sin(th); c_body*cos(th)];
cm_thigh_r = pelvis + [c_thigh*sin(th + q_hip_r); -c_thigh*cos(th + q_hip_r)];
cm_shank_r = knee_r + [c_shank*sin(th + q_hip_r + q_knee_r); -c_shank*cos(th + q_hip_r + q_knee_r)];
cm_foot_r = ankle_r + [c_foot*sin(theta_ankle_r); -c_foot*cos(theta_ankle_r)];
cm_toe_r = toe_joint_r + [c_toe*sin(theta_toe_r); -c_toe*cos(theta_toe_r)];

cm_thigh_l = pelvis + [c_thigh*sin(th + q_hip_l); -c_thigh*cos(th + q_hip_l)];
cm_shank_l = knee_l + [c_shank*sin(th + q_hip_l + q_knee_l); -c_shank*cos(th + q_hip_l + q_knee_l)];
cm_foot_l = ankle_l + [c_foot*sin(theta_ankle_l); -c_foot*cos(theta_ankle_l)];
cm_toe_l = toe_joint_l + [c_toe*sin(theta_toe_l); -c_toe*cos(theta_toe_l)];
               
% Take time derivatives of vectors as required for kinetic energy terms.
dcm_body = ddt(cm_body);
dcm_thigh_r = ddt(cm_thigh_r);
dcm_shank_r = ddt(cm_shank_r);
dcm_foot_r = ddt(cm_foot_r);
dcm_toe_r = ddt(cm_toe_r);
dcm_thigh_l = ddt(cm_thigh_l);
dcm_shank_l = ddt(cm_shank_l);
dcm_foot_l = ddt(cm_foot_l);
dcm_toe_l = ddt(cm_toe_l);


%%% Calculate Kinetic Energy, Potential Energy

% Define kinetic energies. 
T_body  = 0.5*m_body*dot(dcm_body, dcm_body) + 0.5* I_body * dth^2;
T_thigh_r  = 0.5*m_thigh*dot(dcm_thigh_r, dcm_thigh_r) + 0.5* I_thigh * (dth + dq_hip_r)^2;
T_hip_rotor_r = 0.5*I_rotor*(dth + N_ratio*dq_hip_r)^2;

T_shank_r  = 0.5*m_shank*dot(dcm_shank_r, dcm_shank_r) + 0.5* I_shank * (dth + dq_hip_r + dq_knee_r)^2;
T_knee_rotor_r = 0.5*I_rotor*(dth + dq_hip_r + N_ratio*dq_knee_r)^2;

T_foot_r  = 0.5*m_foot*dot(dcm_foot_r, dcm_foot_r) + 0.5* I_foot * (dth + dq_hip_r + dq_knee_r + dq_ankle_r)^2;
T_ankle_rotor_r = 0.5*I_rotor*(dth + dq_hip_r + dq_knee_r + N_ratio*dq_ankle_r)^2;

T_toe_r = 0.5*m_toe*dot(dcm_toe_r, dcm_toe_r) + 0.5* I_toe * (dth + dq_hip_r + dq_knee_r + dq_ankle_r + dq_toe_r)^2;
T_toe_rotor_r = 0.5*I_rotor*(dth + dq_hip_r + dq_knee_r + dq_ankle_r + N_ratio*dq_toe_r)^2;

T_thigh_l  = 0.5*m_thigh*dot(dcm_thigh_l, dcm_thigh_l) + 0.5* I_thigh * (dth + dq_hip_l)^2;
T_hip_rotor_l = 0.5*I_rotor*(dth + N_ratio*dq_hip_l)^2;

T_shank_l  = 0.5*m_shank*dot(dcm_shank_l, dcm_shank_l) + 0.5* I_shank * (dth + dq_hip_l + dq_knee_l)^2;
T_knee_rotor_l = 0.5*I_rotor*(dth + dq_hip_l + N_ratio*dq_knee_l)^2;

T_foot_l  = 0.5*m_foot*dot(dcm_foot_l, dcm_foot_l) + 0.5* I_foot * (dth + dq_hip_l + dq_knee_l + dq_ankle_l)^2; 
T_ankle_rotor_l = 0.5*I_rotor*(dth + dq_hip_l + dq_knee_l + N_ratio*dq_ankle_l)^2;

T_toe_l = 0.5*m_toe*dot(dcm_toe_l, dcm_toe_l) + 0.5* I_toe * (dth + dq_hip_l + dq_knee_l + dq_ankle_l + dq_toe_l)^2;
T_toe_rotor_l = 0.5*I_rotor*(dth + dq_hip_l + dq_knee_l + dq_ankle_l + N_ratio*dq_toe_l)^2;

% Define potential energies.
jhat = [0; 1];

V_body = m_body*g*dot(cm_body, jhat);
V_thigh_r = m_thigh*g*dot(cm_thigh_r, jhat);
V_shank_r = m_shank*g*dot(cm_shank_r, jhat);
V_foot_r = m_foot*g*dot(cm_foot_r, jhat);
V_toe_r = m_toe*g*dot(cm_toe_r, jhat);

V_thigh_l = m_thigh*g*dot(cm_thigh_l, jhat);
V_shank_l = m_shank*g*dot(cm_shank_l, jhat);
V_foot_l = m_foot*g*dot(cm_foot_l, jhat);
V_toe_l = m_toe*g*dot(cm_toe_l, jhat);

% Sum kinetic energy terms, potential energy terms, and generalized force
% contributions.
T = T_body + T_thigh_r + T_shank_r + T_foot_r + T_toe_r + T_hip_rotor_r + T_knee_rotor_r + T_ankle_rotor_r + T_toe_rotor_r ... 
           + T_thigh_l + T_shank_l + T_foot_l + T_toe_l + T_hip_rotor_l + T_knee_rotor_l + T_ankle_rotor_l + T_toe_rotor_l;
V = V_body + V_thigh_r + V_shank_r + V_foot_r + V_toe_r ... 
           + V_thigh_l + V_shank_l + V_foot_l + V_toe_l;

% Calculate rcm, the location of the center of mass
com_robot = (m_body*cm_body + m_thigh*cm_thigh_r + m_shank*cm_shank_r ... 
                        + m_thigh*cm_thigh_l + m_shank*cm_shank_l)/...
        (m_body + (m_thigh + m_shank)*2 );
dcom_robot = ddt(com_robot);

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


% Compute foot jacobian

% keypoints = [pelvis knee_r ankle_r heel_r toe_joint_r toe_tip_r toe_back_r ...
%                     knee_l ankle_l heel_l toe_joint_l toe_tip_l toe_back_l ...
%                     shoulder]; 

J_heel_r = jacobian(heel_r,q);
J_toe_tip_r = jacobian(toe_tip_r,q);
J_toe_back_r = jacobian(toe_back_r,q);
J_heel_l = jacobian(heel_l,q);
J_toe_tip_l = jacobian(toe_tip_l,q);
J_toe_back_l = jacobian(toe_back_l,q);

J_contact = [ J_heel_r; J_toe_tip_r; J_toe_back_r; ...
              J_heel_l; J_toe_tip_l; J_toe_back_l];
          
% Compute ddt( J )
% dJ= reshape( ddt(J(:)) , size(J) );

%%% Write functions to evaluate dynamics, etc...
z = [q;dq];
dynamics.energy      = casadi.Function('energy',{z},{E});
dynamics.A           = casadi.Function('A',{z},{A});
dynamics.grav        = casadi.Function('grav',{z},{Grav_Joint_Sp});
dynamics.cori        = casadi.Function('cori',{z},{Cori_Joint_Sp});
kinematics.keypoints = casadi.Function('keypoints',{z},{keypoints});
kinematics.J_contact = casadi.Function('J_contact',{z},{J_contact});

% Write a function to evaluate the X and Y coordinates and speeds of the center of mass given the current state and parameters
COM = [com_robot; dcom_robot(1:2)]; % Concatenate x and y coordinates and speeds of center of mass in array
kinematics.COM = casadi.Function('COM',{z},{COM});
