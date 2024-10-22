clear all

%% Paramter preparation
syms x y th q_hip_r q_knee_r q_ankle_r q_hip_l q_knee_l q_ankle_l real 
syms dx dy dth dq_hip_r dq_knee_r dq_ankle_r dq_hip_l dq_knee_l dq_ankle_l real 
syms ddx ddy ddth ddq_hip_r ddq_knee_r ddq_ankle_r ddq_hip_l ddq_knee_l ddq_ankle_l real 

c_body = 0.07;
c_thigh = 0.15;
c_shank = 0.12;

l_thigh = 0.28;
l_shank = 0.3;
l_foot = 0.08;

m_body = 15;
m_thigh = 3;
m_shank = 1.5;
m_foot = 0.5;

I_body = 0.1;  % body inertia
I_thigh = 0.05;
I_shank = 0.02;
I_foot = 0.008;

p_shoulder = 0.3;
g = 9.81;

q   = [x; y; th; q_hip_r; q_knee_r; q_ankle_r; q_hip_l; q_knee_l; q_ankle_l];      % generalized coordinates

dq   = [dx; dy; dth; dq_hip_r; dq_knee_r; dq_ankle_r; ...
                     dq_hip_l; dq_knee_l; dq_ankle_l];
                 
ddq   = [ddx; ddy; ddth; ddq_hip_r; ddq_knee_r; ddq_ankle_r; ...
                         ddq_hip_l; ddq_knee_l; ddq_ankle_l];
                     
ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; 

%% Define vectors to key points.
pelvis   = [x; y];         % floating base
knee_r = pelvis + [l_thigh*sin(th + q_hip_r); -l_thigh*cos(th + q_hip_r)];
ankle_r = knee_r + [l_shank*sin(th + q_hip_r + q_knee_r); -l_shank*cos(th + q_hip_r + q_knee_r)];
toe_r = ankle_r + [l_foot*sin(th + pi/2 + q_hip_r  + q_knee_r + q_ankle_r); ...
                  -l_foot*cos(th + pi/2 + q_hip_r  + q_knee_r + q_ankle_r)];
heel_r = ankle_r + [l_foot*sin(th + -pi/2 + q_hip_r  + q_knee_r + q_ankle_r); ...
                  -l_foot*cos(th + -pi/2 + q_hip_r  + q_knee_r + q_ankle_r)];

knee_l = pelvis + [l_thigh*sin(th + q_hip_l); -l_thigh*cos(th + q_hip_l)];
ankle_l = knee_l + [l_shank*sin(th + q_hip_l + q_knee_l); -l_shank*cos(th + q_hip_l + q_knee_l)];
toe_l = ankle_l + [l_foot*sin(th + pi/2 + q_hip_l  + q_knee_l + q_ankle_l); ...
                  -l_foot*cos(th + pi/2 + q_hip_l  + q_knee_l + q_ankle_l)];
heel_l = ankle_l + [l_foot*sin(th + -pi/2 + q_hip_l  + q_knee_l + q_ankle_l); ...
                  -l_foot*cos(th + -pi/2 + q_hip_l  + q_knee_l + q_ankle_l)];

shoulder = pelvis + [ -p_shoulder*sin(th); p_shoulder*cos(th)];

keypoints = [pelvis knee_r ankle_r toe_r heel_r ...
                    knee_l ankle_l toe_l heel_l shoulder]; 


% Compute Jacobian
num_key_pts = size(keypoints,2);
dim_q = size(q,1);
% keypoints_vel = zeros(2, num_key_pts);
% keypoints_J = zeros(2*num_key_pts, dim_q);

for i =1:10
    keypoints_vel(:,i) = ddt(keypoints(:,i));
    keypoints_J(2*i-1:2*i,:) = jacobian(keypoints(:,i),q);
end

% Compute ddt( J )
% dJ= reshape( ddt(J(:)) , size(J) );

%% Define vector for CoM
cm_body = pelvis + [-c_body*sin(th); c_body*cos(th)];
cm_thigh_r = pelvis + [c_thigh*sin(th + q_hip_r); -c_thigh*cos(th + q_hip_r)];
cm_shank_r = knee_r + [c_shank*sin(th + q_hip_r + q_knee_r); -c_shank*cos(th + q_hip_r + q_knee_r)];
cm_foot_r = ankle_r;

cm_thigh_l = pelvis + [c_thigh*sin(th + q_hip_l); -c_thigh*cos(th + q_hip_l)];
cm_shank_l = knee_l + [c_shank*sin(th + q_hip_l + q_knee_l); -c_shank*cos(th + q_hip_l + q_knee_l)];
cm_foot_l = ankle_l;

                
% Take time derivatives of vectors as required for kinetic energy terms.
dcm_body = ddt(cm_body);
dcm_thigh_r = ddt(cm_thigh_r);
dcm_shank_r = ddt(cm_shank_r);
dcm_foot_r = ddt(cm_foot_r);
dcm_thigh_l = ddt(cm_thigh_l);
dcm_shank_l = ddt(cm_shank_l);
dcm_foot_l = ddt(cm_foot_l);

%% Define kinetic energies.
T_body  = 0.5*m_body*dot(dcm_body, dcm_body) + 0.5* I_body * dth^2;
T_thigh_r  = 0.5*m_thigh*dot(dcm_thigh_r, dcm_thigh_r) + 0.5* I_thigh * (dth + dq_hip_r)^2;
T_shank_r  = 0.5*m_shank*dot(dcm_shank_r, dcm_shank_r) + 0.5* I_shank * (dth + dq_hip_r + dq_knee_r)^2;
T_foot_r  = 0.5*m_thigh*dot(dcm_foot_r, dcm_foot_r) + 0.5* I_foot * (dth + dq_hip_r + dq_knee_r + dq_ankle_r)^2;
T_thigh_l  = 0.5*m_thigh*dot(dcm_thigh_l, dcm_thigh_l) + 0.5* I_thigh * (dth + dq_hip_l)^2;
T_shank_l  = 0.5*m_shank*dot(dcm_shank_l, dcm_shank_l) + 0.5* I_shank * (dth + dq_hip_l + dq_knee_l)^2;
T_foot_l  = 0.5*m_thigh*dot(dcm_foot_l, dcm_foot_l) + 0.5* I_foot * (dth + dq_hip_l + dq_knee_l + dq_ankle_l)^2;

%% Define Potential energies.
jhat = [0; 1];

V_body = m_body*g*dot(cm_body, jhat);
V_thigh_r = m_thigh*g*dot(cm_thigh_r, jhat);
V_shank_r = m_shank*g*dot(cm_shank_r, jhat);
V_foot_r = m_foot*g*dot(cm_foot_r, jhat);
V_thigh_l = m_thigh*g*dot(cm_thigh_l, jhat);
V_shank_l = m_shank*g*dot(cm_shank_l, jhat);
V_foot_l = m_foot*g*dot(cm_foot_l, jhat);

% Sum kinetic energy  and potential energy terms
T = T_body + T_thigh_r + T_shank_r + T_foot_r ... 
           + T_thigh_l + T_shank_l + T_foot_l;
V = V_body + V_thigh_r + V_shank_r + V_foot_r ... 
           + V_thigh_l + V_shank_l + V_foot_l;
       
% Calculate rcm, the location of the center of mass
com_robot = (m_body*cm_body + m_thigh*cm_thigh_r + m_shank*cm_shank_r + m_foot*cm_foot_r ... 
                        + m_thigh*cm_thigh_l + m_shank*cm_shank_l + m_foot*cm_foot_l)/...
        (m_body + (m_thigh + m_shank + m_foot)*2);
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
gravity = simplify(jacobian(V, q)');
coriolis = simplify( eom - gravity - A*ddq);

%%% Rearrange Equations of Motion. 
A = jacobian(eom,ddq);
b = A*ddq - eom;

% Write a function to evaluate the X and Y coordinates and speeds of the center of mass given the current state and parameters
COM = [com_robot; dcom_robot]; % Concatenate x and y coordinates and speeds of center of mass in array

%% Save files
z = [q;dq];

matlabFunction(E, 'file', 'energy_biped','var',{z});

matlabFunction(A, 'file', 'A_biped', 'var', {z});
matlabFunction(gravity, 'file','grav_biped', 'var', {z});
matlabFunction(coriolis, 'file','coriolis_biped', 'var', {z});

matlabFunction(keypoints, 'file','keypoints_biped', 'var',{z});
matlabFunction(keypoints_vel, 'file','keypoints_vel_biped', 'var',{z});
matlabFunction(COM, 'file','CoM_biped', 'var',{z});
matlabFunction(keypoints_J, 'file','J_biped', 'var',{z});

%% Export files to C++
