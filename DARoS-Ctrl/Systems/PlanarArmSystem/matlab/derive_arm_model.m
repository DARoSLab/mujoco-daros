clear all

%% Paramter preparation
syms q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 real

c_L1 = 0.07;
c_L2 = 0.15;
c_L3 = 0.12;

l_L1 = 0.28;
l_L2 = 0.3;
l_L3 = 0.18;

m_L1 = 15;
m_L2 = 3;
m_L3 = 1.5;

I_L1 = 0.1;  % body inertia
I_L2 = 0.05;
I_L3 = 0.02;

g = 9.81;

q   = [q1; q2; q3];      % generalized coordinates
dq   = [dq1; dq2; dq3];
ddq   = [ddq1; ddq2; ddq3];
                     
ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; 

%% Define vectors to key points.
p_L1 = [l_L1*cos(q1); l_L1*sin(q1)];
p_L2 = p_L1 + [l_L2*cos(q1+q2); l_L2*sin(q1+q2)];
p_L3 = p_L2 + [l_L3*cos(q1+q2+q3); l_L3*sin(q1+q2+q3)];

keypoints = [p_L1 p_L2 p_L3]; 

% Compute Jacobian
num_key_pts = size(keypoints,2);
dim_q = size(q,1);
% keypoints_vel = zeros(2, num_key_pts);
% keypoints_J = zeros(2*num_key_pts, dim_q);

for i =1:num_key_pts
    keypoints_vel(:,i) = ddt(keypoints(:,i));
    keypoints_J(2*i-1:2*i,:) = jacobian(keypoints(:,i),q);
end

% Compute ddt( J )
% dJ= reshape( ddt(J(:)) , size(J) );

%% Define vector for CoM
cm_L1 = [c_L1*cos(q1); c_L1*sin(q1)];
cm_L2 = p_L1 + [c_L2*cos(q1+q2); c_L2*sin(q1+q2)];
cm_L3 = p_L2 + [c_L3*cos(q1+q2+q3); c_L3*sin(q1+q2+q3)];
                
% Take time derivatives of vectors as required for kinetic energy terms.
dcm_L1 = ddt(cm_L1);
dcm_L2 = ddt(cm_L2);
dcm_L3 = ddt(cm_L3);

%% Define kinetic energies.
T_L1  = 0.5*m_L1*dot(dcm_L1, dcm_L1) + 0.5* I_L1 * dq1^2;
T_L2  = 0.5*m_L2*dot(dcm_L2, dcm_L2) + 0.5* I_L2 * (dq1 + dq2)^2;
T_L3  = 0.5*m_L3*dot(dcm_L3, dcm_L3) + 0.5* I_L3 * (dq1 + dq3 + dq3)^2;

%% Define Potential energies.
jhat = [0; 1];

V_L1 = m_L1*g*dot(cm_L1, jhat);
V_L2 = m_L2*g*dot(cm_L2, jhat);
V_L3 = m_L3*g*dot(cm_L3, jhat);

% Sum kinetic energy  and potential energy terms
T = T_L1 + T_L2 + T_L3;
V = V_L1 + V_L2 + V_L3; 
       
% Calculate rcm, the location of the center of mass
com_robot = [ (m_L1*cm_L1(1) + m_L2*cm_L2(1) + m_L3*cm_L3(1))/(m_L1+m_L2+m_L3);
              (m_L1*cm_L1(2) + m_L2*cm_L2(2) + m_L3*cm_L3(2))/(m_L1+m_L2+m_L3)];
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

% Write a function to evaluate the X and Y coordinates
COM = [com_robot; dcom_robot]; % Concatenate x and y coordinates and speeds of center of mass in array

%% Save files
% Make a folder
folder = 'model';
if ~exist(folder, 'dir')
    mkdir(folder);
end
z = [q;dq];

% save function in folder
cd(folder);
matlabFunction(E, 'file', 'energy_arm','var',{z});

matlabFunction(A, 'file', 'A_arm', 'var', {z});
matlabFunction(gravity, 'file','grav_arm', 'var', {z});
matlabFunction(coriolis, 'file','coriolis_arm', 'var', {z});

matlabFunction(keypoints, 'file','keypoints_arm', 'var',{z});
matlabFunction(keypoints_vel, 'file','keypoints_vel_arm', 'var',{z});
matlabFunction(COM, 'file','CoM_arm', 'var',{z});
matlabFunction(keypoints_J, 'file','J_arm', 'var',{z});
cd ('../')
