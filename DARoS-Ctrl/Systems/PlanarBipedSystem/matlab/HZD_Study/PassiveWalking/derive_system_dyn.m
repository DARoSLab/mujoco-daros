restoredefaultpath
clear all

%% Paramter preparation
syms l M m th phi real 
syms dth dphi real 
syms ddth ddphi real 
syms g real

q   = [th; phi];      % generalized coordinates
dq   = [dth; dphi];                 
ddq   = [ddth; ddphi];
                     
ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; 

%% Define vectors to key points.
body   = [-l*sin(th); l*cos(th)];         % floating base
foot = body + [-l*sin(phi-th); -l*cos(phi-th)];

%% Define vector for CoM
cm_body = body;
cm_foot = foot;
                
% Take time derivatives of vectors as required for kinetic energy terms.
dcm_body = ddt(cm_body);
dcm_foot = ddt(cm_foot);

keypoints = [cm_body cm_foot]; 


% Compute Jacobian
num_key_pts = size(keypoints,2);
dim_q = size(q,1);

for i =1:num_key_pts
    keypoints_vel(:,i) = ddt(keypoints(:,i));
    J = jacobian(keypoints(:,i),q);
    dJ = reshape( ddt(J(:)) , size(J) );
    keypoints_J(2*i-1:2*i,:) = J;
    keypoints_Jdot(2*i-1:2*i, :) = dJ;
end

%% Define kinetic energies.
T_body  = simplify(0.5*M*dot(dcm_body, dcm_body));
T_foot  = simplify(0.5*m*dot(dcm_foot, dcm_foot));

%% Define Potential energies.
jhat = [0; 1];

V_body = M*g*dot(cm_body, jhat);
V_foot = m*g*dot(cm_foot, jhat);

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
gravity = simplify(jacobian(V, q)');
coriolis = simplify( eom - gravity - A*ddq);

%%% Rearrange Equations of Motion. 
A = jacobian(eom,ddq);
b = A*ddq - eom;

A
coriolis
gravity

%%
% J_foot = keypoints_J(3:4,:); 
% Del = simplify(eye(2) - inv(A)*J_foot.'*inv(J_foot* inv(A) *J_foot.')*J_foot )
% 
% alpha = pi/2 + (phi - th);
% rot = [cos(alpha), - sin(alpha);
%        sin(alpha), cos(alpha)];
% J_contact = simplify(rot*J_foot);   
% J_x = J_contact(1,:)
% J_y = J_contact(2,:)
% 
% Del_2 = simplify(eye(2) - inv(A)*J_x.'*inv(J_x* inv(A) *J_x.')*J_x )
% Del_1 = simplify(eye(2) - inv(A)*J_y.'*inv(J_y* inv(A) *J_y.')*J_y )

%% Save files
z = [q;dq];
p = [M; m; l; g];

name = "passive";
matlabFunction(E, 'file', 'energy_'+name,'var',{z, p});

matlabFunction(A, 'file', 'A_'+name, 'var', {z,p});
matlabFunction(gravity, 'file','grav_'+name, 'var', {z,p});
matlabFunction(coriolis, 'file','coriolis_'+name, 'var', {z,p});

matlabFunction(keypoints, 'file','keypoints_'+name, 'var',{z,p});
matlabFunction(keypoints_vel, 'file','keypoints_vel_'+name, 'var',{z,p});
matlabFunction(keypoints_J, 'file','J_'+name, 'var',{z, p});
matlabFunction(keypoints_Jdot, 'file','Jdot_'+name, 'var',{z, p});

