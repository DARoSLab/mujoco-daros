restoredefaultpath
clear all

%% Paramter preparation
syms x y q_hip q_knee real 
syms dx dy dq_hip dq_knee real 
syms ddx ddy ddq_hip ddq_knee real 

syms l_thigh l_shank real
% l_thigh = 0.345;
% l_shank = 0.355;

m_body = 15;

g = 9.81;

q   = [x; y; q_hip; q_knee];      % generalized coordinates
dq   = [dx; dy; dq_hip; dq_knee];                 
ddq   = [ddx; ddy; ddq_hip; ddq_knee];                     
ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; 

%% Define vectors to key points.
pelvis   = [x; y];         % floating base
knee = pelvis + [l_thigh*sin(q_hip); -l_thigh*cos(q_hip)];
ankle = knee + [l_shank*sin(q_hip + q_knee); -l_shank*cos(q_hip + q_knee)];

keypoints = [pelvis knee ankle]; 


J_ankle = jacobian(ankle,q);

J_contact = J_ankle;

% Compute Jacobian
num_key_pts = size(keypoints,2);
dim_q = size(q,1);

for i =1:num_key_pts
    keypoints_vel(:,i) = ddt(keypoints(:,i));
    J = jacobian(keypoints(:,i),q);
    keypoints_J(2*i-1:2*i,:) = J;
end

%% Save files
z = [q;dq(1:2)];
p = [l_thigh; l_shank];

matlabFunction(keypoints, 'file','keypoints_biped', 'var',{z, p});
% matlabFunction(keypoints_vel, 'file','keypoints_vel_biped', 'var',{z});
matlabFunction(J_contact, 'file','J_biped', 'var',{z, p});

