restoredefaultpath
clear all

%% Paramter preparation
syms x y q_hip q_knee q_ankle real 
syms dx dy dq_hip dq_knee dq_ankle real 
syms ddx ddy ddq_hip ddq_knee ddq_ankle real 

l_thigh = 0.345;
l_shank = 0.355;

m_body = 15;

l_heel_x = -0.045;
l_heel_y = -0.06;
l_toe_x = 0.2;

g = 9.81;

q   = [x; y; q_hip; q_knee; q_ankle];      % generalized coordinates
dq   = [dx; dy; dq_hip; dq_knee; dq_ankle];                 
ddq   = [ddx; ddy; ddq_hip; ddq_knee; ddq_ankle];                     
ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; 

%% Define vectors to key points.
pelvis   = [x; y];         % floating base
knee = pelvis + [l_thigh*sin(q_hip); -l_thigh*cos(q_hip)];
ankle = knee + [l_shank*sin(q_hip + q_knee); -l_shank*cos(q_hip + q_knee)];

theta_ankle = q_hip + q_knee + q_ankle;
heel = ankle + [cos(theta_ankle), -sin(theta_ankle); 
                    sin(theta_ankle), cos(theta_ankle)]*[l_heel_x; l_heel_y];
toe = ankle + [cos(theta_ankle), -sin(theta_ankle); 
                    sin(theta_ankle), cos(theta_ankle)]*[l_toe_x; l_heel_y];

keypoints = [pelvis knee ankle heel toe]; 


J_heel = jacobian(heel,q);
J_toe = jacobian(toe,q);

J_contact = [J_heel; J_toe];

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

matlabFunction(keypoints, 'file','keypoints_biped', 'var',{z});
% matlabFunction(keypoints_vel, 'file','keypoints_vel_biped', 'var',{z});
matlabFunction(J_contact, 'file','J_biped', 'var',{z});

