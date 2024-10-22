function R = QuaternionMatrixSym(q)
%% =========================Quaternion Rotation============================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% December 15, 2015
%
% Calculate a Rotation Matrix from the given Quaternion in symbolic form.

%if(size(q,2) ~= 4), error('q must be 1-by-4'); end;

% Normalize the quaternion
%qin = q./( sqrt(sum(q.^2,2)) * ones(1,4));
qin = q/norm(q);

% Form the roatation matrix from the quaternion
R = sym(zeros(3,3));

R(1,1) = (1 - 2*q(3)^2-2*q(4)^2);
R(1,2) = 2*(q(2)*q(3) + q(1)*q(4));
R(1,3) = 2*(q(2)*q(4) - q(1)*q(3));

R(2,1) = 2*(q(2)*q(3) - q(1)*q(4));
R(2,2) = (1 - 2*q(2)^2 - 2*q(4)^2);
R(2,3) = 2*(q(3)*q(4) + q(1)*q(2));

R(3,1) = 2*(q(2)*q(4) + q(1)*q(3));
R(3,2) = 2*(q(3)*q(4) - q(1)*q(2));
R(3,3) = (1 - 2*q(2)^2 - 2*q(3)^2);

% Rotate the vector by matrix
R = R'; %[R', zeros(3, 1); zeros(1, 3), 1];