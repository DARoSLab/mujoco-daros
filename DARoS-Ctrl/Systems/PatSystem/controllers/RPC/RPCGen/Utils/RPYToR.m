function R = RPYToR(rpy)
% rpyToQuat takes earth-fixed sequenial roll-pitch-yaw euler angles to a
% unit quaternion
%   [quat] = rpyToQuat(rpy)
%   quat= [q0,q1,q2,q3] assumed to follow q0 = cos(angle / 2) while
%   [q1,q2,q3] = axis*sin(angle / 2) for angle/axis expression of R
%   note: earth-fixed roll-pitch-yaw is the same as body-fixed
%   yaw-pitch-roll sequence

roll = rpy(1);
pitch = rpy(2);
yaw = rpy(3);


Rz = [cos(yaw), -sin(yaw), 0;...
    sin(yaw), cos(yaw), 0;...
    0, 0, 1];
Ry = [cos(pitch), 0, sin(pitch);...
    0, 1, 0;...
    -sin(pitch), 0, cos(pitch)];
Rx = [1, 0, 0;...
    0, cos(roll), -sin(roll);...
    0, sin(roll), cos(roll)];

R = Rx*Ry*Rz;
% R = [cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(pitch), ];