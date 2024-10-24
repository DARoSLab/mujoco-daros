function keypoints = keypoints_biped(in1)
%KEYPOINTS_BIPED
%    KEYPOINTS = KEYPOINTS_BIPED(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    04-Jul-2024 15:09:23

q_hip_l = in1(6,:);
q_hip_r = in1(4,:);
q_knee_l = in1(7,:);
q_knee_r = in1(5,:);
th = in1(3,:);
x = in1(1,:);
y = in1(2,:);
t2 = q_hip_l+th;
t3 = q_hip_r+th;
t4 = cos(t2);
t5 = cos(t3);
t6 = q_knee_l+t2;
t7 = q_knee_r+t3;
t8 = sin(t2);
t9 = sin(t3);
t10 = t4.*(7.0./2.5e+1);
t11 = t5.*(7.0./2.5e+1);
t12 = t8.*(7.0./2.5e+1);
t13 = t9.*(7.0./2.5e+1);
t14 = -t10;
t15 = -t11;
keypoints = reshape([x,y,t13+x,t15+y,t13+x+sin(t7).*(3.0./1.0e+1),t15+y-cos(t7).*(3.0./1.0e+1),t12+x,t14+y,t12+x+sin(t6).*(3.0./1.0e+1),t14+y-cos(t6).*(3.0./1.0e+1),x-sin(th).*(3.0./1.0e+1),y+cos(th).*(3.0./1.0e+1)],[2,6]);