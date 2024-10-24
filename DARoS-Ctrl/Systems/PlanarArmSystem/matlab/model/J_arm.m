function keypoints_J = J_arm(in1)
%J_ARM
%    KEYPOINTS_J = J_ARM(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    09-Feb-2024 09:57:03

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = cos(q1);
t3 = sin(q1);
t4 = q1+q2;
t5 = cos(t4);
t6 = q3+t4;
t7 = sin(t4);
t10 = t2.*(7.0./2.5e+1);
t11 = t3.*(7.0./2.5e+1);
t8 = cos(t6);
t9 = sin(t6);
t12 = -t11;
t13 = t5.*(3.0./1.0e+1);
t14 = t7.*(3.0./1.0e+1);
t15 = -t14;
t16 = t8.*(9.0./5.0e+1);
t17 = t9.*(9.0./5.0e+1);
t18 = -t17;
keypoints_J = reshape([t12,t10,t12+t15,t10+t13,t12+t15+t18,t10+t13+t16,0.0,0.0,t15,t13,t15+t18,t13+t16,0.0,0.0,0.0,0.0,t18,t16],[6,3]);