function COM = CoM_arm(in1)
%COM_ARM
%    COM = COM_ARM(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    09-Feb-2024 09:57:03

dq1 = in1(4,:);
dq2 = in1(5,:);
dq3 = in1(6,:);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = cos(q1);
t3 = sin(q1);
t4 = q1+q2;
t5 = cos(t4);
t6 = q3+t4;
t7 = sin(t4);
t12 = t2.*(7.7e+1./6.5e+2);
t13 = t3.*(7.7e+1./6.5e+2);
t8 = cos(t6);
t9 = sin(t6);
t10 = t5.*(3.0./6.5e+1);
t11 = t7.*(3.0./6.5e+1);
t14 = t8.*(3.0./3.25e+2);
t15 = t9.*(3.0./3.25e+2);
t16 = t10+t12+t14;
t17 = t11+t13+t15;
COM = [t16;t17;-dq2.*(t11+t15)-dq3.*t9.*(3.0./3.25e+2)-dq1.*t17;dq2.*(t10+t14)+dq1.*t16+dq3.*t14];