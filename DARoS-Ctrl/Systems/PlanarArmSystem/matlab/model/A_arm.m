function A = A_arm(in1)
%A_ARM
%    A = A_ARM(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    09-Feb-2024 09:57:03

q2 = in1(2,:);
q3 = in1(3,:);
t2 = cos(q2);
t3 = cos(q3);
t4 = q2+q3;
t5 = cos(t4);
t6 = t3.*(2.7e+1./2.5e+2);
t7 = t2.*(6.3e+1./2.5e+2);
t8 = t3.*(2.7e+1./5.0e+2);
t9 = t5.*5.04e-2;
t10 = t8+2.16e-2;
t11 = t8+t9+6.16e-2;
t12 = t6+t7+t9+2.741e-1;
A = reshape([t2.*(6.3e+1./1.25e+2)+t5.*(6.3e+1./6.25e+2)+t6+8.204e-1,t12,t11,t12,t6+2.741e-1,t10,t11,t10,1.016e-1],[3,3]);
