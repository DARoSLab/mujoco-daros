function keypoints_J = J_biped(in1)
%J_BIPED
%    KEYPOINTS_J = J_BIPED(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    11-Jul-2024 13:47:40

q_ankle_l = in1(10,:);
q_ankle_r = in1(6,:);
q_hip_l = in1(8,:);
q_hip_r = in1(4,:);
q_knee_l = in1(9,:);
q_knee_r = in1(5,:);
q_toe_l = in1(11,:);
q_toe_r = in1(7,:);
th = in1(3,:);
t2 = q_hip_l+th;
t3 = q_hip_r+th;
t4 = cos(t2);
t5 = cos(t3);
t6 = q_knee_l+t2;
t7 = q_knee_r+t3;
t8 = sin(t2);
t9 = sin(t3);
t10 = cos(t6);
t11 = cos(t7);
t12 = q_ankle_l+t6;
t13 = q_ankle_r+t7;
t14 = sin(t6);
t15 = sin(t7);
t26 = t4.*(1.9e+1./5.0e+1);
t27 = t5.*(1.9e+1./5.0e+1);
t28 = t8.*(1.9e+1./5.0e+1);
t29 = t9.*(1.9e+1./5.0e+1);
t16 = cos(t12);
t17 = cos(t13);
t18 = q_toe_l+t12;
t19 = q_toe_r+t13;
t20 = sin(t12);
t21 = sin(t13);
t30 = t10.*(1.7e+1./5.0e+1);
t31 = t11.*(1.7e+1./5.0e+1);
t32 = t14.*(1.7e+1./5.0e+1);
t33 = t15.*(1.7e+1./5.0e+1);
t22 = sin(t18);
t23 = sin(t19);
t24 = cos(t18);
t25 = cos(t19);
t34 = t16./2.0e+1;
t35 = t17./2.0e+1;
t36 = t16.*(3.0./5.0e+1);
t37 = t17.*(3.0./5.0e+1);
t38 = t20./2.0e+1;
t39 = t21./2.0e+1;
t40 = t20.*(3.0./5.0e+1);
t41 = t21.*(3.0./5.0e+1);
t52 = t16.*(3.0./1.0e+2);
t53 = t17.*(3.0./1.0e+2);
t54 = t16.*(6.0./1.25e+2);
t55 = t17.*(6.0./1.25e+2);
t56 = t20.*(3.0./1.0e+2);
t57 = t21.*(3.0./1.0e+2);
t58 = t20.*(6.0./1.25e+2);
t59 = t21.*(6.0./1.25e+2);
t70 = t28+t32;
t71 = t29+t33;
t72 = t26+t30;
t73 = t27+t31;
t42 = t24./2.5e+1;
t43 = t25./2.5e+1;
t44 = t24./5.0e+1;
t45 = t25./5.0e+1;
t46 = t22./2.5e+1;
t47 = t23./2.5e+1;
t48 = t22./5.0e+1;
t49 = t23./5.0e+1;
t50 = -t36;
t51 = -t37;
t60 = t24.*(3.0./2.0e+2);
t61 = t25.*(3.0./2.0e+2);
t64 = t22.*(3.0./2.0e+2);
t65 = t23.*(3.0./2.0e+2);
t66 = -t58;
t67 = -t59;
t74 = t34+t40+t72;
t75 = t35+t41+t73;
t78 = t54+t56+t70;
t79 = t55+t57+t71;
t62 = -t46;
t63 = -t47;
t68 = -t60;
t69 = -t61;
t76 = t38+t50+t70;
t77 = t39+t51+t71;
t80 = t52+t66+t72;
t81 = t53+t67+t73;
t82 = t42+t48+t78;
t83 = t43+t49+t79;
t84 = t44+t62+t80;
t85 = t45+t63+t81;
t86 = t44+t64+t80;
t87 = t45+t65+t81;
t88 = t48+t68+t78;
t89 = t49+t69+t79;
keypoints_J = reshape([1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,1.0,0.0,0.0,t27,t29,t73,t71,t75,t77,t81,t79,t85,t83,t87,t89,t26,t28,t72,t70,t74,t76,t80,t78,t84,t82,t86,t88,cos(th).*(-1.1e+1./2.0e+1),sin(th).*(-1.1e+1./2.0e+1),0.0,0.0,t27,t29,t73,t71,t75,t77,t81,t79,t85,t83,t87,t89,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t31,t33,t31+t35+t41,t33+t39+t51,t31+t53+t67,t33+t55+t57,t31+t45+t53+t63+t67,t33+t43+t49+t55+t57,t31+t45+t53+t65+t67,t33+t49+t55+t57+t69,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t35+t41,t39+t51,t53+t67,t55+t57,t45+t53+t63+t67,t43+t49+t55+t57,t45+t53+t65+t67,t49+t55+t57+t69,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t45+t63,t43+t49,t45+t65,t49+t69,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t26,t28,t72,t70,t74,t76,t80,t78,t84,t82,t86,t88,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t30,t32,t30+t34+t40,t32+t38+t50,t30+t52+t66,t32+t54+t56,t30+t44+t52+t62+t66,t32+t42+t48+t54+t56,t30+t44+t52+t64+t66,t32+t48+t54+t56+t68,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t34+t40,t38+t50,t52+t66,t54+t56,t44+t52+t62+t66,t42+t48+t54+t56,t44+t52+t64+t66,t48+t54+t56+t68,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t44+t62,t42+t48,t44+t64,t48+t68,0.0,0.0],[28,11]);
