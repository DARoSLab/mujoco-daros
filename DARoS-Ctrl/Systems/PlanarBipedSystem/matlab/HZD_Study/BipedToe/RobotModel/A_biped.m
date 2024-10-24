function A = A_biped(in1)
%A_BIPED
%    A = A_BIPED(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    11-Jul-2024 13:47:35

q_ankle_l = in1(10,:);
q_ankle_r = in1(6,:);
q_hip_l = in1(8,:);
q_hip_r = in1(4,:);
q_knee_l = in1(9,:);
q_knee_r = in1(5,:);
q_toe_l = in1(11,:);
q_toe_r = in1(7,:);
th = in1(3,:);
t2 = cos(q_ankle_l);
t3 = cos(q_ankle_r);
t4 = cos(q_knee_l);
t5 = cos(q_knee_r);
t6 = cos(q_toe_l);
t7 = cos(q_toe_r);
t8 = cos(th);
t9 = sin(q_ankle_l);
t10 = sin(q_ankle_r);
t11 = sin(q_toe_l);
t12 = sin(q_toe_r);
t13 = sin(th);
t14 = q_ankle_l+q_knee_l;
t15 = q_ankle_r+q_knee_r;
t16 = q_ankle_l+q_toe_l;
t17 = q_ankle_r+q_toe_r;
t18 = q_hip_l+th;
t19 = q_hip_r+th;
t20 = t8.*3.0;
t21 = t13.*3.0;
t22 = cos(t14);
t23 = cos(t15);
t24 = cos(t16);
t25 = cos(t17);
t26 = cos(t18);
t27 = cos(t19);
t28 = q_toe_l+t14;
t29 = q_toe_r+t15;
t30 = sin(t14);
t31 = sin(t15);
t32 = q_knee_l+t18;
t33 = q_knee_r+t19;
t34 = sin(t18);
t35 = sin(t19);
t40 = t14+t18;
t41 = t15+t19;
t56 = t6.*1.2e-4;
t57 = t7.*1.2e-4;
t58 = t6.*6.0e-5;
t59 = t7.*6.0e-5;
t60 = t11.*1.92e-4;
t61 = t12.*1.92e-4;
t62 = t11.*9.6e-5;
t63 = t12.*9.6e-5;
t68 = t9.*3.264e-3;
t69 = t10.*3.264e-3;
t74 = t4.*4.408e-1;
t75 = t5.*4.408e-1;
t76 = t4.*2.204e-1;
t77 = t5.*2.204e-1;
t78 = t2.*1.496e-2;
t79 = t3.*1.496e-2;
t80 = t2.*7.48e-3;
t81 = t3.*7.48e-3;
t84 = t9.*6.528e-3;
t85 = t10.*6.528e-3;
t36 = cos(t28);
t37 = cos(t29);
t38 = cos(t32);
t39 = cos(t33);
t42 = sin(t32);
t43 = sin(t33);
t44 = -t20;
t45 = -t21;
t46 = cos(t40);
t47 = cos(t41);
t48 = t18+t28;
t49 = t19+t29;
t50 = sin(t40);
t51 = sin(t41);
t64 = t26.*(8.7e+1./5.0e+1);
t65 = t27.*(8.7e+1./5.0e+1);
t66 = t34.*(8.7e+1./5.0e+1);
t67 = t35.*(8.7e+1./5.0e+1);
t82 = -t68;
t83 = -t69;
t86 = t24.*1.36e-3;
t87 = t25.*1.36e-3;
t88 = t24.*6.8e-4;
t89 = t25.*6.8e-4;
t90 = t30.*3.648e-3;
t91 = t31.*3.648e-3;
t92 = -t84;
t93 = -t85;
t106 = t22.*1.672e-2;
t107 = t23.*1.672e-2;
t108 = t22.*8.36e-3;
t109 = t23.*8.36e-3;
t112 = t30.*7.296e-3;
t113 = t31.*7.296e-3;
t134 = t58+t62+3.029e-3;
t135 = t59+t63+3.029e-3;
t52 = sin(t48);
t53 = sin(t49);
t54 = cos(t48);
t55 = cos(t49);
t70 = t38.*(2.9e+1./5.0e+1);
t71 = t39.*(2.9e+1./5.0e+1);
t72 = t42.*(2.9e+1./5.0e+1);
t73 = t43.*(2.9e+1./5.0e+1);
t98 = t46.*(1.1e+1./5.0e+2);
t99 = t47.*(1.1e+1./5.0e+2);
t100 = t46.*(6.0./6.25e+2);
t101 = t47.*(6.0./6.25e+2);
t102 = t50.*(1.1e+1./5.0e+2);
t103 = t51.*(1.1e+1./5.0e+2);
t104 = t50.*(6.0./6.25e+2);
t105 = t51.*(6.0./6.25e+2);
t110 = -t90;
t111 = -t91;
t114 = t36.*1.52e-3;
t115 = t37.*1.52e-3;
t116 = t36.*7.6e-4;
t117 = t37.*7.6e-4;
t120 = -t112;
t121 = -t113;
t136 = t88+t134;
t137 = t89+t135;
t140 = t56+t60+t80+t82+t88+8.9908e-3;
t141 = t57+t61+t81+t83+t89+8.9908e-3;
t94 = t54./5.0e+2;
t95 = t55./5.0e+2;
t96 = t52./5.0e+2;
t97 = t53./5.0e+2;
t118 = -t104;
t119 = -t105;
t138 = t116+t136;
t139 = t117+t137;
t144 = t108+t110+t116+t140;
t145 = t109+t111+t117+t141;
t146 = t56+t60+t76+t78+t86+t92+t108+t110+t116+1.733918e-1;
t147 = t57+t61+t77+t79+t87+t93+t109+t111+t117+1.733918e-1;
t148 = t56+t60+t74+t78+t86+t92+t106+t114+t120+7.465928e-1;
t149 = t57+t61+t75+t79+t87+t93+t107+t115+t121+7.465928e-1;
t122 = t96+t100+t102;
t123 = t97+t101+t103;
t124 = t94+t98+t118;
t125 = t95+t99+t119;
t126 = t72+t122;
t127 = t73+t123;
t128 = t70+t124;
t129 = t71+t125;
t130 = t66+t126;
t131 = t67+t127;
t132 = t64+t128;
t133 = t65+t129;
t142 = t45+t130+t131;
t143 = t44+t132+t133;
A = reshape([2.9e+1,0.0,t143,t133,t129,t125,t95,t132,t128,t124,t94,0.0,2.9e+1,t142,t131,t127,t123,t97,t130,t126,t122,t96,t143,t142,t56+t57+t60+t61+t74+t75+t78+t79+t86+t87+t92+t93+t106+t107+t114+t115+t120+t121+2.4931696,t149,t147,t145,t139,t148,t146,t144,t138,t133,t131,t149,t57+t61+t75+t79+t87+t93+t107+t115+t121+7.466648e-1,t147,t145,t139,0.0,0.0,0.0,0.0,t129,t127,t147,t147,t57+t61+t79+t87+t93+1.734638e-1,t141,t137,0.0,0.0,0.0,0.0,t125,t123,t145,t145,t141,t57+t61+9.0628e-3,t135,0.0,0.0,0.0,0.0,t95,t97,t139,t139,t137,t135,3.101e-3,0.0,0.0,0.0,0.0,t132,t130,t148,0.0,0.0,0.0,0.0,t56+t60+t74+t78+t86+t92+t106+t114+t120+7.466648e-1,t146,t144,t138,t128,t126,t146,0.0,0.0,0.0,0.0,t146,t56+t60+t78+t86+t92+1.734638e-1,t140,t136,t124,t122,t144,0.0,0.0,0.0,0.0,t144,t140,t56+t60+9.0628e-3,t134,t94,t96,t138,0.0,0.0,0.0,0.0,t138,t136,t134,3.101e-3],[11,11]);
