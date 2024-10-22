classdef FootTrajGen < handle
   properties
       foot_ini
       foot_fin
       mid_height = 0.1;
       ini_time
       swing_duration
       
       x_trj_coeffs
       x_vel_coeffs
       x_acc_coeffs
       
       y_trj_coeffs
       y_vel_coeffs
       y_acc_coeffs
   end
   
   % x traj: 5th order (ini_pos, ini_vel, ini_acc, fin_pos, fin_vel)
   % y traj(vertical): 6th order (ini_pos, ini_vel, ini_acc, mid_pos,
   % fin_pos, fin_vel)
   
   methods
       function obj = genTrajectory(obj, pos_ini, pos_fin, ini_time, swing_duration)
           obj.ini_time = ini_time;
           obj.swing_duration = swing_duration;
           
           % Horizontal
           p_i = pos_ini(1);
           v_i = 0;
           a_i = 0;
           p_e = pos_fin(1);
           v_e = 0;
           obj.x_trj_coeffs = [p_i, v_i, a_i/2, 4*p_e - a_i - 4*p_i - v_e - 3*v_i, a_i/2 - 3*p_e + 3*p_i + v_e + 2*v_i];
           obj.x_vel_coeffs = [v_i, a_i, 12*p_e - 3*a_i - 12*p_i - 3*v_e - 9*v_i, 2*a_i - 12*p_e + 12*p_i + 4*v_e + 8*v_i];
           obj.x_acc_coeffs = [a_i, 24*p_e - 6*a_i - 24*p_i - 6*v_e - 18*v_i, 6*a_i - 36*p_e + 36*p_i + 12*v_e + 24*v_i];

           % Vertical           
           p_i = pos_ini(2);
           v_i = 0;
           a_i = 0;
           p_e = pos_fin(2);
           v_e = 0;
           p_m = obj.mid_height;
           obj.y_trj_coeffs = [p_i, v_i, a_i/2, 32*p_m - 6*p_e - 26*p_i - 2*a_i + v_e - 11*v_i, (5*a_i)/2 + 17*p_e + 47*p_i - 64*p_m - 3*v_e + 18*v_i, 32*p_m - 10*p_e - 22*p_i - a_i + 2*v_e - 8*v_i];
           obj.y_vel_coeffs = [v_i, a_i, 96*p_m - 18*p_e - 78*p_i - 6*a_i + 3*v_e - 33*v_i, 10*a_i + 68*p_e + 188*p_i - 256*p_m - 12*v_e + 72*v_i, 160*p_m - 50*p_e - 110*p_i - 5*a_i + 10*v_e - 40*v_i];
           obj.y_acc_coeffs = [a_i, 192*p_m - 36*p_e - 156*p_i - 12*a_i + 6*v_e - 66*v_i, 30*a_i + 204*p_e + 564*p_i - 768*p_m - 36*v_e + 216*v_i, 640*p_m - 200*p_e - 440*p_i - 20*a_i + 40*v_e - 160*v_i];
       end
       
       function [pos, vel, acc] = getSplinePt(obj, curr_time)
           t = (curr_time - obj.ini_time)/obj.swing_duration;
           t = min(max(0,t),1);
           t_vec = [1, t, t^2, t^3, t^4, t^5];
           pos(1,1) = obj.x_trj_coeffs * t_vec(1:5).';
           pos(2,1) = obj.y_trj_coeffs * t_vec.';
           
           vel(1,1) = obj.x_vel_coeffs * t_vec(1:4).';
           vel(2,1) = obj.y_vel_coeffs * t_vec(1:5).';
           
           
           acc(1,1) = obj.x_acc_coeffs * t_vec(1:3).';
           acc(2,1) = obj.y_acc_coeffs * t_vec(1:4).';
       end
   end
end