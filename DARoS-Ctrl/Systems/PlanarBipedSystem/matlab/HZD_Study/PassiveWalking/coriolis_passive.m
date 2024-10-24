function coriolis = coriolis_passive(in1,in2)
%CORIOLIS_PASSIVE
%    CORIOLIS = CORIOLIS_PASSIVE(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    11-Jul-2024 07:21:02

dphi = in1(4,:);
dth = in1(3,:);
l = in2(3,:);
m = in2(2,:);
phi = in1(2,:);
t2 = sin(phi);
t3 = l.^2;
coriolis = [dphi.*m.*t2.*t3.*(dphi.*2.0-dth.*4.0).*(-1.0./2.0);-dth.^2.*m.*t2.*t3];