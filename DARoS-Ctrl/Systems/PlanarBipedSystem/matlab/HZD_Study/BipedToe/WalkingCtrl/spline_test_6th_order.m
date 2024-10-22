clc
clear all
close all
%%

% Test 6th order variable bazier
syms p [6 1] real
syms a [5 1] real
syms b [4 1] real
syms c [3 1] real
syms d [2 1] real
syms pt t real

a(1) = (1-t)*p(1) + t*p(2);
a(2) = (1-t)*p(2) + t*p(3);
a(3) = (1-t)*p(3) + t*p(4);
a(4) = (1-t)*p(4) + t*p(5);
a(5) = (1-t)*p(5) + t*p(6);

b(1) = (1-t)*a(1) + t*a(2);
b(2) = (1-t)*a(2) + t*a(3);
b(3) = (1-t)*a(3) + t*a(4);
b(4) = (1-t)*a(4) + t*a(5);

c(1) = (1-t)*b(1) + t*b(2);
c(2) = (1-t)*b(2) + t*b(3);
c(3) = (1-t)*b(3) + t*b(4);

d(1) = (1-t)*c(1) + t*c(2);
d(2) = (1-t)*c(2) + t*c(3);

pt = simplify((1-t)*d(1) + t*d(2));

for i=1:6
 diff(pt, p(i))
end

vt = diff(pt,t);
at = diff(vt,t);
%%
% p_i = 0.0; p_e = -0.00; v_i = 0; v_e = 0; a_i = 0; p_m = 0.05;
syms p_i p_e v_i v_e a_i p_m real

eqns(1) =  subs(pt, t, 0) == p_i;
eqns(2) =  subs(pt, t, 1) == p_e;
eqns(3) =  subs(vt, t, 0) == v_i;
eqns(4) =  subs(vt, t, 1) == v_e;
eqns(5) =  subs(at, t, 0) == a_i;
eqns(6) =  subs(pt, t, 0.5) == p_m;

S = solve(eqns, p);

pt_subs = simplify(subs(pt,S))
vt_subs = simplify(subs(diff(pt_subs,t),S))
at_subs = simplify(subs(diff(vt_subs,t),S))

% coefficient vector
pt_coeffs = coeffs(pt_subs, t)
vt_coeffs = coeffs(vt_subs, t)
at_coeffs = coeffs(at_subs, t)

figure
subplot(3,1,1)
fplot(pt_subs, [0 1])

subplot(3,1,2)
fplot(vt_subs, [0 1])

subplot(3,1,3)
fplot(at_subs, [0 1])






