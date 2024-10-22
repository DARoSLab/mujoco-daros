clc
clear all
close all
%%

syms a real

eqn = 2*sin(a) + a*cos(a) == -a;

solve(eqn)

t = [-1:0.01: 5];

y1 = 2*sin(t) + t.* cos(t);
y2 = -t;

figure
plot(t, y1, t, y2)

%%
eqns(1) =  subs(pt, t, 0) == p_i;
eqns(2) =  subs(pt, t, 1) == p_e;
eqns(3) =  subs(vt, t, 0) == v_i;
eqns(4) =  subs(vt, t, 1) == v_e;
eqns(5) =  subs(at, t, 0) == a_i;

S = solve(eqns, p);

pt_subs = simplify(subs(pt,S))
vt_subs = simplify(subs(diff(pt_subs,t),S))
at_subs = simplify(subs(diff(vt_subs, t),S))

% coefficient vector
pt_coeffs = coeffs(pt_subs, t);
vt_coeffs = coeffs(vt_subs, t);
at_coeffs = coeffs(at_subs, t);