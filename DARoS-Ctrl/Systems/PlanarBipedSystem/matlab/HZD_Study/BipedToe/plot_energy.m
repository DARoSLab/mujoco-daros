clc
clear all
close all

%%

tau_square_wbc = [8.716e5, 1.0176e6, 1.2846e6, 1.4278e6];
dist_wbc = [0.6031, 0.8360, 1.0726, 1.1748];

delta_t = 1.51;

tau_square_hzd = [4.3e5, 5.502e5, 5.9539e5, 7.22e5];
dist_hzd = [0.62, 0.825, 1.0567, 1.19];

N = length(dist_wbc);
for i = 1:N
    vel_wbc(i) = dist_wbc(i)/delta_t;
    norm_tau_wbc(i) = tau_square_wbc(i)/dist_wbc(i);
    
    vel_hzd(i) = dist_hzd(i)/delta_t;
    norm_tau_hzd(i) = tau_square_hzd(i)/dist_wbc(i);
end

%%
figure('position', [0, 0, 200, 400])
hold on
plot(vel_hzd, norm_tau_hzd, '*-', 'linewidth', 3)
plot(vel_wbc, norm_tau_wbc, '*-', 'linewidth', 2)
axis tight
ylim([3.7e5, 1.5e6])
set(gca,'fontsize',12)
grid on
