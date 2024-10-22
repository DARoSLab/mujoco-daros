close all
clear all
clc
%%


m = 0.7; %kg
xlim = [-0.5, 0.5];
ylim = [-0.3, 0.3];

q = [0, 0, -1.3, 3.6].';


%% sim
gamma = 0.7;
q_trj_dyn = q;
q_trj_pos = q;

dt = 0.001;
num_step = 500;

dyn_sim = false;
    
for i = 1:num_step
    ddq = [0;-9.81];
    q_trj_dyn(3:4, i+1) = q_trj_dyn(3:4, i) + ddq*dt;
    
    % collision check (x)
    if(q_trj_dyn(1,i)<xlim(1))
        q_trj_dyn(3,i+1) = max(q_trj_dyn(3,i+1), -gamma*q_trj_dyn(3,i+1));
    elseif(q_trj_dyn(1,i) > xlim(2))
        q_trj_dyn(3,i+1) = min(q_trj_dyn(3,i+1), -gamma*q_trj_dyn(3,i+1));
    end
    
    % collision check (y)
    if(q_trj_dyn(2,i)<ylim(1))
        q_trj_dyn(4,i+1) = max(q_trj_dyn(4,i+1), -gamma*q_trj_dyn(4,i+1));
    elseif(q_trj_dyn(2,i) > ylim(2))
        q_trj_dyn(4,i+1) = min(q_trj_dyn(4,i+1), -gamma*q_trj_dyn(4,i+1));
    end
    
    q_trj_dyn(1:2, i+1) = q_trj_dyn(1:2,i) + q_trj_dyn(3:4,i+1)*dt;
end

% Position-based Sim 

k = 10000;
alpha = 1/k;
alpha_tilde = alpha/dt^2;    
mu = 0.009;
for i = 1:num_step
%     ddq = [0;-9.81];
    ddq = [0;0];
    
    q_sub = q_trj_pos(1:2,i);
    dq_sub = q_trj_pos(3:4,i);
    q_pre = q_sub;
    q_sub = q_sub + dq_sub*dt + ddq*dt^2;
        
    lambda_sum = 0;
    lambda_del = 0;
    % collision check (x)
    if(q_sub(1)<xlim(1))
        c = [q_sub(1) - xlim(1); q_sub(2) - q_trj_pos(2,i)];
        lambda_del = (-c - alpha_tilde*lambda_sum)/(1/m + alpha_tilde);
    elseif(q_sub(1) > xlim(2))
        c = [ q_sub(1)- xlim(2); q_sub(2) - q_trj_pos(2,i)];
        lambda_del = (-c - alpha_tilde*lambda_sum)/(1/m + alpha_tilde);
    end

    % collision check (y)
    if(q_sub(2)<ylim(1)) % hit floor
        c = [q_sub(1) - q_trj_pos(1,i); q_sub(2) - ylim(1)];
        lambda_del = (-c - alpha_tilde*lambda_sum)/(1/m + alpha_tilde);        
    elseif(q_sub(2) > ylim(2)) % hit ceiling 
        for k = 1:5
            c = q_sub(2) - ylim(2);
            lambda_del = (-c - alpha_tilde*lambda_sum)/(1/m + alpha_tilde);    
            lambda_sum_next = lambda_sum + lambda_del;

            lambda_sum_next = min(lambda_sum_next,0);
            lambda_del = lambda_sum_next - lambda_sum;
            del_c = lambda_del/m;
            q_sub(2) = q_sub(2) + del_c;
            lambda_sum = lambda_sum + lambda_del;
        end
    end
    dq_sub = (q_sub - q_pre)/dt;
    q_pre = q_sub;
    
    q_trj_pos(1:2,i+1) = q_sub;
    q_trj_pos(3:4,i+1) = dq_sub;
end 



%%
figure
hold on
plot(q_trj_dyn(1,:), q_trj_dyn(2,:));
plot(q_trj_pos(1,:), q_trj_pos(2,:));

plot(q_trj_dyn(1,1), q_trj_dyn(2,1),'g.', 'markersize',35,'linewidth',4);
plot(q_trj_dyn(1,end), q_trj_dyn(2,end),'r.', 'markersize',35,'linewidth',4);
plot(q_trj_pos(1,end), q_trj_pos(2,end),'m.', 'markersize',35,'linewidth',4);


line([xlim(1), xlim(1)], [ylim(1), ylim(2)], 'color','k','linewidth',3);
line([xlim(1), xlim(2)], [ylim(2), ylim(2)], 'color','k','linewidth',3);
line([xlim(2), xlim(2)], [ylim(1), ylim(2)], 'color','k','linewidth',3);
line([xlim(2), xlim(1)], [ylim(1), ylim(1)], 'color','k','linewidth',3);