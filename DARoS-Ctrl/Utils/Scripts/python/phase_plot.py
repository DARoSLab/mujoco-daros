import numpy as np
import matplotlib.pyplot as plt
import math
def getDesiredCOMState(t, com_pos_ini,  com_vel_ini, stance_foot_loc_ini, com_height=0.42):
    com_state = [0, 0, 0, 0]
    omega_ = math.sqrt(9.81/com_height)
    for i in range(2):
        A = ((com_pos_ini[i] - stance_foot_loc_ini[i]) + com_vel_ini[i]/omega_)/2.
        B = ((com_pos_ini[i] - stance_foot_loc_ini[i]) - com_vel_ini[i]/omega_)/2.
        com_state[i] = A * math.exp(omega_ * t) \
        + B * math.exp(-omega_ * t) + stance_foot_loc_ini[i]
        com_state[i+2] = omega_ * ( A * math.exp(omega_ * t) \
            - B * math.exp(-omega_ * t) )
    return com_state
def get_foot_landing_loc(switch_state, com_height=0.42):
    omega_ = math.sqrt(9.81/com_height)
    kappa_ = [-0.069698, -0.069698]
    t_prime_ = [0.193822, 0.193822 ]
    target_loc = [0, 0]
    for i in range(2):
        exp_weight = 1.0 / math.tanh(omega_ * t_prime_[i])
        target_loc[i] = switch_state[i] + (switch_state[i+2]/omega_) * exp_weight \
            - kappa_[i]*switch_state[i]
    return target_loc
def unroll_steps(com_pos, com_vel, stance_foot_loc, n=1):
    fig,a =  plt.subplots(2,2)
    for i in range(n):
        ts = np.linspace(0.0, 0.33, 10).tolist()
        fn = lambda t : getDesiredCOMState(t, com_pos,  com_vel, stance_foot_loc)
        com_states = np.array(list(map(fn, ts)))
        com_pos = com_states[-1, :2]
        com_vel = com_states[-1, 2:]
        stance_foot_loc = get_foot_landing_loc(com_states[-1])
        a[0][0].plot(com_states[:, 0], com_states[:, 2])
        a[0][0].plot(stance_foot_loc[0], 0, 'o')
        a[0][1].plot(com_states[:, 1], com_states[:, 3])
        a[0][1].plot(stance_foot_loc[1], 0, 'o')
        a[1][0].plot(ts,com_states[:, 2])
        a[1][0].plot([0.193822, 0.193822 ],[-0.1, 0.1])
        a[1][1].plot(ts,com_states[:, 3])
        a[1][1].plot([0.193822, 0.193822 ],[-0.1, 0.1])

    # a[0][0].xlim(-0.1, 0.1)
    # a[0][0].ylim(-0.15, 0.15)
    plt.show()
if __name__ == "__main__":
    com_pos = [-0.022582, 0.0105793]
    com_vel = [0.05, -0.0192106]
    stance_foot_loc = [-0.0125695, 0.0266629, 0.00203595]
    unroll_steps(com_pos, com_vel, stance_foot_loc, n=10)
