#!/usr/bin/python
import matplotlib.pyplot as plt
import numpy as np
import scipy.io as si
import math
import argparse
import os
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
def unroll_steps(lcm_file_path, step_range=range(1, 2), lcm_type='tvr_data_lcmt'):
    tvr_data_dict = mat_to_dict(lcm_file_path, lcm_type)
    fig,a =  plt.subplots(1,2)
    for i in step_range:
        ts = np.linspace(0.0, 0.14, 10).tolist()
        step_one = tvr_data_dict['step_count'] ==i
        fn = lambda t : getDesiredCOMState(t, tvr_data_dict['COM_pos_ini'][step_one][0],  tvr_data_dict['COM_vel_ini'][step_one][0], tvr_data_dict['stance_foot_loc_ini'][step_one][0])
        com_states = np.array(list(map(fn, ts)))
        com_pos2 = com_states[-1, :2]
        com_vel2 = com_states[-1, 2:]
        step_two = tvr_data_dict['step_count'] ==i+1
        fn2 = lambda t : getDesiredCOMState(t, com_states[-1, :2],  com_states[-1, 2:], tvr_data_dict['stance_foot_loc_ini'][step_two][-1])
        com_states2 = np.array(list(map(fn2, ts)))
        com_states = np.concatenate([com_states, com_states2])


        a[0].plot(com_states[:, 0], com_states[:, 2], label='step {} ref'.format(i))
        # a[0].plot(com_states2[:, 0], com_states2[:, 2], label='step 2 {} ref'.format(i))
        step_one[0] = False
        a[0].plot(tvr_data_dict['COM_pos'][step_one, 0], tvr_data_dict['COM_vel'][step_one, 0], label='step {}'.format(i))
        # a[0].plot(tvr_data_dict['COM_pos_des'][step_one, 0], tvr_data_dict['COM_vel_des'][step_one, 0], '*', label='step {} rec ref'.format(i))
        a[0].plot(tvr_data_dict['foot_landing_loc_des'][step_one, 0][-1], 0, 'o', label='step {} Landing ref'.format(i))
        a[0].plot(tvr_data_dict['foot_landing_loc'][step_one, 0][-1], 0, 'x', label='step {} Landing '.format(i))
        a[0].set_title('X')
        a[0].set_xlabel(r'$\mathbf{x}$', fontsize=13)
        a[0].set_ylabel(r'$\mathbf{\dot{x}}$', fontsize=13)
        a[0].legend()

        a[1].plot(com_states[:, 1], com_states[:, 3], label='step {} ref'.format(i))
        # a[1].plot(com_states2[:, 1], com_states2[:, 3], label='step 2 {} ref'.format(i))
        a[1].plot(tvr_data_dict['COM_pos'][step_one, 1], tvr_data_dict['COM_vel'][step_one, 1], label='step {}'.format(i))
        # a[1].plot(tvr_data_dict['COM_pos_des'][step_one, 1], tvr_data_dict['COM_vel_des'][step_one, 1], '*',label='step {} rec ref'.format(i))
        a[1].plot(tvr_data_dict['foot_landing_loc_des'][step_one, 1][-1], 0, 'o', fillstyle=None, label='step {} Landing ref'.format(i))
        a[1].plot(tvr_data_dict['foot_landing_loc'][step_one, 1][-1], 0, 'x', label='step {} Landing '.format(i))
        a[1].set_title('Y')
        a[1].set_xlabel(r'$\mathbf{y}$', fontsize=13)
        a[1].set_ylabel(r'$\mathbf{\dot{y}}$', fontsize=13)
        a[1].legend()
    fig.suptitle('Phase Portrait', fontsize=16)

def plot_wbc_data(lcm_file_path):
    wbc_data_dict = mat_to_dict(lcm_file_path, 'wbc_lcm_data')
    tvr_data_dict = mat_to_dict(lcm_file_path, 'tvr_data_lcmt')
    fig,a =  plt.subplots(3,2)
    fig.suptitle('Foot Position Tracking', fontsize=16)
    a[0][0].plot(wbc_data_dict['lcm_timestamp'], wbc_data_dict['foot_pos_cmd'][:, 0], label='ref')
    a[0][0].plot(wbc_data_dict['lcm_timestamp'], wbc_data_dict['foot_pos'][:, 0], label='measured')
    a[0][0].legend()
    a[0][0].set_title('X')
    a[1][0].plot(wbc_data_dict['lcm_timestamp'], wbc_data_dict['foot_pos_cmd'][:, 1], label='ref')
    a[1][0].plot(wbc_data_dict['lcm_timestamp'], wbc_data_dict['foot_pos'][:, 1], label='measured')
    a[1][0].legend()
    a[1][0].set_title('Y')

    a[2][0].plot(wbc_data_dict['lcm_timestamp'], wbc_data_dict['foot_pos_cmd'][:, 2], label='ref')
    a[2][0].plot(wbc_data_dict['lcm_timestamp'], wbc_data_dict['foot_pos'][:, 2], label='measured')
    a[2][0].legend()
    a[2][0].set_title('Z')
    a[2][0].set_xlabel('time [sec]')

    a[0][1].plot(wbc_data_dict['lcm_timestamp'], wbc_data_dict['foot_pos_cmd'][:, 3], label='ref')
    a[0][1].plot(wbc_data_dict['lcm_timestamp'], wbc_data_dict['foot_pos'][:, 3], label='measured')
    a[0][1].legend()
    a[0][1].set_title('X')
    a[1][1].plot(wbc_data_dict['lcm_timestamp'], wbc_data_dict['foot_pos_cmd'][:, 4], label='ref')
    a[1][1].plot(wbc_data_dict['lcm_timestamp'], wbc_data_dict['foot_pos'][:, 4], label='measured')
    a[1][1].legend()
    a[1][1].set_title('Y')

    a[2][1].plot(wbc_data_dict['lcm_timestamp'], wbc_data_dict['foot_pos_cmd'][:, 5], label='ref')
    a[2][1].plot(wbc_data_dict['lcm_timestamp'], wbc_data_dict['foot_pos'][:, 5], label='measured')
    a[2][1].legend()
    a[2][1].set_title('Z')
    a[2][1].set_xlabel('time [sec]')

    fig2,a2 =  plt.subplots(3,1)
    fig2.suptitle('COM velocity profile', fontsize=16)
    a2[0].plot(tvr_data_dict['lcm_timestamp'][1:, ], tvr_data_dict['COM_vel'][:, 0][1:, ])
    # a2[0].legend()
    a2[0].set_title('X')
    a2[1].plot(tvr_data_dict['lcm_timestamp'], tvr_data_dict['COM_vel'][:, 1])
    # a2[1].legend()
    a2[1].set_title('Y')

    a2[2].plot(tvr_data_dict['lcm_timestamp'], tvr_data_dict['COM_vel'][:, 2])
    # a2[2].legend()
    a2[2].set_title('Z')
    a2[2].set_xlabel('time [sec]')
def plot_mocap_data(lcm_file_path,  N=1000):
    mocap_data_dict = mat_to_dict(lcm_file_path, 'pat_mocap_kin_lcmt')
    fig,a =  plt.subplots(1,3)
    fig.suptitle('Foot Position Tracking', fontsize=16)
    mocap_data_dict['lf_pos_mocap'][np.abs(mocap_data_dict['lf_pos_mocap'])>10] = 0.0
    mocap_data_dict['rf_pos_mocap'][np.abs(mocap_data_dict['rf_pos_mocap'])>10] = 0.0
    mocap_data_dict['rf_pos_mocap'][mocap_data_dict['rf_pos_mocap']<0] = 0.0
    mocap_data_dict['lf_pos_mocap'][mocap_data_dict['rf_pos_mocap']<0] = 0.0

    a[0].plot(mocap_data_dict['lf_pos_kin'][:N, 1], mocap_data_dict['lf_pos_kin'][:N, 0], "*", color='r', label='lf_kin')
    a[0].plot(mocap_data_dict['lf_pos_mocap'][:N, 1], mocap_data_dict['lf_pos_mocap'][:N, 0], "*", color='g', label='lf_mocap')
    a[0].plot(mocap_data_dict['rf_pos_kin'][:N, 1], mocap_data_dict['rf_pos_kin'][:N, 0], "*", color='b', label='rf_kin')
    a[0].plot(mocap_data_dict['rf_pos_mocap'][:N, 1], mocap_data_dict['rf_pos_mocap'][:N, 0], "*", color='y', label='rf_mocap')
    # a[0].plot(mocap_data_dict['lf_pos_mocap_raw'][:N, 1], mocap_data_dict['lf_pos_mocap_raw'][:N, 0], "*", color='b', label='lf_mocap')
    # a[0].plot(mocap_data_dict['rf_pos_mocap_raw'][:N, 1], mocap_data_dict['rf_pos_mocap_raw'][:N, 0], "*", color='b', label='rf_mocap')
    a[0].legend()
    # a[0].set_xlim([-0.08, 0.08])
    # a[0].set_ylim([-0.08, 0.08])
    a[0].set_xlabel(r'$y(m)$')
    a[0].set_ylabel(r'$x(m)$')
    a[1].plot(mocap_data_dict['lf_pos_kin'][:N, 1], mocap_data_dict['lf_pos_kin'][:N, 2], "*", color='r', label='lf_kin')
    a[1].plot(mocap_data_dict['lf_pos_mocap'][:N, 1], mocap_data_dict['lf_pos_mocap'][:N, 2], "*", color='g', label='lf_mocap')
    a[1].plot(mocap_data_dict['rf_pos_kin'][:N, 1], mocap_data_dict['rf_pos_kin'][:N, 2], "*", color='b', label='rf_kin')
    a[1].plot(mocap_data_dict['rf_pos_mocap'][:N, 1], mocap_data_dict['rf_pos_mocap'][:N, 2], "*", color='y', label='rf_mocap')
    # a[1].plot(mocap_data_dict['lf_pos_mocap_raw'][:N, 1], mocap_data_dict['lf_pos_mocap_raw'][:N, 2], "*", color='b', label='lf_mocap_raw')
    # a[1].plot(mocap_data_dict['rf_pos_mocap_raw'][:N, 1], mocap_data_dict['rf_pos_mocap_raw'][:N, 2], "*", color='b', label='rf_mocap_raw')
    a[1].legend()
    # a[1].set_xlim([-0.08, 0.08])
    # a[1].set_ylim([0.2, 0.5])

    a[1].set_xlabel(r'$y(m)$')
    a[1].set_ylabel(r'$z(m)$')
    a[2].plot(mocap_data_dict['lf_pos_kin'][:N, 2], mocap_data_dict['lf_pos_kin'][:N, 0], "*", color='r', label='lf_kin')
    a[2].plot(mocap_data_dict['lf_pos_mocap'][:N, 2], mocap_data_dict['lf_pos_mocap'][:N, 0], "*", color='g', label='lf_mocap')
    a[2].plot(mocap_data_dict['rf_pos_kin'][:N, 2], mocap_data_dict['rf_pos_kin'][:N, 0], "*", color='b', label='rf_kin')
    a[2].plot(mocap_data_dict['rf_pos_mocap'][:N, 2], mocap_data_dict['rf_pos_mocap'][:N, 0], "*", color='y', label='rf_mocap')
    # a[2].plot(mocap_data_dict['lf_pos_mocap_raw'][:N, 2], mocap_data_dict['lf_pos_mocap_raw'][:N, 0], "*", color='b', label='lf_mocap_raw')
    # a[2].plot(mocap_data_dict['rf_pos_mocap_raw'][:N, 2], mocap_data_dict['rf_pos_mocap_raw'][:N, 0], "*", color='b', label='rf_mocap_raw')
    a[2].legend()
    a[2].set_xlabel(r'$z(m)$')
    a[2].set_ylabel(r'$x(m)$')
    # a[2].set_ylim([-0.08, 0.08])
    # a[2].set_xlim([0.2, 0.5])

def mat_to_dict(file, lcm_type):
    lcm_data = si.loadmat(file)[lcm_type]
    lcm_data_dict = {}
    for name in lcm_data.dtype.names:
        if lcm_data[name][0][0].shape[0] == 1:
            lcm_data_dict[name] = lcm_data[name][0][0].reshape(-1)
        else:
            lcm_data_dict[name] = lcm_data[name][0][0]
        lcm_data_dict[name][lcm_data_dict[name]>1e5] = 0.0
    return lcm_data_dict


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--lcm-path', type=str, required=True)
    parser.add_argument('-l', type=int, default=0)
    parser.add_argument('-u', type=int, default=1)
    parser.add_argument('--tvr', type=int, default=0)
    parser.add_argument('--wbc', type=int, default=0)
    parser.add_argument('--mocap', type=int, default=1)
    args = parser.parse_args()
    mat_path = args.lcm_path + '.mat'
    #convert lcm to matlab
    if not os.path.exists(mat_path):
        os.system('bash ~/DARoS/utils/scripts/log_convert.sh {} {}'.format(args.lcm_path, mat_path))
    else:
        print("using already converted .mat file from {}".format(mat_path))
    if(args.tvr>0):
        unroll_steps(mat_path, range(args.l, args.u))
    if(args.wbc>0):
        plot_wbc_data(mat_path)
    if(args.mocap>0):
        plot_mocap_data(mat_path)
    plt.show()
