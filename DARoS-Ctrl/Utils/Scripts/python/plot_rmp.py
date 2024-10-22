import os
import argparse
import numpy as np
import matplotlib.pyplot as plt

CONFIG_PATH = "../../../config/"
RESULT_PATH = os.path.join(CONFIG_PATH, "rmp_experiments")
if __name__ =='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--exp-name', type=str, required=True)
    args = parser.parse_args()
    force_and_ext_np = np.load(os.path.join(RESULT_PATH, args.exp_name + '.npy'))
    print("Success rate: {}%".format(100*force_and_ext_np[:, 3].mean()))
    print("Success rate collision: {}%".format(100*np.logical_and(force_and_ext_np[:, 3]>0,force_and_ext_np[:, 5]>0.03).mean()))

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    for f in force_and_ext_np:
        if f[3] > 0:
            ax.scatter(f[0], f[1], f[2], marker='o', color='g') #Success
        else:
            ax.scatter(f[0], f[1], f[2], marker='o', color='r')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    fig = plt.figure()
    ax = fig.add_subplot()

    for f in force_and_ext_np:
        if f[3] > 0:
            if f[5] > 0.03:
                ax.scatter(f[4], f[1], marker='*', color='g') #Success
            else:
                ax.scatter(f[4], f[1], marker='*', color='y') #Success
        else:
            ax.scatter(f[4], f[1], marker='*', color='r')
    ax.set_xlabel('Duration (sec)')
    ax.set_ylabel('Lateral Force (sec) ')

    plt.show()
