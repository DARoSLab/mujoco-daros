import os
import sys
import numpy as np
import yaml
import argparse
import matplotlib.pyplot as plt
import multiprocessing as mp
CONFIG_PATH = "../../../config/"
EXTERNAL_FORCE_PATH = os.path.join(CONFIG_PATH, "external_force_params")
EXTERNAL_FORCE_PATH_COPY = os.path.join(CONFIG_PATH, "external_force_params")
EXIT_STATUS_PATH = os.path.join(CONFIG_PATH, "exit_statuses")
RESULT_PATH = os.path.join(CONFIG_PATH, "rmp_experiments")
np.random.seed(487)
RANDOM_FORCES = np.random.uniform(-10, 10, (2000, 3))
RANDOM_DURATIONS = np.random.uniform(0.1, 0.4, 2000)
RANDOM_FORCES[:, 0] = 0
RANDOM_FORCES[:, 2] = 0
SAVE_EXP_EXT_FORCE = False
def run_experiment(exp_id):
    global EXP_EXTERNAL_FORCE_PATH
    external_force_param  = {}
    external_force_param['external_force_time'] = [3]
    external_force_param['external_force_duration'] = [RANDOM_DURATIONS[exp_id].tolist()]
    external_force_param['external_force'] = RANDOM_FORCES[exp_id].tolist()
    #dump force parameter file
    with open(os.path.join(EXTERNAL_FORCE_PATH, "{}.yaml".format(exp_id)), "w") as f:
        yaml.dump(external_force_param, f, default_flow_style = None)
        f.close()

    if SAVE_EXP_EXT_FORCE:
        #save copy for each experiment
        with open(os.path.join(EXP_EXTERNAL_FORCE_PATH, "{}.yaml".format(exp_id)), "w") as f:
            yaml.dump(external_force_param, f, default_flow_style = None)
            f.close()
    #assume experiment failed
    with open(os.path.join(EXIT_STATUS_PATH, "{}.txt".format(exp_id)), "w") as f:
        f.write("0");
        f.close()
    #run experiment s: sim d: disable viz s: supress print
    os.system("./../run_pat_sim.sh {} s d s".format(exp_id))
    #read exit status if the experiment ended sucessfully(0/1).
    with open(os.path.join(EXIT_STATUS_PATH, "{}.txt".format(exp_id)), "r") as f:
        ext = int(f.readline())
        min_distance = float(f.readline())
        f.close()
    external_force_param['external_force'].append(ext)
    external_force_param['external_force'].append(RANDOM_DURATIONS[exp_id])
    external_force_param['external_force'].append(min_distance)

    return external_force_param['external_force']


if __name__ =='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--exp-name', type=str, required=True)
    parser.add_argument('-n', type=int, required=False, default=10)
    parser.add_argument('-np', type=int, required=False, default=1)
    args = parser.parse_args()
    exp_name = args.exp_name + '_{}'.format(args.n)
    EXP_EXTERNAL_FORCE_PATH = os.path.join(EXTERNAL_FORCE_PATH, exp_name)
    if not os.path.isdir(EXTERNAL_FORCE_PATH):
        os.mkdir(EXTERNAL_FORCE_PATH)
    if not os.path.isdir(EXP_EXTERNAL_FORCE_PATH):
        os.mkdir(EXP_EXTERNAL_FORCE_PATH)
    if not os.path.isdir(EXIT_STATUS_PATH):
        os.mkdir(EXIT_STATUS_PATH)
    if not os.path.isdir(RESULT_PATH):
        os.mkdir(RESULT_PATH)

    pool = mp.Pool(mp.cpu_count())
    result = pool.map(run_experiment, range(args.n))

    print("result: ", result)
    result_np = np.array(result)
    reward = np.logical_and(result_np[:, 3]>0, result_np[:, 5]>0.03).mean()
    with open("reward.csv", "w") as f:
        f.write(str(reward))
    np.save(os.path.join(RESULT_PATH, exp_name), result_np)
