import os
import sys
import numpy as np
import yaml
import argparse
import matplotlib.pyplot as plt
import multiprocessing as mp
CONFIG_PATH = "../../../config/"
EXTERNAL_FORCE_PATH = os.path.join(CONFIG_PATH, "external_force_params")

def playback(exp_name, exp_id):
    EXP_EXTERNAL_FORCE_PATH = os.path.join(EXTERNAL_FORCE_PATH, exp_name)

    with open(os.path.join(EXP_EXTERNAL_FORCE_PATH, "{}.yaml".format(exp_id)), "r") as f:
        external_force_param = yaml.safe_load(f)
        f.close()
    with open(os.path.join(EXTERNAL_FORCE_PATH, "{}.yaml".format(exp_id)), "w") as f:
        yaml.dump(external_force_param, f, default_flow_style = None)
        f.close()

    os.system("./../run_pat_sim.sh {} s f ".format(exp_id))
if __name__ =='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--exp-name', type=str, required=True)
    parser.add_argument('--exp-id', type=int, required=False, default=0)
    args = parser.parse_args()
    playback(args.exp_name, args.exp_id)
