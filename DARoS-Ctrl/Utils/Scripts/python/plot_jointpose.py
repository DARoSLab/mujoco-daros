import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
import csv
CONFIG_PATH = "../../../build/"
file_path = os.path.join(CONFIG_PATH, "jointposelog.csv")
if __name__ =='__main__':
    with open(file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        data = list(reader)

    x = [float(row[0]) for row in data]
    xd = [float(row[2]) for row in data]
    plt.subplot(2, 1, 1)
    plt.plot(x)
    # plt.plot(xd, marker='o', linestyle='-')
    plt.title('knee joint position')
    plt.grid(True)
    plt.subplot(2, 1, 2)
    plt.plot(xd)
    plt.title('knee joint velocity')
    plt.grid(True)
    plt.show()