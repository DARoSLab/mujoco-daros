import distutils.util
import os
import subprocess
if subprocess.run('nvidia-smi').returncode:
  raise RuntimeError(
      'Cannot communicate with GPU. '
      'Make sure you are using a GPU Colab runtime. '
      'Go to the Runtime menu and select Choose runtime type.')

# Add an ICD config so that glvnd can pick up the Nvidia EGL driver.
# This is usually installed as part of an Nvidia driver package, but the Colab
# kernel doesn't install its driver via APT, and as a result the ICD is missing.
# (https://github.com/NVIDIA/libglvnd/blob/master/src/EGL/icd_enumeration.md)
NVIDIA_ICD_CONFIG_PATH = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
if not os.path.exists(NVIDIA_ICD_CONFIG_PATH):
  with open(NVIDIA_ICD_CONFIG_PATH, 'w') as f:
    f.write("""{
    "file_format_version" : "1.0.0",
    "ICD" : {
        "library_path" : "libEGL_nvidia.so.0"
    }
}
""")

# Configure MuJoCo to use the EGL rendering backend (requires GPU)
print('Setting environment variable to use GPU rendering:')
# %env MUJOCO_GL=egl

try:
  print('Checking that the installation succeeded:')
  import mujoco
  mujoco.MjModel.from_xml_string('<mujoco/>')
except Exception as e:
  raise e from RuntimeError(
      'Something went wrong during installation. Check the shell output above '
      'for more information.\n'
      'If using a hosted Colab runtime, make sure you enable GPU acceleration '
      'by going to the Runtime menu and selecting "Choose runtime type".')

print('Installation successful.')

# Tell XLA to use Triton GEMM, this improves steps/sec by ~30% on some GPUs
xla_flags = os.environ.get('XLA_FLAGS', '')
xla_flags += ' --xla_gpu_triton_gemm_any=True'
os.environ['XLA_FLAGS'] = xla_flags


import time
import itertools
import numpy as np
from typing import Callable, NamedTuple, Optional, Union, List
# import matplotlib.pyplot as plt

# More legible printing from numpy.
np.set_printoptions(precision=3, suppress=True, linewidth=100)

#@title Import MuJoCo, MJX, and Brax
from datetime import datetime
from etils import epath
import functools
from typing import Any, Dict, Sequence, Tuple, Union
import os
# from ml_collections import config_dict


import mujoco
# from mujoco import mjx

# import jax
# from jax import numpy as jp
# import numpy as np
# from flax.training import orbax_utils
# from flax import struct
# from matplotlib import pyplot as plt
# import mediapy as media
# from orbax import checkpoint as ocp

# from brax import base
# from brax import envs
# from brax import math
# from brax.base import Base, Motion, Transform
# from brax.envs.base import Env, PipelineEnv, State
# from brax.mjx.base import State as MjxState
# from brax.training.agents.ppo import train as ppo
# from brax.training.agents.ppo import networks as ppo_networks
# from brax.io import html, mjcf, model


mj_model = mujoco.MjModel.from_xml_path("../Robots/MiniArm/miniArm.xml")
mj_data = mujoco.MjData(mj_model)

duration = 0.01

kp = 250
kd = 3.5
desired_position = np.array([0.0, -1.0, 0.0, 1.5, -1.0, 0.0])  # Example desired position


while mj_data.time < duration:# Calculate joint position error
  print(mj_data.qpos)
  position_error = desired_position - mj_data.qpos

  # Calculate joint velocity
  velocity = mj_data.qvel

  # Calculate control signal
  control_signal = kp * position_error - kd * velocity

  # Apply control signal to joints
  mj_data.ctrl = control_signal

  mujoco.mj_step(mj_model, mj_data)
  