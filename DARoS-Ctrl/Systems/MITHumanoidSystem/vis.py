import lcm
from tello_MPC_info_lcmt import tello_MPC_info_lcmt
import matplotlib.pyplot as plt
import numpy as np


#interactive mode on
plt.ion()
# the default horizon is 10
horizon = 10
defaultx = np.linspace(0, 0.05 * (horizon), horizon+1)
defaulty = np.zeros(horizon+1)


figure, (axX, axY, axZ, axDX, axDY, axDZ) = plt.subplots(6, 1, figsize=(10, 8))

lineX, = axX.plot(defaultx, defaulty, 'r-')
axX.set_title('Position X')
axX.set_ylim([0, 100])

lineY, = axY.plot(defaultx, defaulty, 'r-') 
axY.set_ylim([-1, 1])
axY.set_title('Position Y')

lineZ, = axZ.plot(defaultx, defaulty, 'r-')
axZ.set_title('Position Z')
axZ.set_ylim([0.5, 0.7])

lineDX, = axDX.plot(defaultx, defaulty, 'r-')
axDX.set_title('Velocity X')
axDX.set_ylim([0, 3])

lineDY, = axDY.plot(defaultx, defaulty, 'r-')
axDY.set_title('Velocity Y')
axDY.set_ylim([-1, 1])

lineDZ, = axDZ.plot(defaultx, defaulty, 'r-')
axDZ.set_title('Velocity Z')
axDZ.set_ylim([-1, 1])


def my_handler(channel, data):
    msg = tello_MPC_info_lcmt.decode(data)
    global defaultx
    global figure
    global horizon
    # Position X
    X = np.array(msg.x)[0:horizon+1]
    lineX.set_ydata(X)
    # Position Y
    Y = np.array(msg.y)[0:horizon+1]
    lineY.set_ydata(Y)
    # Position Z
    Z = np.array(msg.z)[0:horizon+1]
    lineZ.set_ydata(Z)
    # Velocity X
    DX = np.array(msg.dx)[0:horizon+1]
    lineDX.set_ydata(DX)
    # Velocity Y
    DY = np.array(msg.dy)[0:horizon+1]
    lineDY.set_ydata(DY)
    # Velocity Z
    DZ = np.array(msg.dz)[0:horizon+1]
    lineDZ.set_ydata(DZ)

    # refresh canvas
    figure.canvas.draw()
    figure.canvas.flush_events()



lc = lcm.LCM()
subscription = lc.subscribe("mpcInfo", my_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass