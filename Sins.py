#%% Import Libraries
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.signal import find_peaks
import pandas as pd

import os
print(os.getcwd())
#os.chdir("/Users/nyinyia/Documents/09_LSU_GIT/i_PID/Libraries_NNA")
os.chdir("/Users/animeshbangal/Desktop/Python Code")
print(os.listdir())

from PID_controller import PID_controller
from iPID_controller import iPID_controller
from InputShaping import InputShaping
from Closed_loop import Closed_loop
from SpringMassSystem import SpringMassSystem
from PerformanceMetrics import PerformanceMetrics
from sampling import sampling
import seaborn as sns

# %% Simulation Parameters
m = 1.0  # mass (kg)
k = 2.0  # spring constant (N/m)

duration = 35
dt = 0.01
time = np.arange(0, duration, dt)
x0 = 0
v0 = 0
init = [x0, v0]
spring = SpringMassSystem(m, k)
#step_ref = np.ones_like(time) 
#step_ref[:int(1/dt)] = 0
#exp_ref = 1 - np.exp(-0.5*time) 

sin_ref = np.sin(time)

# %% Ziegler–Nichols tuning
Ku = 1.0
T = 4.4
Kp_zn = .6*Ku
Ki_zn = (2*Kp_zn)/T
Kd_zn = (Kp_zn*T)/8
PID_zn = PID_controller(Kp_zn, Ki_zn, Kd_zn)
iPID_zn = iPID_controller(Kp_zn, Ki_zn, Kd_zn)
PID_zn = Closed_loop(spring, PID_zn)
iPID_zn = Closed_loop(spring, iPID_zn)
t, zn_PID = PID_zn.simulate(time, dt, ref=sin_ref, init = init, control_index = 0)
t, zn_iPID = iPID_zn.simulate(time, dt, ref=sin_ref, init = init, control_index = 0)

plt.figure(figsize=(8, 6))
plt.plot(t, sin_ref,'r', label = r'$ x^{*} $',linewidth=2)
plt.plot(t, zn_PID[:,0],'c-', label=r'$ x_{\mathrm{PID}} $', linewidth=2)
plt.plot(t, zn_iPID[:,0],'b-', label=r'$ x_{\mathrm{iPID}} $', linewidth=2)
plt.xlabel(r"$\mathrm{Time} \ \ \mathrm{[s]}$", fontsize = 20)
plt.ylabel(r"$\ x \ \ \mathrm{[m]}$", fontsize = 20)
plt.xlim(0, duration)
plt.legend(fontsize = 20)
plt.tick_params(axis='both', labelsize=20) 
plt.grid()

plt.tight_layout()
plt.show()

