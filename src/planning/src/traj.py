import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

def fit_xy(t, v):
    return v*t+(a_xy/2)*(t**2)

def fit_z(t, v):
    return v*t+(a_z/2)*(t**2)

def fit_pos(t, pos_samples):
    """
    Returns velocities in x, y, z. pos_samples (n, 3)
    """
    x_samples = pos_samples[:, 0]
    y_samples = pos_samples[:, 1]
    z_samples = pos_samples[:, 2]
    paramx, covx = curve_fit(fit_xy, t, x_samples)
    paramy, covy = curve_fit(fit_xy, t, y_samples)
    paramz, covz = curve_fit(fit_z, t, z_samples)
    return np.concatenate((paramx, paramy, paramz))

def xy_intercept(v):
    t_int = 2*v[2]/9.8
    return fit_xy(t_int, v[0]), fit_xy(t_int, v[1])

# getting intercepts
pos_samples = ... # (n, 3) xyz samples yuhhh
a_xy = 0
a_z = -9.8
delta_t = 1/30 # camera frequency
n = pos_samples.shape[0]

t = np.linspace(0, n*delta_t, n)
v_fit = fit_pos(t, pos_samples)

# intercepts
x, y = xy_intercept(v_fit)