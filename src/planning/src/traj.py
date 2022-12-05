import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

pos_samples = ... # (n, 3) xyz samples yuhhh
a_xy = 0
a_z = -9.8
delta_t = 1/30 # camera frequency

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

def detect_bounce(pos_samples):
    if len(pos_samples) >= 3:
        one, two, three = pos_samples[-3:]
        return (one[2]-two[2]) > 0 and three[2] > two[2]
    return False

def xy_intercept(v):
    t_int = 2*v[2]/9.8
    return fit_xy(t_int, v[0]), fit_xy(t_int, v[1])

if __name__ == "__main__":
    # getting intercepts
    n = pos_samples.shape[0]

    t = np.linspace(0, n*delta_t, n)
    v_fit = fit_pos(t, pos_samples)

    # intercepts
    x, y = xy_intercept(v_fit)
