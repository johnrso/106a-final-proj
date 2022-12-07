import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

#pos_samples = ... # (n, 3) xyz samples yuhhh
a_xy = 0
a_z = -9.8
delta_t = 1/30 # camera frequency

def fit(t, a, b, c):
    return a*(t**2)+b*t+c

# def fit_z(t, v):
#     return v*t+(a_z/2)*(t**2) + 3

def fit_pos(t, pos_samples):
    """
    Returns velocities in x, y, z. pos_samples (n, 3)
    """
    x_samples = pos_samples[:, 0]
    y_samples = pos_samples[:, 1]
    z_samples = pos_samples[:, 2]
    paramx, covx = curve_fit(fit, t, x_samples)
    paramy, covy = curve_fit(fit, t, y_samples)
    paramz, covz = curve_fit(fit, t, z_samples)
    return [paramx, paramy, paramz]

def detect_bounce(pos_samples):
    if len(pos_samples) >= 3:
        one, two, three = pos_samples[-3:]
        return (one[2]-two[2]) > 0 and three[2] > two[2]
    return False

def xy_intercept(v, z_int):
    a, b, c = v[2]
    c -= z_int
    t_int = np.roots([a, b, c])
    return fit(t_int[0], *v[0]), fit(t_int[0], *v[1])

def sample_from_traj(v):
    t = np.random.uniform()
    x = fit(t, *v[0])
    y = fit(t, *v[1])
    z = fit(t, *v[2])
    return [x, y, z]

# if __name__ == "__main__":
#     # getting intercepts
#     n = pos_samples.shape[0]

#     t = np.linspace(0, n*delta_t, n)
#     v_fit = fit_pos(t, pos_samples)

#     # intercepts
#     x, y = xy_intercept(v_fit)
