import numpy as np
x_trim = np.array([[0.000000, 0.000000, -100.000000, 24.968741, 0.000000, 1.249786, 0.999687, 0.000000, 0.025003, 0.000000, 0.000000, 0.000000, 0.000000]]).T
u_trim = np.array([[-0.124784, 0.001832, -0.000325, 0.676723]]).T
Va_trim = 25.000000
alpha_trim = 0.050012
theta_trim = 0.050011
a_phi1 = 22.628851
a_phi2 = 130.883678
a_theta1 = 5.294738
a_theta2 = 99.947422
a_theta3 = -36.112390
a_V1 = 0.281681
a_V2 = 8.143746
a_V3 = 9.800000
A_lon = np.array([
    [0.000000, 0.000000, 0.000000, 0.000000, -0.049990],
    [0.000000, 0.000000, 0.000000, 0.000000, 0.998750],
    [-1.249786, 24.968741, 0.000000, 1.000051, 0.000000],
    [0.000000, 0.000000, 0.000000, 0.000000, -24.999996],
    [0.000000, 0.000000, 0.000000, 0.000000, 0.000000]])
B_lon = np.array([
    [-0.138397, 8.143746],
    [-2.586184, 0.000000],
    [-36.112390, 0.000000],
    [0.000000, 0.000000],
    [0.000000, 0.000000]])
A_lat = np.array([
    [0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.000000, 0.000000, 0.000000, 1.000000, -0.000000],
    [-24.968741, 0.000000, 0.000000, 0.050052, 1.001252],
    [0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.000000, 0.000000, 0.000000, 0.000000, 0.000000]])
B_lat = np.array([
    [1.486172, 3.764969],
    [130.883678, -1.796374],
    [5.011735, -24.881341],
    [0.000000, 0.000000],
    [0.000000, 0.000000]])
Ts = 0.010000