import random
import numpy as np
from pandas import array
import stl
from math import cos, sin
from matplotlib import pyplot
from mpl_toolkits import mplot3d
import operator


def mesh_rotation(stl_mesh, theta):
    """Rotates mesh by angle theta around z-axis"""
    length = len(stl_mesh.vectors)
    rot = np.array([[cos(theta), -sin(theta), 0],
                   [sin(theta), cos(theta), 0], [0, 0, 1]])
    for face in range(length):
        for vertice in range(3):
            stl_mesh.vectors[face][vertice] = np.dot(
                rot, stl_mesh.vectors[face][vertice])


def polygon_rotation(x, y, angle):
    """Rotates 2D-Polygon around angle"""
    x_new = [0, 0, 0, 0]
    y_new = [0, 0, 0, 0]
    for i in range(len(x)):
        x_new[i] = np.cos(angle) * x[i] + (-np.sin(angle))*(y[i]-5)
        y_new[i] = np.sin(angle)*x[i] + np.cos(angle) * (y[i]-5) + 5
    return x_new, y_new


def plot_mesh(mesh):
    """Plots mesh in a 3D-Plot"""
    figure = pyplot.figure()
    axes = figure.add_subplot(projection='3d')
    axes.add_collection3d(mplot3d.art3d.Poly3DCollection(mesh.vectors))
    scale = mesh.points.flatten()
    axes.auto_scale_xyz(scale, scale, scale)
    pyplot.show()


def get_angle(v1, v2):
    """Returns angle between vectors v1 and v2"""
    return np.arccos((np.dot(v1, v2))/(np.linalg.norm(v1)*np.linalg.norm(v2)))


def get_vector(start, end):
    """Returns vector from start point to end point"""
    return np.subtract(end, start)


def unit_vector(vector):
    """Normalizes vector so that |vector| == 1"""
    return vector / np.linalg.norm(vector)


def get_radial_stress(omega, rho, a, P, r):
    """"Should return radial stress at given point(r, rho) depending on force P, angle of attack omega and included half wedge angle a (Isn't verified)"""
    # factor = 2 * P / r
    # bracket = np.cos(omega) * np.cos(rho) / (2 * a + np.sin(2*a)) + \
    #     np.sin(omega) * np.cos(rho) / (2 * a - np.sin(2*a))
    return 400


def get_radial_stress_2(rho_1, alpha, F, n, r):
    """Different formula to calculate radial stress (Isn't verified)"""
    factor = 2 * F / r
    bracket = (-np.cos(rho_1) / (2 * alpha - np.sin(2*alpha))) - \
        (n * np.cos(np.pi/2 - rho_1) / (2 * alpha + np.sin(2*alpha)))
    return factor * bracket


def get_cutting_edge(x, y):
    """Returns point of the grain with minimal y-value -> coordinates of the cutting edge"""
    y_ce_index, y_ce = min(enumerate(y), key=operator.itemgetter(1))
    x_ce = x[y_ce_index]
    return np.array([x_ce, y_ce])


def make_crack(x, y, cut_edge, rf, pod):
    """Modifies grain by cutting away a triangle at the danger point
    Returns modified x and y values of the grain"""
    rake_face_length = np.linalg.norm(rf)
    crack_length = random.uniform(0.05*rake_face_length, 0.1*rake_face_length)
    crack_direction = np.array([pod[1], -pod[0]]) / \
        np.linalg.norm(np.array([pod[1], -pod[0]]))
    crack = pod + crack_length * crack_direction
    rf_crack_lower = pod - \
        random.uniform(0.1*crack_length, 0.5*crack_length) * \
        (pod/np.linalg.norm(pod))
    rf_crack_upper = pod + \
        random.uniform(0.1*crack_length, 0.5*crack_length) * \
        (pod/np.linalg.norm(pod))
    x_crack = np.array([rf_crack_lower[0], crack[0], rf_crack_upper[0]])
    y_crack = np.array([rf_crack_lower[1], crack[1], rf_crack_upper[1]])
    for index, value in enumerate(x):
        if value == cut_edge[0] and y[index] == cut_edge[1]:
            crack_index = index + 1
            break
    x_mod = np.insert(x, crack_index, x_crack, axis=None)
    y_mod = np.insert(y, crack_index, y_crack, axis=None)
    return x_mod, y_mod
