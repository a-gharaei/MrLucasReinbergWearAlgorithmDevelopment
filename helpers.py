import operator
from math import cos, sin

import numpy as np
import pyclipper
from matplotlib import pyplot
from mpl_toolkits import mplot3d
from shapely.geometry import LineString, Point, Polygon, LinearRing


def get_vector_miny(array: np.ndarray) -> np.ndarray:
    y = []
    for ind, value in enumerate(array):
        y.append(value[1])
    y_ind, y_value = min(enumerate(y), key=operator.itemgetter(1))
    return array[y_ind]



def get_vector_maxx(array: np.ndarray) -> np.ndarray:
    x = []
    for ind, value in enumerate(array):
        x.append(value[0])
    x_ind, x_value = max(enumerate(x), key=operator.itemgetter(1))
    return array[x_ind]


def polygon_to_plot(polygon):
    x = np.array([])
    y = np.array([])
    for ind, point in enumerate(polygon):
        x = np.append(x, point[0])
        y = np.append(y, point[1])
    x = np.append(x, polygon[0][0])
    y = np.append(y, polygon[0][1])
    return x, y


def polygonIntersection(subj, clip):
    """Given 2 intersecting polygons, it returns the area of the intersection"""
    SCALING_FACTOR = 1000
    pc = pyclipper.Pyclipper()
    pc.AddPath(
        pyclipper.scale_to_clipper(subj, SCALING_FACTOR), pyclipper.PT_SUBJECT, True
    )
    pc.AddPath(
        pyclipper.scale_to_clipper(clip, SCALING_FACTOR), pyclipper.PT_CLIP, True
    )
    solution = pyclipper.scale_from_clipper(
        pc.Execute(
            pyclipper.CT_INTERSECTION, pyclipper.PFT_EVENODD, pyclipper.PFT_EVENODD
        ),
        SCALING_FACTOR,
    )
    pgon = Polygon(solution[0])
    return solution[0], pgon.area


def getIntersectionPolygon(subj):
    clip = subj.cuttingLine()
    subj = subj.vertices[-1]
    SCALING_FACTOR = 1000
    pc = pyclipper.Pyclipper()
    pc.AddPath(
        pyclipper.scale_to_clipper(subj, SCALING_FACTOR), pyclipper.PT_SUBJECT, True
    )
    pc.AddPath(
        pyclipper.scale_to_clipper(clip, SCALING_FACTOR), pyclipper.PT_CLIP, False
    )
    solution = pyclipper.scale_from_clipper(
        pc.Execute(
            pyclipper.CT_INTERSECTION, pyclipper.PFT_EVENODD, pyclipper.PFT_EVENODD
        ),
        SCALING_FACTOR,
    )


def numpyInsertPoint(array1: np.ndarray, array2: np.ndarray, index: int):
    """python insert() but for numpy ndarray

    array1 -> array in which the point is inserted
    array2 -> point which is to be inserted"""
    list1 = array1.tolist()
    list2 = array2.tolist()
    list1.insert(index, list2)
    arr = np.array([np.array(x) for x in list1], dtype=object)
    return arr


def numpyInsertArrayOfPoints(array1: np.ndarray, array2: np.ndarray, index: int):
    """python insert() but for numpy ndarray"""
    list1 = array1.tolist()
    list2 = array2.tolist()
    for ind, point in enumerate(list2):
        list1.insert(index + ind, point)
    arr = np.array([np.array(x) for x in list1], dtype=object)
    return arr


def numpyAppend(array1: np.ndarray, array2: np.ndarray):
    """python append() but for numpy ndarray"""
    list1 = array1.tolist()
    list2 = array2.tolist()
    list1.append(list2)
    arr = np.array([np.array(x) for x in list1], dtype=object)
    return arr


def numpyAppendArrayOfPoints(array1: np.ndarray, array2: np.ndarray):
    """python append() but for numpy ndarray"""
    list1 = array1.tolist()
    list2 = array2.tolist()
    for point in list2:
        list1.append(point)
    arr = np.array([np.array(x) for x in list1], dtype=object)
    return arr


def getPointIndex(array, point):
    for ind, p in enumerate(array):
        if (p == point).all():
            return ind


def mesh_rotation(stl_mesh, theta):
    """Rotates mesh by angle theta around z-axis"""
    length = len(stl_mesh.vectors)
    rot = np.array(
        [[cos(theta), -sin(theta), 0], [sin(theta), cos(theta), 0], [0, 0, 1]]
    )
    for face in range(length):
        for vertice in range(3):
            stl_mesh.vectors[face][vertice] = np.dot(
                rot, stl_mesh.vectors[face][vertice]
            )
    return stl_mesh


def polygon_rotation(x, y, angle):
    """Rotates 2D-Polygon around angle"""
    x_new = [0, 0, 0, 0]
    y_new = [0, 0, 0, 0]
    for i in range(len(x)):
        x_new[i] = np.cos(angle) * x[i] + (-np.sin(angle)) * (y[i])
        y_new[i] = np.sin(angle) * x[i] + np.cos(angle) * (y[i])
    return x_new, y_new


def plot_mesh(mesh):
    """Plots mesh in a 3D-Plot"""
    figure = pyplot.figure()
    axes = figure.add_subplot(projection="3d")
    axes.add_collection3d(mplot3d.art3d.Poly3DCollection(mesh.vectors, alpha=0.7, edgecolors='k'))
    scale = mesh.points.flatten()
    axes.auto_scale_xyz(scale, scale, scale)
    pyplot.show()


def get_angle(v1, v2):
    """Returns angle between vectors v1 and v2"""
    return np.arccos((np.dot(v1, v2)) / (np.linalg.norm(v1) * np.linalg.norm(v2)))


def get_vector(start, end):
    """Returns vector from start point to end point"""
    return np.subtract(end, start)


def unit_vector(vector):
    """Normalizes vector so that |vector| == 1"""
    if np.linalg.norm(vector) == 0:
        raise Exception("Initial vector has no length!")
    return vector / np.linalg.norm(vector)


def rotate_point2D(point: np.ndarray, theta, rot_origin=np.array([0, 0])) -> np.ndarray:
    rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    temp = point
    temp[0] = temp[0] - rot_origin[0]
    temp[1] = temp[1] - rot_origin[1]
    point[0] = np.cos(theta) * temp[0] + -np.sin(theta) * temp[1]
    point[1] = np.sin(theta) * point[0] + np.cos(theta) * point[1]
    point[0] = point[0] + rot_origin[0]
    point[1] = point[1] + rot_origin[1]
    return point


def line_polygon_intersection(polygon: np.ndarray, line: np.ndarray):
    pass


def get_point_data(polygon, int_point):
    point = Point(int_point)
    for ind, test_point in enumerate(polygon):
        line = LineString([polygon[ind - 1], test_point])
        if line.distance(point) < 1e-8:
            return ind - 1, polygon[ind - 1]


def same_list_new_start(array, index):
    temp1 = array[index:]
    temp2 = array[:index]
    return np.concatenate((temp1, temp2), axis=0)


def is_clockwise(polygon : np.ndarray) -> bool:
    ring = LinearRing(polygon)
    if ring.is_ccw:
        return False
    else:
        return True

def get_cutting_edge(vertices : np.ndarray) -> np.ndarray:
    x = []
    y = []
    for ind, value in enumerate(vertices):
        y.append(value[1])
    y_ind, y_value = min(enumerate(y), key=operator.itemgetter(1))
    min_indexes = [i for i, z in enumerate(y) if z == y_value]
    for ind in min_indexes:
        x.append(vertices[ind][0])
    x_ind, x_value = min(enumerate(x), key=operator.itemgetter(1))
    min_ind = min_indexes[x_ind]
    return vertices[min_ind]

    




