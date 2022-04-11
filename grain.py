import operator

import numpy as np
import trimesh
from shapely.geometry import LinearRing


class Grain2D:
    def __init__(self, vertices: np.ndarray, penetration_depth: float):
        # Validation of the received arguments
        assert (
            type(vertices) == np.ndarray
        ), f"Vertices {vertices} is not of type: numpy.ndarray!"
        assert vertices.shape[1] == 2, "The Vertices are not two-dimensional!"
        assert (
            len(vertices) > 2
        ), "There are not enough point to create a grain. Does have at least three!"

        # Assigning default variables
        self.cutting_edge(vertices)
        self.arrange_vertices(vertices)
        self.penetration_depth = penetration_depth

    def arrange_vertices(self, vertices):
        ring = LinearRing(vertices)
        if ring.is_ccw:
            vertices = vertices[::-1]
        for index, vertice in enumerate(vertices):
            if (vertice == self.cutting_edge).all():
                cutting_edge_index = index
                break
        temp1 = vertices[cutting_edge_index:]
        temp2 = vertices[:cutting_edge_index]
        vertices = np.concatenate((temp1, temp2), axis=0)
        self.vertices = np.array([vertices])

    def cutting_edge(self, vertices: np.ndarray) -> np.ndarray:
        y = [value[1] for value in vertices]
        y_ind, y_value = min(enumerate(y), key=operator.itemgetter(1))
        min_indexes = [i for i, z in enumerate(y) if z == y_value]
        x = [vertices[ind][0] for ind in min_indexes]
        x_ind, x_value = min(enumerate(x), key=operator.itemgetter(1))
        min_ind = min_indexes[x_ind]
        self.cutting_edge = vertices[min_ind]


class Grain3D:
    ex = np.array([1, 0, 0])
    ey = np.array([0, 1, 0])
    ez = np.array([0, 0, 1])

    def __init__(self, grain: trimesh.base.Trimesh) -> None:
        # Validation of the received arguments
        assert type(grain) == trimesh.base.Trimesh, "Wrong type of mesh!"
        # Assigning default variables
        self.mesh = grain
        self.vertices = grain.vertices
        self.faces = grain.faces
        self.cutting_edge = self.get_cutting_edge()
        self.bounds = grain.bounds
        self.minx = grain.bounds[0][0]
        self.miny = grain.bounds[0][1]
        self.minz = grain.bounds[0][2]
        self.maxx = grain.bounds[1][0]
        self.maxy = grain.bounds[1][1]
        self.maxz = grain.bounds[1][2]
        self.center = grain.center_mass

    def get_cutting_edge(self):
        z = [value[2] for value in self.vertices]
        z_ind, z_value = min(enumerate(z), key=operator.itemgetter(1))
        return self.vertices[z_ind]
