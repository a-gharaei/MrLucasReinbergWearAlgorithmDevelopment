import numpy as np
from typing import *
import stl
import pygeos
import matplotlib.pyplot as plt

# --------------------------  Main classes for geometry/object creation ------------------------------


class Vector:
    """Vector is the most basic geometry class which has all the basic operations needed for
    vector calculation and manipulation that is used in iBrus.
    """
    __value: np.ndarray

    def __init__(self, x: float, y: float, z: float):
        self.value = np.array([x, y, z, 1])

    def transform(self, transform):
        if isinstance(transform, Transform):
            value = np.matmul(transform.value, self.value)
            return Vector(value[0], value[1], value[2])
        else:
            raise Exception('input is not of type Transform')

    def x(self) -> float:
        return self.value[0]

    def y(self) -> float:
        return self.value[1]

    def z(self) -> float:
        return self.value[2]


class Transform:
    """Transform is wrapper class for geometric transformation, including rotation, translation,
    and frame change. It's represented as homogenous transform in a 4x4 matrix."""
    value: np.ndarray

    def __init__(self, matrix: np.ndarray):
        self.value = matrix

    @classmethod
    def from_projection_along_vector_to_x_z_plane(cls, v: Vector):
        if np.abs((v.y() - 0)) >= 1e-12:
            matrix = np.identity(4)
            matrix[0, 1] = -v.x() / v.y()
            matrix[1, 1] = 0
            matrix[2, 1] = -v.z() / v.y()
            return cls(matrix)
        else:
            raise Exception(
                "vector  is almost parallel to x_z plane ")

    @ classmethod
    def identity(cls):
        matrix = np.identity(4)
        return cls(matrix)


class Pose:
    """Pose is a basic geometry properties for all the geometric objects defined in iBrus.
    It is represented as the transform which transforms the basis vectors of
    world coordinate system to the basis vectors of the frame attached to the pose.
    Each pose is defined in a certain frame and has strict hierarchy.
    For example: A grain's pose is defined in tool frame, A tool's pose is defined in global frame.
    A FlatManifold's pose is defined in Workpiece frame,
    A workpiece's pose is defined in global frame.
    """
    value: Transform

    def __init__(self, transform: Transform):
        self.value = transform

    @ classmethod
    def identity(cls):
        return cls(Transform.identity())


class Mesh:
    vertices: List[Vector]
    triangle_indices: List[List[int]]
    pose: Pose
    mesh_indices_for_area2d_points: List[int]

    def __init__(self, vertices: List[Vector], triangle_indices: List[List[int]], pose: Pose):
        self.vertices = vertices
        self.triangle_indices = triangle_indices
        self.pose = pose

    def transform(self, transform: Transform):
        # transforms the mesh vertices with the given transform
        vertices = [vertex.transform(transform) for vertex in self.vertices]
        return Mesh(vertices, self.triangle_indices, self.pose)

    def to_stl_mesh(self):
        mesh_stl = stl.mesh.Mesh(
            np.zeros(len(self.triangle_indices), dtype=stl.mesh.Mesh.dtype))
        for i, f in enumerate(self.triangle_indices):
            for j in range(3):
                vector = self.vertices[f[j]]
                mesh_stl.vectors[i][j] = vector.value[:3]
        mesh_stl.update_normals()
        return mesh_stl

    def to_pygeos_area(self, precision):
        """"Convert the Mesh into 2D pygeos area and remove the triangles
        that only defines a line or a point"""
        # ignores y-values!!
        # 5. Calculate the union of all triangles
        polygons = []
        for triangle in self.triangle_indices:
            # if any of two of vertex are at the same point in x_z plane, then this triangle is
            # discarded.
            condition_1 = np.sqrt(np.square(self.vertices[triangle[0]].x() - self.vertices[triangle[1]].x()) +
                                  np.square(self.vertices[triangle[0]].z() - self.vertices[triangle[1]].z())) \
                < precision

            condition_2 = np.sqrt(np.square(self.vertices[triangle[0]].x() - self.vertices[triangle[2]].x()) +
                                  np.square(self.vertices[triangle[0]].z() - self.vertices[triangle[2]].z())) \
                < precision

            condition_3 = np.sqrt(np.square(self.vertices[triangle[1]].x() - self.vertices[triangle[2]].x()) +
                                  np.square(self.vertices[triangle[1]].z() - self.vertices[triangle[2]].z())) \
                < precision

            # After the projection, it Triangle becomes a line, it also need to be neglected
            # The cases where x_values or z_values are all the same.
            condition_4 = self.vertices[triangle[0]].x(
            ) == self.vertices[triangle[1]].x() == self.vertices[triangle[2]].x()

            condition_5 = self.vertices[triangle[0]].z(
            ) == self.vertices[triangle[1]].z() == self.vertices[triangle[2]].z()

            if condition_1 or condition_2 or condition_3 or condition_4 or condition_5:
                continue

            tuple_pair = [(self.vertices[triangle[0]].x(), self.vertices[triangle[0]].z()),
                          (self.vertices[triangle[1]].x(),
                           self.vertices[triangle[1]].z()),
                          (self.vertices[triangle[2]].x(), self.vertices[triangle[2]].z())]

            polygons.append(pygeos.polygons(tuple_pair))

        try:
            area = pygeos.constructive.simplify(
                pygeos.set_operations.union_all(polygons), tolerance=1E-9)

            # Remove polygon points which have not descended from mesh vertices
            area_points_corresponding_to_mesh_points = []
            for point in pygeos.get_coordinates(area):
                for vertex in self.vertices:
                    if point[0] == vertex.x() and point[1] == vertex.z():
                        area_points_corresponding_to_mesh_points.append(point)
                        break
            area2d = pygeos.polygons(
                area_points_corresponding_to_mesh_points)
        except pygeos.GEOSException:
            print(
                'pygeos.GEOSException is encountered, retrying union_all with 1E-3 grid size.')
            area = pygeos.constructive.simplify(
                pygeos.set_operations.union_all(polygons, grid_size=1E-3), tolerance=1E-9)

            # Remove polygon points which have not descended from mesh vertices
            area_points_corresponding_to_mesh_points = []
            for point in pygeos.get_coordinates(area):
                for vertex in self.vertices:
                    if point[0] == np.round(vertex.x(), 3) and point[1] == np.round(vertex.z(), 3):
                        area_points_corresponding_to_mesh_points.append(point)
                        break
            area2d = pygeos.polygons(
                area_points_corresponding_to_mesh_points)
        except RuntimeWarning:
            print(polygons)
            raise Exception('A RuntimeWarning is generated here.')
        else:
            pass

        return area2d


class Grain:
    """This holds the grain class. As the basic unit for calculation, It should have the following
    attributes:
    pose: a 4x4 matrix to represent orientation and position
    orthogonal projected area: as 3x3 matrix
    material:
    representation: mesh/3D Polygon/ 2D Polygon
    """
    mesh: Mesh
    pose: Pose

    def __init__(self, pose: Pose, mesh: Mesh):

        self.mesh = mesh
        self.pose = pose

    @ classmethod
    def from_stl(cls, path: str, grain_pose: Pose, mesh_pose: Pose):
        stl_mesh = stl.mesh.Mesh.from_file(path)
        vertices = []
        triangle_indices = []
        for face in stl_mesh.vectors:
            vertices.extend((tuple(face[0]), tuple(face[1]), tuple(face[2])))
            triangle_indices.append(
                (tuple(face[0]), tuple(face[1]), tuple(face[2])))
        unique_vertices = list(set(vertices))
        new_indices = [[unique_vertices.index(triangle[i])
                        for i in range(3)] for triangle in triangle_indices]
        new_vertices = [Vector(vertex[0], vertex[1], vertex[2])
                        for vertex in unique_vertices]
        our_mesh = Mesh(new_vertices, new_indices, mesh_pose)
        return cls(grain_pose, our_mesh)

    def get_mesh(self):
        return self.mesh
# ---------------------------------------------------------------------------------------


# --------------------------- Project a mesh onto x-z plane -----------------------------
def project_mesh_onto(mesh: Mesh, projection_direction: Vector) -> Mesh:
    '''Project mesh in plane frame onto the plane. Input mesh should be represented in manifold(slice) frame'''
    # 2. Project triangles x-z plane in manifold frame
    projection = Transform.from_projection_along_vector_to_x_z_plane(
        projection_direction)
    projected_mesh = mesh.transform(projection)

    return projected_mesh
# ---------------------------------------------------------------------------------------


# ------------------------------ Sample mesh projection ---------------------------------
path = r"./Resources/Grains/Cuboctahedron.stl"
grain = Grain.from_stl(path, Pose.identity(), Pose.identity())
grain_mesh = grain.get_mesh()

new_mesh = project_mesh_onto(grain_mesh, Vector(1, 0.5, 0.5))
new_stl_mesh = new_mesh.to_stl_mesh()
new_stl_mesh.save(r"./Resources/Grains/projected_cuboctahedron.stl")

area_2d = new_mesh.to_pygeos_area(1E-5)

polygon_points = pygeos.coordinates.get_coordinates(area_2d)
polygon_x = [
    point[0] for point in polygon_points]
polygon_z = [
    point[1] for point in polygon_points]
plt.plot(*[polygon_x, polygon_z],
         label='Projected area')
plt.show()
