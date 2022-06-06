import random

import numpy as np
import trimesh
from scipy.spatial import ConvexHull
from shapely.geometry import LineString

import FractureWear.geometrics as geom
import FractureWear.helpers as hlp
from FractureWear.geometrics import Vector
from FractureWear.grain import Grain2D, Grain3D

ex = np.array([1, 0], dtype=float)  # Unit vector in x-direction
ey = np.array([0, 1], dtype=float)  # Unit vector in y-direction


def get_contact_length(grain: Grain2D) -> float:
    """Calculates grain-chip contact length

    Args:
        grain (Grain2D): Instance of Grain2D

    Returns:
        float: Returns contact grain-chip contact length
    """
    cutting_rectangle = hlp.cuttingRectangle(grain)
    cutting_line = [cutting_rectangle[1], cutting_rectangle[0]]
    intersection = hlp.line_polygon_intersection(
        grain.vertices[-1].tolist(), cutting_line
    )  # Intersection between grain and workpiece
    if intersection == 0:
        return 0
    if intersection[0][0] < intersection[1][0]:
        return np.linalg.norm(
            geom.get_direction_vector(grain.cutting_edge, intersection[0])
        )
    return np.linalg.norm(
        geom.get_direction_vector(grain.cutting_edge, intersection[1])
    )


def align_grain(grain: Grain3D, cutting_direction: Vector) -> trimesh.base.Trimesh:
    """Rotates the grain around the z-axes so that the moving direction points at the negativ x-direction.

    Args:
        grain (Grain3D): Instance of Grain3D which has to be align
        cutting_direction (Vector): Moving direction of the grain

    Returns:
        trimesh.base.Trimesh: align mesh of type Trimesh
    """
    cutting_direction.project_onto("xy")
    rotation_angle = np.pi - Vector(1, 0, 0).angle_xy(cutting_direction)
    rotated_mesh = geom.mesh_rotation(grain.mesh, rotation_angle, "z")
    return rotated_mesh, rotation_angle


def project_grain_on_xz(mesh: trimesh.base.Trimesh) -> np.ndarray:
    """Projects grain on the xz plane by setting the y-values equal to zero

    Args:
        mesh (trimesh.base.Trimesh): Mesh to be projected

    Returns:
        np.ndarray: Polygon of the projection
    """
    projected_polygon = []
    for vertice in mesh.vertices:
        new_vertice = [vertice[0], vertice[2]]
        projected_polygon.append(new_vertice)
    hull = ConvexHull(projected_polygon)
    final_polygon = [projected_polygon[idx] for idx in hull.vertices]
    return np.array([np.array(x) for x in final_polygon], dtype=object)


def make_crack(grain: Grain2D) -> np.ndarray:
    """Calculates crack initiation point and direction of the crack.

    Args:
        grain (Grain2D): Instance of Grain2D which has to be fractured

    Returns:
        np.ndarray: crack_initiaion_point (crack_origin),
        np.ndarray: normal vector to the crack direction
    """
    contact_length = get_contact_length(grain)
    if contact_length == 0:
        print("contact_length is 0")
        return 0, 0
    initiation_point = LineString(grain.vertices[0][:]).interpolate(
        random.uniform(2, 3) * contact_length
    )
    initiation_point = list(initiation_point.coords)
    initiation_point = np.array([initiation_point[0][0], initiation_point[0][1]])
    crack = initiation_point + contact_length * ex
    theta = random.uniform(np.pi / 10, np.pi / 4)
    crack_point = geom.point_rotation(crack, theta, initiation_point)
    crack_direction = geom.get_direction_vector(initiation_point, crack_point)
    crack_normal = geom.unit_vector(
        np.array([-crack_direction[1], 0, crack_direction[0]])
    )
    crack_origin = np.array([initiation_point[0], 0, initiation_point[1]])
    return crack_origin, crack_normal


def get_new_vertices(
    old_mesh: trimesh.base.Trimesh,
    additional_vertices: np.ndarray,
    plane_origin: np.ndarray,
    plane_normal: np.ndarray,
) -> np.ndarray:
    """Finds the new vertices of a cracked grain. Searches all vertices that lie 'over' the cutting plane
    Args:
        old_mesh (trimesh.base.Trimesh): Unfractured mesh
        additional_vertices (np.ndarray): cutting polygon
        plane_origin (np.ndarray): cutting plane origin
        plane_normal (np.ndarray): cutting plane normal vector

    Returns:
        np.ndarray: array of vertices of the fractured grain
    """
    old_vertices = old_mesh.vertices.tolist()
    additional_vertices = additional_vertices.tolist()
    new_vertices = []
    coefficients = geom.get_plane_coefficents(plane_origin, plane_normal)
    ref_point = np.array([old_mesh.bounds[0][0], 0, old_mesh.bounds[1][2]])
    if hlp.plane_side(ref_point, coefficients) > 0:
        new_vertices.extend(
            point for point in old_vertices if hlp.plane_side(point, coefficients) > 0
        )

    else:
        new_vertices.extend(
            point for point in old_vertices if hlp.plane_side(point, coefficients) < 0
        )

    new_vertices.extend(iter(additional_vertices))
    return np.array([np.array(x) for x in new_vertices], dtype=object)


def generate_mesh(vertices: np.ndarray) -> trimesh.base.Trimesh:
    """Generates a mesh from a set of vertices
    (works only for convex objects)
    Args:
        vertices (np.ndarray): vertices of the new mesh

    Returns:
        trimesh.base.Trimesh: Mesh of type Trimesh
    """
    hull = ConvexHull(vertices)
    indices = hull.simplices
    return trimesh.Trimesh(vertices=vertices, faces=indices)
